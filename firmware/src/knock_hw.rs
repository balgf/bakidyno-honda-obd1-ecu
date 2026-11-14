//! `STM32H743VIT6` Knock Window hardware driver (`ADC3` `PC2_C` + `TIM6` `200 kS/s` + `TIM7` delay
//! + `DMAMUX2`/`BDMA_CH0` -> `D3` `SRAM4` ping-pong buffers).
//!
//! Hardware architecture (`_DOCS/stm32h743_pinout.md`):
//! - Input: `PC2_C` (`ADC3_INP0`, Channel `0`, direct pad switch `SYSCFG_PMCR.PC2SO = 1`).
//! - Pacer: `TIM6_TRGO` (`240 MHz / 1200 = 200.000 kHz`, `EXTSEL = 13`, `EXTEN = RISING_EDGE`).
//! - Delay: `TIM7` (`1.000 MHz` one-pulse) arms `TIM6 + ADC3 + BDMA_CH0` at `TDC + window_start`.
//! - DMA: `DMAMUX2` channel `0` (`dmareq_id = 17` = `ADC3`) -> `BDMA_CH0` into `.sram4` (`0x3800_0000`).

use core::cell::{RefCell, UnsafeCell};
use core::future::poll_fn;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering, compiler_fence, fence};
use core::task::Poll;

use defmt::info;
use ecu_core::knock::{KnockWindowPlan, MAX_KNOCK_WINDOW_SAMPLES, MIN_KNOCK_WINDOW_SAMPLES};
use ecu_types::Cylinder;
use embassy_stm32::interrupt::typelevel;
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::pac::adc::vals::{
    Adcaldif, Adstp, Boost, Difsel, Dmngt, Exten, Ovrmod, Pcsel, Res,
};
use embassy_stm32::pac::adccommon::vals::Presc;
use embassy_stm32::pac::bdma::vals::{Dir, Pl, Size};
use embassy_stm32::pac::gpio::vals::{Moder, Pupdr};
use embassy_stm32::pac::timer::regs::SrCore;
use embassy_stm32::pac::timer::vals::{Mms, Urs};
use embassy_stm32::{Peri, interrupt, pac, peripherals, rcc};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::waitqueue::AtomicWaker;

use crate::ECU_STATE;

/// `DMAMUX2` request line for `ADC3` (`stm32h743vi`).
const DMAMUX2_REQ_ADC3: u8 = 17;
/// `ADC3` `CFGR.EXTSEL` index for `TIM6_TRGO` (`0b01101`).
const ADC3_EXTSEL_TIM6_TRGO: u8 = 13;
/// `240 MHz / 1200 = 200.000 kHz` (`5.000 µs` per `ADC3` sample).
const TIM6_200KHZ_ARR: u16 = 1_199;
/// `240 MHz / 240 = 1.000 MHz` (`1 µs` per `TIM7` tick).
const TIM7_1MHZ_PSC: u16 = 239;
/// Safety timeout (`µs`) after which a wedged capture is reset automatically.
const CAPTURE_WATCHDOG_US: u32 = 20_000;

#[repr(C, align(4))]
struct Sram4PingPong {
    bufs: UnsafeCell<[[u16; MAX_KNOCK_WINDOW_SAMPLES]; 2]>,
}

// SAFETY: `CAPTURE_STATE` ensures mutually exclusive access: `BDMA_CH0` writes only `write_buf`
// while `phase == CapturePhase::Capturing`, and `knock_dsp_task` reads only `ready_window.buf_idx`
// which cannot become `write_buf` until `ready_window` is consumed.
unsafe impl Sync for Sram4PingPong {}

#[unsafe(link_section = ".sram4")]
static KNOCK_SRAM4_BUFS: Sram4PingPong = Sram4PingPong {
    bufs: UnsafeCell::new([[0u16; MAX_KNOCK_WINDOW_SAMPLES]; 2]),
};

#[derive(Clone, Copy, PartialEq, Eq)]
enum CapturePhase {
    Idle,
    ArmedDelay,
    Capturing,
}

#[derive(Clone, Copy)]
struct ActiveCapture {
    cylinder: Cylinder,
    rpm: u16,
    sample_count: u16,
    buf_idx: usize,
    started_tim2_us: u32,
}

#[derive(Clone, Copy)]
struct ReadyWindow {
    buf_idx: usize,
    cylinder: Cylinder,
    rpm: u16,
    sample_count: u16,
}

struct CaptureState {
    phase: CapturePhase,
    write_buf: usize,
    active: Option<ActiveCapture>,
    ready: Option<ReadyWindow>,
}

impl CaptureState {
    const fn new() -> Self {
        Self {
            phase: CapturePhase::Idle,
            write_buf: 0,
            active: None,
            ready: None,
        }
    }
}

static CAPTURE_STATE: Mutex<CriticalSectionRawMutex, RefCell<CaptureState>> =
    Mutex::new(RefCell::new(CaptureState::new()));
static KNOCK_WAKER: AtomicWaker = AtomicWaker::new();
static KNOCK_HW_ENABLED: AtomicBool = AtomicBool::new(true);
static CAPTURE_OVERRUNS: AtomicU32 = AtomicU32::new(0);

/// Owned hardware resources for [`KnockHw`].
pub struct KnockPeripherals {
    pub adc3: Peri<'static, peripherals::ADC3>,
    pub bdma_ch0: Peri<'static, peripherals::BDMA_CH0>,
    pub tim6: Peri<'static, peripherals::TIM6>,
    pub tim7: Peri<'static, peripherals::TIM7>,
    pub pc2: Peri<'static, peripherals::PC2>,
}

/// Hardware Knock Window capture engine handle retained in `main`.
pub struct KnockHw {
    _adc3: Peri<'static, peripherals::ADC3>,
    _bdma_ch0: Peri<'static, peripherals::BDMA_CH0>,
    _tim6: Peri<'static, peripherals::TIM6>,
    _tim7: Peri<'static, peripherals::TIM7>,
    _pc2: Peri<'static, peripherals::PC2>,
}

impl KnockHw {
    /// Initializes `D3` `SRAM4` ping-pong buffers, `ADC3` (`PC2_C`), `TIM6` (`200 kHz`), `TIM7`,
    /// and `DMAMUX2`/`BDMA_CH0`.
    pub fn init(
        p: KnockPeripherals,
        _irqs: impl typelevel::Binding<typelevel::TIM7, Tim7KnockInterruptHandler>
        + typelevel::Binding<typelevel::BDMA_CHANNEL0, BdmaCh0KnockInterruptHandler>,
    ) -> Self {
        let base_ptr = KNOCK_SRAM4_BUFS.bufs.get() as usize;
        let total_bytes = core::mem::size_of::<[[u16; MAX_KNOCK_WINDOW_SAMPLES]; 2]>();
        defmt::assert!(
            base_ptr >= 0x3800_0000 && base_ptr + total_bytes <= 0x3801_0000,
            "Knock BDMA buffers must reside inside D3 SRAM4 (0x3800_0000..0x3801_0000)"
        );

        // Zero NOLOAD `.sram4` ping-pong storage deterministically on cold boot.
        // SAFETY: Initialization runs single-threaded before interrupts/BDMA are armed.
        unsafe {
            core::ptr::write_bytes(
                KNOCK_SRAM4_BUFS.bufs.get().cast::<u16>(),
                0,
                2 * MAX_KNOCK_WINDOW_SAMPLES,
            );
        }

        // Keep `PC2` analog + no pulls (`SYSCFG_PMCR.PC2SO = 1` routes `PC2_C` to `ADC3_INP0`).
        pac::RCC.ahb4enr().modify(|w| {
            w.set_gpiocen(true);
            w.set_adc3en(true);
            w.set_bdmaen(true);
        });
        pac::GPIOC.moder().modify(|w| w.set_moder(2, Moder::ANALOG));
        pac::GPIOC
            .pupdr()
            .modify(|w| w.set_pupdr(2, Pupdr::FLOATING));

        rcc::enable_and_reset::<peripherals::ADC3>();
        rcc::enable_and_reset::<peripherals::BDMA>();
        rcc::enable_and_reset::<peripherals::TIM6>();
        rcc::enable_and_reset::<peripherals::TIM7>();

        init_adc3_knock_channel();
        init_tim6_200khz_pacer();
        init_tim7_delay_timer();

        interrupt::typelevel::TIM7::set_priority(interrupt::Priority::P3);
        interrupt::typelevel::BDMA_CHANNEL0::set_priority(interrupt::Priority::P3);
        // SAFETY: NVIC handlers are bound via `Irqs` and synchronize via `CAPTURE_STATE`.
        unsafe {
            interrupt::typelevel::TIM7::unpend();
            interrupt::typelevel::TIM7::enable();
            interrupt::typelevel::BDMA_CHANNEL0::unpend();
            interrupt::typelevel::BDMA_CHANNEL0::enable();
        }

        info!(
            "KnockHw init: ADC3 PC2_C @ 200 kHz TIM6_TRGO -> BDMA_CH0 SRAM4 @ 0x{=usize:08x}",
            base_ptr
        );

        Self {
            _adc3: p.adc3,
            _bdma_ch0: p.bdma_ch0,
            _tim6: p.tim6,
            _tim7: p.tim7,
            _pc2: p.pc2,
        }
    }
}

/// Reflects runtime `KnockConfig::enabled` into hardware gating.
#[inline]
pub fn set_hw_enabled(enabled: bool) {
    KNOCK_HW_ENABLED.store(enabled, Ordering::Release);
    if !enabled {
        stop_hardware_capture();
        critical_section::with(|cs| {
            let mut st = CAPTURE_STATE.borrow(cs).borrow_mut();
            st.phase = CapturePhase::Idle;
            st.active = None;
        });
    }
}

/// Returns total hardware window/DMA overruns observed since boot.
#[inline]
pub fn overrun_count() -> u32 {
    CAPTURE_OVERRUNS.load(Ordering::Relaxed)
}

/// Schedules an `ADC3` combustion-window capture from a `TIM2` `CKP` tooth event.
pub fn schedule_window(plan: KnockWindowPlan) {
    if !KNOCK_HW_ENABLED.load(Ordering::Relaxed) {
        return;
    }

    let sample_count = plan.sample_count.clamp(
        MIN_KNOCK_WINDOW_SAMPLES as u16,
        MAX_KNOCK_WINDOW_SAMPLES as u16,
    );
    let target_start_us = plan
        .tooth_timestamp_us
        .wrapping_add(u32::from(plan.delay_from_tooth_us));
    let now_us = pac::TIM2.cnt().read();

    critical_section::with(|cs| {
        let mut st = CAPTURE_STATE.borrow(cs).borrow_mut();
        if st.phase != CapturePhase::Idle {
            let timed_out = st
                .active
                .is_some_and(|a| now_us.wrapping_sub(a.started_tim2_us) > CAPTURE_WATCHDOG_US);
            if timed_out {
                stop_hardware_capture();
                st.phase = CapturePhase::Idle;
                st.active = None;
                CAPTURE_OVERRUNS.fetch_add(1, Ordering::Relaxed);
            } else {
                CAPTURE_OVERRUNS.fetch_add(1, Ordering::Relaxed);
                return;
            }
        }

        // Skip capture if the alternate ping-pong slot is still queued for DSP.
        let buf_idx = st.write_buf;
        if st.ready.is_some_and(|r| r.buf_idx == buf_idx) {
            CAPTURE_OVERRUNS.fetch_add(1, Ordering::Relaxed);
            return;
        }

        st.active = Some(ActiveCapture {
            cylinder: plan.cylinder,
            rpm: plan.rpm,
            sample_count,
            buf_idx,
            started_tim2_us: now_us,
        });

        let rem_us = target_start_us.wrapping_sub(now_us) as i32;
        if rem_us <= 20 {
            st.phase = CapturePhase::Capturing;
            start_bdma_adc3_tim6(sample_count, buf_idx);
        } else {
            st.phase = CapturePhase::ArmedDelay;
            let delay_ticks = (rem_us as u32).clamp(1, 50_000) as u16;
            let t7 = pac::TIM7;
            t7.cr1().modify(|w| w.set_cen(false));
            t7.cnt().write(|w| w.set_cnt(0));
            t7.arr().write(|w| w.set_arr(delay_ticks));
            t7.sr().write_value(SrCore(0));
            t7.dier().write(|w| w.set_uie(true));
            t7.cr1().modify(|w| w.set_cen(true));
        }
    });
}

/// Background async task draining completed `.sram4` knock bursts into `ECU_STATE`.
#[embassy_executor::task]
pub async fn knock_dsp_task() -> ! {
    loop {
        let ready = poll_fn(|cx| {
            KNOCK_WAKER.register(cx.waker());
            critical_section::with(|cs| {
                let mut st = CAPTURE_STATE.borrow(cs).borrow_mut();
                match st.ready.take() {
                    Some(w) => Poll::Ready(w),
                    None => Poll::Pending,
                }
            })
        })
        .await;

        let len = (ready.sample_count as usize).min(MAX_KNOCK_WINDOW_SAMPLES);
        // Ensure CPU sees all BDMA writes before reading `.sram4` samples.
        fence(Ordering::Acquire);
        let mut local = [0u16; MAX_KNOCK_WINDOW_SAMPLES];
        // SAFETY: `BDMA_CH0` only targets `write_buf`, which was flipped to `ready.buf_idx ^ 1`
        // when `ready` was published, and `len <= MAX_KNOCK_WINDOW_SAMPLES`.
        unsafe {
            let base_ptr = KNOCK_SRAM4_BUFS.bufs.get().cast::<u16>();
            let src = core::slice::from_raw_parts(
                base_ptr.add(ready.buf_idx * MAX_KNOCK_WINDOW_SAMPLES),
                len,
            );
            local[..len].copy_from_slice(src);
        }

        let mut state = ECU_STATE.lock().await;
        state
            .ecu
            .process_knock_window(ready.cylinder, ready.rpm, &local[..len]);
    }
}

/// Interrupt handler for `TIM7` (`1 MHz` combustion-window start delay).
pub struct Tim7KnockInterruptHandler;

impl typelevel::Handler<typelevel::TIM7> for Tim7KnockInterruptHandler {
    /// # Safety
    ///
    /// Invoked exclusively by hardware `TIM7` NVIC IRQ; synchronizes via `CAPTURE_STATE`.
    unsafe fn on_interrupt() {
        let t7 = pac::TIM7;
        t7.sr().write_value(SrCore(0));
        t7.dier().write(|w| w.set_uie(false));
        t7.cr1().modify(|w| w.set_cen(false));

        critical_section::with(|cs| {
            let mut st = CAPTURE_STATE.borrow(cs).borrow_mut();
            if st.phase != CapturePhase::ArmedDelay {
                return;
            }
            if let Some(mut act) = st.active {
                act.started_tim2_us = pac::TIM2.cnt().read();
                st.active = Some(act);
                st.phase = CapturePhase::Capturing;
                start_bdma_adc3_tim6(act.sample_count, act.buf_idx);
            } else {
                st.phase = CapturePhase::Idle;
            }
        });
    }
}

/// Interrupt handler for `BDMA_CHANNEL0` (`ADC3` -> `.sram4` window completion).
pub struct BdmaCh0KnockInterruptHandler;

impl typelevel::Handler<typelevel::BDMA_CHANNEL0> for BdmaCh0KnockInterruptHandler {
    /// # Safety
    ///
    /// Invoked exclusively by hardware `BDMA_CHANNEL0` NVIC IRQ; synchronizes via `CAPTURE_STATE`.
    unsafe fn on_interrupt() {
        let bdma = pac::BDMA;
        let isr = bdma.isr().read();
        stop_hardware_capture();
        fence(Ordering::Release);

        let te = isr.teif(0);
        let tc = isr.tcif(0);

        critical_section::with(|cs| {
            let mut st = CAPTURE_STATE.borrow(cs).borrow_mut();
            let active = st.active.take();
            st.phase = CapturePhase::Idle;

            if te {
                CAPTURE_OVERRUNS.fetch_add(1, Ordering::Relaxed);
                return;
            }

            if tc && let Some(act) = active {
                if st.ready.is_some() {
                    CAPTURE_OVERRUNS.fetch_add(1, Ordering::Relaxed);
                } else {
                    st.ready = Some(ReadyWindow {
                        buf_idx: act.buf_idx,
                        cylinder: act.cylinder,
                        rpm: act.rpm,
                        sample_count: act.sample_count,
                    });
                    st.write_buf = act.buf_idx ^ 1;
                    KNOCK_WAKER.wake();
                }
            }
        });
    }
}

fn init_adc3_knock_channel() {
    // Kernel clock is `PLL3_R = 48.000 MHz` (`Adcsel::PLL3_R`); `DIV1` -> `48.0 MHz` <= `50 MHz`.
    pac::ADC3_COMMON.ccr().modify(|w| w.set_presc(Presc::DIV1));

    let adc3 = pac::ADC3;
    adc3.cr().modify(|w| {
        w.set_deeppwd(false);
        w.set_advregen(true);
    });
    cortex_m::asm::delay(480 * 20);

    adc3.cr().modify(|w| w.set_boost(Boost::LT50));

    for ch in 0..20 {
        adc3.difsel()
            .modify(|w| w.set_difsel(ch, Difsel::SINGLE_ENDED));
    }

    adc3.cr().modify(|w| {
        w.set_adcaldif(Adcaldif::SINGLE_ENDED);
        w.set_adcallin(true);
    });
    adc3.cr().modify(|w| w.set_adcal(true));
    while adc3.cr().read().adcal() {}
    cortex_m::asm::delay(480 * 2);

    adc3.isr().write(|w| w.set_adrdy(true));
    adc3.cr().modify(|w| w.set_aden(true));
    while !adc3.isr().read().adrdy() {}
    adc3.isr().write(|w| w.set_adrdy(true));

    // Configure regular sequence length = 1, Channel 0 (`PC2_C` `ADC3_INP0`), 64.5 cycle sample
    // time (~1.52 us total conversion @ 48 MHz, comfortably inside the 5.00 us `200 kS/s` slot).
    adc3.cfgr().modify(|w| {
        w.set_res(Res::BITS16);
        w.set_cont(false);
        w.set_ovrmod(Ovrmod::OVERWRITE);
        w.set_dmngt(Dmngt::DMA_ONE_SHOT);
        w.set_extsel(ADC3_EXTSEL_TIM6_TRGO);
        w.set_exten(Exten::RISING_EDGE);
    });
    adc3.cfgr2().modify(|w| {
        w.set_rovse(false);
        w.set_lshift(0);
    });
    adc3.sqr1().modify(|w| {
        w.set_l(0);
        w.set_sq(0, 0);
    });
    adc3.pcsel().modify(|w| w.set_pcsel(0, Pcsel::PRESELECTED));
    adc3.smpr(0)
        .modify(|w| w.set_smp(0, embassy_stm32::adc::SampleTime::CYCLES64_5));
}

fn init_tim6_200khz_pacer() {
    let t6 = pac::TIM6;
    t6.cr1().modify(|w| {
        w.set_cen(false);
        w.set_opm(false);
        w.set_arpe(false);
        w.set_urs(Urs::COUNTER_ONLY);
    });
    t6.psc().write_value(0);
    t6.arr().write(|w| w.set_arr(TIM6_200KHZ_ARR));
    t6.cr2().modify(|w| w.set_mms(Mms::UPDATE));
    t6.egr().write(|w| w.set_ug(true));
    t6.sr().write_value(SrCore(0));
}

fn init_tim7_delay_timer() {
    let t7 = pac::TIM7;
    t7.cr1().modify(|w| {
        w.set_cen(false);
        w.set_opm(true);
        w.set_arpe(false);
        w.set_urs(Urs::COUNTER_ONLY);
    });
    t7.psc().write_value(TIM7_1MHZ_PSC);
    t7.dier().write(|w| w.set_uie(false));
    t7.egr().write(|w| w.set_ug(true));
    t7.sr().write_value(SrCore(0));
}

fn stop_hardware_capture() {
    let t6 = pac::TIM6;
    t6.cr1().modify(|w| w.set_cen(false));
    t6.sr().write_value(SrCore(0));

    let t7 = pac::TIM7;
    t7.cr1().modify(|w| w.set_cen(false));
    t7.dier().write(|w| w.set_uie(false));
    t7.sr().write_value(SrCore(0));

    let adc3 = pac::ADC3;
    if adc3.cr().read().adstart() && !adc3.cr().read().addis() {
        adc3.cr().modify(|w| w.set_adstp(Adstp::STOP));
    }

    let bdma = pac::BDMA;
    bdma.ch(0).cr().modify(|w| w.set_en(false));
    bdma.ifcr().write(|w| {
        w.set_gif(0, true);
        w.set_tcif(0, true);
        w.set_htif(0, true);
        w.set_teif(0, true);
    });
}

fn start_bdma_adc3_tim6(sample_count: u16, buf_idx: usize) {
    let t6 = pac::TIM6;
    t6.cr1().modify(|w| w.set_cen(false));
    // Pre-load CNT near ARR so first `200 kHz` sample occurs promptly once window opens.
    t6.cnt()
        .write(|w| w.set_cnt(TIM6_200KHZ_ARR.saturating_sub(10)));
    t6.sr().write_value(SrCore(0));

    let adc3 = pac::ADC3;
    if adc3.cr().read().adstart() && !adc3.cr().read().addis() {
        adc3.cr().modify(|w| w.set_adstp(Adstp::STOP));
        for _ in 0..512 {
            if !adc3.cr().read().adstart() {
                break;
            }
        }
    }
    adc3.isr().write(|w| {
        w.set_eos(true);
        w.set_eoc(true);
        w.set_ovr(true);
    });

    let bdma = pac::BDMA;
    let ch = bdma.ch(0);
    ch.cr().modify(|w| w.set_en(false));
    for _ in 0..512 {
        if !ch.cr().read().en() {
            break;
        }
    }
    bdma.ifcr().write(|w| {
        w.set_gif(0, true);
        w.set_tcif(0, true);
        w.set_htif(0, true);
        w.set_teif(0, true);
    });

    pac::DMAMUX2.ccr(0).write(|w| {
        w.set_dmareq_id(DMAMUX2_REQ_ADC3);
    });

    // SAFETY: `buf_idx` is `< 2` and exclusively owned by BDMA while `phase == Capturing`.
    let dst_addr = unsafe {
        KNOCK_SRAM4_BUFS
            .bufs
            .get()
            .cast::<u16>()
            .add(buf_idx * MAX_KNOCK_WINDOW_SAMPLES) as u32
    };
    ch.par().write_value(adc3.dr().as_ptr() as u32);
    ch.mar().write_value(dst_addr);
    ch.ndtr().write(|w| w.set_ndt(sample_count));
    compiler_fence(Ordering::Release);

    ch.cr().write(|w| {
        w.set_dir(Dir::FROM_PERIPHERAL);
        w.set_circ(false);
        w.set_pinc(false);
        w.set_minc(true);
        w.set_psize(Size::BITS16);
        w.set_msize(Size::BITS16);
        w.set_pl(Pl::HIGH);
        w.set_tcie(true);
        w.set_teie(true);
        w.set_en(true);
    });

    adc3.cr().modify(|w| w.set_adstart(true));
    t6.cr1().modify(|w| w.set_cen(true));
}
