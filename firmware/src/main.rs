#![no_std]
#![no_main]

//! `STM32H743VIT6` (`LQFP-100`) ECU Firmware Entrypoint.
//!
//! Pin & peripheral assignments match `_DOCS/stm32h743_pinout.md` 1:1 across both
//! Stage 1 (Shopee WeAct `STM32H743VIT6` breadboard rig) and Stage 2 (Custom OBD1 PCB):
//! - `25.000 MHz` `HSE` -> `PLL1` `480 MHz` `SYSCLK` (`VoltageScale::Scale0`), `PLL3Q` `48 MHz` USB
//! - `PB15` (`ACTUATOR_OE_N`): Held HIGH (`OFF`) during boot/init; asserted LOW (`ENABLED`) after
//!   clocks, calibration header/CRC, and safe initial output states are verified!
//! - `SYSCFG_PMCR`: Opens `PC2SO` & `PC3SO` switches for zero-impedance `ADC3` `D3` knock sampling.

mod aux_io_hw;
mod can_hw;
mod dlc_hw;
mod knock_hw;
mod nvm;
mod outputs;
mod sd_logger_hw;
mod sensors_adc;
mod usb_comm;
mod wideband_hw;

use defmt::info;
use ecu_core::EngineCoordinator;
use ecu_core::aux_io::{AuxIoConfig, AuxIoController, AuxIoTelemetry};
use ecu_core::sensors::InvertedMcuSwitchPins;
use ecu_types::{CalibrationBlob, SyncState, WidebandState};
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::rcc::mux::{Adcsel, Fdcansel, Sdmmcsel, Usbsel};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, LsConfig, Pll, PllDiv, PllMul, PllPreDiv, PllSource,
    Sysclk, VoltageScale,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::input_capture::{CapturePin, InputCapture};
use embassy_stm32::timer::low_level::{CountingMode, InputCaptureMode, InputTISelection};
use embassy_stm32::{
    Config, bind_interrupts, can, dma, exti, interrupt, pac, peripherals, sdmmc, timer, usart, usb,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker};
use nvm::NvmFlash;
use outputs::HardwareOutputs;
use proto_tunerstudio::TS_PAGE_SIZE;
use sensors_adc::{Adc1SensorPins, Adc1Sensors};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    TIM2 => timer::CaptureCompareInterruptHandler<peripherals::TIM2>;
    TIM1_CC => outputs::Tim1CopInterruptHandler;
    TIM3 => outputs::Tim3InjInterruptHandler;
    TIM15 => wideband_hw::Tim15WidebandInterruptHandler;
    TIM7 => knock_hw::Tim7KnockInterruptHandler;
    BDMA_CHANNEL0 => knock_hw::BdmaCh0KnockInterruptHandler;
    DMA1_STREAM0 => dma::InterruptHandler<peripherals::DMA1_CH0>;
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
    USART1 => usart::BufferedInterruptHandler<peripherals::USART1>;
    EXTI9_5 => exti::InterruptHandler<interrupt::typelevel::EXTI9_5>;
    EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
    SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
});

pub(crate) struct SharedEcuState {
    pub(crate) ecu: EngineCoordinator,
    pub(crate) aux: AuxIoController,
    pub(crate) aux_cfg: AuxIoConfig,
    pub(crate) aux_telemetry: AuxIoTelemetry,
    pub(crate) cal: Option<CalibrationBlob>,
    vss_kmh: f32,
    last_vss_ts_us: Option<u32>,
}

pub(crate) static ECU_STATE: Mutex<CriticalSectionRawMutex, SharedEcuState> =
    Mutex::new(SharedEcuState {
        ecu: EngineCoordinator::new(),
        aux: AuxIoController::new(),
        aux_cfg: AuxIoConfig::default_honda_obd1(),
        aux_telemetry: AuxIoTelemetry {
            outputs: ecu_core::aux_io::AuxOutputs {
                altc_on: false,
                iab_on: false,
                evap_on: false,
                evap_duty_per_mille: 0,
                pe0_on: false,
                pe1_on: false,
            },
            flex: ecu_core::aux_io::FlexFuelState::new(),
            altf: ecu_core::aux_io::AltfState::new(),
        },
        cal: None,
        vss_kmh: 0.0,
        last_vss_ts_us: None,
    });

/// Configures the `STM32H743VIT6` clock tree from the `25.000 MHz` external quartz crystal:
/// - `SYSCLK` = `480 MHz` (`PLL1_P`: `25 MHz / 5 * 192 / 2`) in `VOS0` LDO mode
/// - `HCLK`   = `240 MHz`, `PCLK1..4` = `120 MHz` (`240 MHz` timer kernel -> exact `1 µs` prescale)
/// - `PLL3_Q` = `48.000 MHz` (`25 MHz / 5 * 48 / 5`) for jitter-free `USB_OTG_FS` (`PA11`/`PA12`)
fn configure_h743_clocks() -> Config {
    let mut config = Config::default();
    config.rcc.voltage_scale = VoltageScale::Scale0;
    config.rcc.hse = Some(Hse {
        freq: Hertz(25_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV5,
        mul: PllMul::MUL192,
        fracn: None,
        divp: Some(PllDiv::DIV2), // 480 MHz CPU
        divq: Some(PllDiv::DIV8), // 120 MHz SPI1 / FDCAN1 / SDMMC1
        divr: Some(PllDiv::DIV4), // 240 MHz
    });
    config.rcc.pll3 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV5,
        mul: PllMul::MUL48,
        fracn: None,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV5), // Exact 48.000 MHz USB_OTG_FS
        divr: Some(PllDiv::DIV5),
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV2;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.apb3_pre = APBPrescaler::DIV2;
    config.rcc.apb4_pre = APBPrescaler::DIV2;
    config.rcc.ls = LsConfig::default_lse();
    config.rcc.mux.usbsel = Usbsel::PLL3_Q;
    config.rcc.mux.adcsel = Adcsel::PLL3_R;
    config.rcc.mux.fdcansel = Fdcansel::PLL1_Q;
    config.rcc.mux.sdmmcsel = Sdmmcsel::PLL1_Q;
    config
}

/// Opens internal analog switches on `PC2_C` (`ADC3_INP0` Knock) and `PC3_C` (`ADC3_INP1` Aux)
/// per `_DOCS/stm32h743_pinout.md` Section 5 Rule 3.
fn isolate_direct_adc3_pads() {
    pac::RCC
        .apb4enr()
        .modify(|w| w.set_syscfgen(/* syscfgen= */ true));
    pac::SYSCFG.pmcr().modify(|w| {
        w.set_pc2so(/* pc2so= */ true);
        w.set_pc3so(/* pc3so= */ true);
    });
}

/// Hard real-time `TIM2` (`1.000 MHz` 32-bit Input Capture) task decoding the Honda OBD1 `24/4/1`
/// distributor signals (`PA15` `CKP`, `PA1` `TDC`, `PB10` `CYP`) and `PB11` `VSS`.
#[embassy_executor::task]
async fn tim2_trigger_task(mut tim2_ic: InputCapture<'static, peripherals::TIM2>) {
    // Configure CH2 (`PA1 TDC`), CH3 (`PB10 CYP`), and CH4 (`PB11 VSS`) for hardware rising-edge
    // timestamp latching into `TIM2->CCR2..4` without firing redundant IRQs before `CKP`.
    for ch in [Channel::Ch2, Channel::Ch3, Channel::Ch4] {
        tim2_ic.set_input_ti_selection(ch, InputTISelection::Normal);
        tim2_ic.set_input_capture_mode(ch, InputCaptureMode::Rising);
        tim2_ic.enable(ch);
    }
    pac::TIM2.sr().write(|_| {});

    loop {
        // Note: `embassy-stm32` `InputCaptureFuture::poll` reads `CCR` through `TimGp16` (16-bit),
        // so we read the full 32-bit `TIM2->CCR1` via `get_capture_value(Channel::Ch1)` on wakeup.
        let _ = tim2_ic.wait_for_rising_edge(Channel::Ch1).await;
        let ckp_ts_us = tim2_ic.get_capture_value(Channel::Ch1);
        let sr = pac::TIM2.sr().read();

        let mut state = ECU_STATE.lock().await;
        if sr.ccif(2) {
            let cyp_ts_us = tim2_ic.get_capture_value(Channel::Ch3);
            state.ecu.on_cyp_capture(cyp_ts_us);
        }
        if sr.ccif(1) {
            let tdc_ts_us = tim2_ic.get_capture_value(Channel::Ch2);
            state.ecu.on_tdc_capture(tdc_ts_us);
        }
        if sr.ccif(3) {
            let vss_ts_us = tim2_ic.get_capture_value(Channel::Ch4);
            if let Some(prev_vss) = state.last_vss_ts_us {
                let dt_vss = vss_ts_us.wrapping_sub(prev_vss);
                if dt_vss > 1_000 {
                    state.vss_kmh = 1_448_000.0 / (dt_vss as f32);
                }
            }
            state.last_vss_ts_us = Some(vss_ts_us);
        }
        pac::TIM2.sr().modify(|w| {
            for ch in 0..4 {
                w.set_ccof(ch, /* ccof= */ false);
            }
        });

        let state_ref = &mut *state;
        if let Some(ref cal) = state_ref.cal {
            let tooth_out = state_ref.ecu.on_ckp_capture(ckp_ts_us, cal);
            if let Some(ev) = tooth_out.event {
                HardwareOutputs::schedule_tooth_outputs(ckp_ts_us, &tooth_out.cop, &tooth_out.inj);
                if let Some(plan) = state_ref.ecu.knock.plan_on_ckp_tooth(&ev) {
                    knock_hw::schedule_window(plan);
                }
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(configure_h743_clocks());
    isolate_direct_adc3_pads();

    // 1. SAFETY FIRST: Hold `PB15` (`ACTUATOR_OE_N` on 3x `SN74AHCT541`) HIGH (`DISABLED`)
    //    and initialize `SPI1` NOR Flash (`PD6` CS# held HIGH) before loading tune!
    let mut actuator_oe_n = Output::new(p.PB15, Level::High, Speed::Low);
    let mut nvm = NvmFlash::init(p.SPI1, p.PB3, p.PD7, p.PB4, p.PD6).await;

    // 2. Initialize digital outputs to safe OFF state BEFORE enabling `ACTUATOR_OE_N`
    let mut heartbeat_led = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut vts_high_en = Output::new(p.PA8, Level::Low, Speed::Medium); // A4 VTS* +12V High-Side
    let mut flr_out = Output::new(p.PD3, Level::Low, Speed::Low); // A7/A8 Fuel Pump Relay
    let mut mil_out = Output::new(p.PD5, Level::Low, Speed::Low); // A13 Check Engine Light
    let mut fanc_out = Output::new(p.PB12, Level::Low, Speed::Low); // A12 Radiator Fan Relay
    let mut acc_out = Output::new(p.PB13, Level::Low, Speed::Low); // A15 A/C Clutch Relay
    let mut altc_out = Output::new(p.PD8, Level::Low, Speed::Low); // A16 Dual-mode Alternator
    let mut iab_out = Output::new(p.PD9, Level::Low, Speed::Low); // A17 Intake Runner Butterfly
    let mut evap_out = Output::new(p.PD10, Level::Low, Speed::Low); // A20 EVAP Purge
    let mut egr_a18_out = Output::new(p.PE0, Level::Low, Speed::Low); // A11 EGR* / A18 Spare
    let mut var_a19_out = Output::new(p.PE1, Level::Low, Speed::Low); // A19 VAR* Spare

    // 3. Initialize `SN74HCS14` conditioned switch inputs & `TPS1H100` fault input
    //    (Weak internal pulls keep unconnected bench pins in deterministic idle-safe states.)
    let sts_in = Input::new(p.PB5, Pull::Up); // B9 STS Cranking (`HIGH` = not cranking)
    let vtp_in = Input::new(p.PE4, Pull::Up); // D6 VTP* Oil Pressure (`HIGH` = unpressurized)
    let scs_in = Input::new(p.PE6, Pull::Down); // D4 SCS Service Check (`LOW` = jumper open)
    let acs_in = Input::new(p.PE7, Pull::Down); // B5 ACS* A/C Request (`LOW` = off)
    let psp_in = Input::new(p.PE8, Pull::Down); // B8 PSP* Power Steering (`LOW` = off)
    let bksw_in = Input::new(p.PE10, Pull::Up); // D2 BKSW* Brake Switch (`HIGH` = released)
    let clutch_in = Input::new(p.PE12, Pull::Down); // B7 Clutch Switch (`LOW` = released)
    let vts_fault_n = Input::new(p.PB14, Pull::Up); // TPS1H100A-Q1 Active-Low Fault

    // 4. Initialize 32-bit `TIM2` (`1.000 MHz`) Input Capture & `TIM1/TIM3/TIM16/TIM4` outputs
    let ckp_pin = CapturePin::new(p.PA15, Pull::Down); // B15/B16 CKP (24 teeth / 720°)
    let tdc_pin = CapturePin::new(p.PA1, Pull::Down); // B13/B14 TDC (4 teeth / 720°)
    let cyp_pin = CapturePin::new(p.PB10, Pull::Down); // B11/B12 CYP (1 tooth / 720°)
    let vss_pin = CapturePin::new(p.PB11, Pull::Down); // B10 VSS Vehicle Speed
    let tim2_ic = InputCapture::new(
        p.TIM2,
        Some(ckp_pin),
        Some(tdc_pin),
        Some(cyp_pin),
        Some(vss_pin),
        Irqs,
        Hertz(1_000_000),
        CountingMode::EdgeAlignedUp,
    );
    let mut hw_outputs = HardwareOutputs::init(
        p.TIM1, p.PE9, p.PE11, p.PE13, p.PE14, p.TIM3, p.PC6, p.PC7, p.PB0, p.PB1, p.TIM16, p.PB8,
        p.TIM4, p.PD14, p.PD15, Irqs, Irqs,
    );
    let mut adc_sensors = Adc1Sensors::new(
        p.ADC1,
        p.DMA1_CH0,
        Adc1SensorPins {
            map_pc0: p.PC0,
            tps_pc1: p.PC1,
            ect_pa2: p.PA2,
            iat_pa3: p.PA3,
            vbatt_pc4: p.PC4,
            vcc_5v_pa0: p.PA0,
            eld_pc5: p.PC5,
        },
    );
    // Must follow `Adc1Sensors::new(...)` so `ADC12RST` is never pulsed after `ADC1` init.
    let mut wideband_hw = wideband_hw::WidebandHw::init(wideband_hw::WidebandPeripherals {
        dac1: p.DAC1,
        pa4_ip_dac: p.PA4,
        pa5_vm_dac: p.PA5,
        adc2: p.ADC2,
        pa6_un_sense: p.PA6,
        pa7_ip_sense: p.PA7,
        tim15: p.TIM15,
        pe5_nernst_ac: p.PE5,
        tim17: p.TIM17,
        pb9_heater_pwm: p.PB9,
    });
    let _knock_hw = knock_hw::KnockHw::init(
        knock_hw::KnockPeripherals {
            adc3: p.ADC3,
            bdma_ch0: p.BDMA_CH0,
            tim6: p.TIM6,
            tim7: p.TIM7,
            pc2: p.PC2,
        },
        Irqs,
    );

    // 5. Load calibration from `SPI1` NOR Flash (or default map) & spawn trigger + USB CDC tasks
    let mut boot_page_buf = [0u8; TS_PAGE_SIZE];
    let cal = nvm.load_calibration(&mut boot_page_buf).await;
    defmt::assert!(cal.validate_header());
    proto_tunerstudio::pack_page(&cal, &mut boot_page_buf);
    sd_logger_hw::update_cal_crc(ecu_types::crc32(&boot_page_buf));
    ECU_STATE.lock().await.cal = Some(cal);

    let jid = nvm.jedec_id();
    let usb_init = usb_comm::init_cdc_acm(p.USB_OTG_FS, Irqs, p.PA12, p.PA11);
    let (can_tx, can_rx, can_props) = can_hw::init_and_selftest(p.FDCAN1, p.PD0, p.PD1).await;
    let dlc_uart = dlc_hw::init_dlc_uart(p.USART1, p.PA10, p.PA9);
    let altf_exti = aux_io_hw::init_altf_input(p.PB7, p.EXTI7, Irqs);
    let flex_exti = aux_io_hw::init_flex_input(p.PE15, p.EXTI15, Irqs);
    let (sdmmc, pd4_detect, mut log_mark_btn) = sd_logger_hw::init_hardware(
        sd_logger_hw::SdLoggerPeripherals {
            sdmmc1: p.SDMMC1,
            pc12_ck: p.PC12,
            pd2_cmd: p.PD2,
            pc8_d0: p.PC8,
            pc9_d1: p.PC9,
            pc10_d2: p.PC10,
            pc11_d3: p.PC11,
            pd4_detect: p.PD4,
            pc13_key: p.PC13,
            rtc: p.RTC,
        },
        Irqs,
    );
    {
        let state = ECU_STATE.lock().await;
        aux_io_hw::set_capture_polarity(
            state.aux_cfg.flex_active_low_pulse,
            state.aux_cfg.altf_active_low_duty,
        );
    }

    info!(
        "Civic OBD1 STM32H743VIT6 ECU booted @ 480 MHz | USB_OTG_FS CDC + SPI1 NVM | \
         flash={} jedec={=u8:02x}{=u8:02x}{=u8:02x}",
        nvm.is_detected(),
        jid[0],
        jid[1],
        jid[2]
    );
    spawner.spawn(tim2_trigger_task(tim2_ic).expect("valid tim2_trigger_task token"));
    spawner
        .spawn(usb_comm::usb_device_task(usb_init.usb_dev).expect("valid usb_device_task token"));
    spawner.spawn(
        usb_comm::usb_protocol_task(usb_init.cdc, nvm).expect("valid usb_protocol_task token"),
    );
    spawner.spawn(can_hw::can_tx_task(can_tx, can_props).expect("valid can_tx_task token"));
    spawner.spawn(can_hw::can_rx_task(can_rx).expect("valid can_rx_task token"));
    spawner.spawn(aux_io_hw::exti7_altf_task(altf_exti).expect("valid exti7_altf_task token"));
    spawner.spawn(aux_io_hw::exti15_flex_task(flex_exti).expect("valid exti15_flex_task token"));
    if let Some(uart) = dlc_uart {
        spawner.spawn(dlc_hw::dlc_uart_task(uart).expect("valid dlc_uart_task token"));
    }
    spawner.spawn(sd_logger_hw::sd_logger_task(sdmmc, pd4_detect).expect("valid sd_logger token"));
    spawner.spawn(knock_hw::knock_dsp_task().expect("valid knock_dsp_task token"));

    // 6. All outputs verified safe -> Assert `PB15` (`ACTUATOR_OE_N`) LOW to arm low-side buffers!
    actuator_oe_n.set_low();

    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut tick_count: u32 = 0;

    loop {
        ticker.next().await;
        tick_count = tick_count.wrapping_add(1);
        log_mark_btn.poll_1khz();

        let mut raw_adc = adc_sensors.read_raw(Irqs).await;
        wideband_hw.read_adc_into(&mut raw_adc);

        let switches = InvertedMcuSwitchPins {
            pb5_sts_pin_high: sts_in.is_high(),
            pe4_vtp_pin_high: vtp_in.is_high(),
            pe6_scs_pin_high: scs_in.is_high(),
            pe7_acs_pin_high: acs_in.is_high(),
            pe8_psp_pin_high: psp_in.is_high(),
            pe10_bksw_pin_high: bksw_in.is_high(),
            pe12_clutch_pin_high: clutch_in.is_high(),
        }
        .decode();

        let now_us = pac::TIM2.cnt().read();
        let flex_sample = aux_io_hw::take_flex_sample();
        let altf_sample = aux_io_hw::take_altf_sample();
        let (out, snap, aux_snap, faults_empty) = {
            let mut state = ECU_STATE.lock().await;
            if let Some(last_vss) = state.last_vss_ts_us
                && now_us.wrapping_sub(last_vss) > 1_000_000
            {
                state.vss_kmh = 0.0;
            }
            let vss_kmh = state.vss_kmh;
            let state_ref = &mut *state;
            let Some(ref cal) = state_ref.cal else {
                continue;
            };
            let out = state_ref.ecu.tick_1khz(
                now_us,
                &raw_adc,
                switches,
                vss_kmh,
                vts_fault_n.is_low(),
                cal,
            );
            let snap = state_ref.ecu.snapshot();
            let aux_snap = state_ref.aux.update_1khz(
                snap.rpm,
                &snap.sensors,
                snap.wideband_state == WidebandState::ClosedLoopReady,
                flex_sample,
                altf_sample,
                &state_ref.aux_cfg,
            );
            state_ref.aux_telemetry = aux_snap;
            (out, snap, aux_snap, snap.faults.is_empty())
        };

        HardwareOutputs::enforce_safety(snap.sync_state == SyncState::FullSynced720);
        hw_outputs.set_iacv_duty_per_mille(out.iacv_duty_per_mille);

        vts_high_en.set_level(Level::from(out.vts_high_side_on));
        flr_out.set_level(Level::from(out.fuel_pump_relay_on));
        fanc_out.set_level(Level::from(out.fan_relay_on));
        acc_out.set_level(Level::from(out.ac_clutch_relay_on));
        mil_out.set_level(Level::from(out.mil_lamp_on));
        altc_out.set_level(Level::from(aux_snap.outputs.altc_on));
        iab_out.set_level(Level::from(aux_snap.outputs.iab_on));
        evap_out.set_level(Level::from(aux_snap.outputs.evap_on));
        egr_a18_out.set_level(Level::from(aux_snap.outputs.pe0_on));
        var_a19_out.set_level(Level::from(aux_snap.outputs.pe1_on));
        wideband_hw.apply_outputs(out.lsu_ip_dac_code, out.lsu_heater_duty_per_mille);

        if tick_count.is_multiple_of(2) {
            sd_logger_hw::push_500hz_sample(&snap, &aux_snap);
        }

        // 2 Hz onboard `PE3` heartbeat LED (`5 Hz` fast blink if any diagnostic fault is set)
        let blink_period_ms = if faults_empty { 250 } else { 100 };
        if tick_count.is_multiple_of(blink_period_ms) {
            heartbeat_led.toggle();
        }

        if tick_count.is_multiple_of(1_000) {
            let (spk_cnt, inj_cnt, tach_cnt, cop_wd) = HardwareOutputs::telemetry_counts();
            let can_st = can_hw::stats();
            let dlc_cnt = dlc_hw::answered_frame_count();
            let sd_st = sd_logger_hw::status_report();
            let knk_ov = knock_hw::overrun_count();
            let knk_t = ECU_STATE.lock().await.ecu.knock_telemetry(knk_ov);
            let knk_events = knk_t.per_cyl_knock_events[0]
                + knk_t.per_cyl_knock_events[1]
                + knk_t.per_cyl_knock_events[2]
                + knk_t.per_cyl_knock_events[3];
            let knk_max_ret = knk_t.per_cyl_retard_deg_x10.into_iter().max().unwrap_or(0);
            let eth_pct = if aux_snap.flex.is_valid() {
                aux_snap.flex.ethanol_pct as i16
            } else {
                -1
            };
            let altf_pct = if aux_snap.altf.valid {
                aux_snap.altf.field_duty_pct as i16
            } else {
                -1
            };
            info!(
                "ECU 1Hz | up={}s | sync={} | rpm={} | adv={}deg | inj1={}us | spk={} | \
                 inj={} | tach={} | wd={} | flr={} | wb={} | ri={}ohm | lam={} | htr={} | \
                 eth={} | altf={} | aux=0x{=u8:02x} | can={}/{}/{}/{}/tec={} | dlc={} | \
                 knk=w:{}/o:{}/e:{}/r:{} | sd=on:{}/r:{}/d:{}/e:{}/mk:{} | flt=0x{=u32:08x}",
                tick_count / 1_000,
                snap.sync_state,
                snap.rpm,
                snap.actuators.spark_advance_deg_x10 / 10,
                snap.actuators.inj_pw_us[0],
                spk_cnt,
                inj_cnt,
                tach_cnt,
                cop_wd,
                out.fuel_pump_relay_on,
                snap.wideband_state,
                snap.lsu_ri_ohms as u32,
                snap.lambda,
                out.lsu_heater_duty_per_mille,
                eth_pct,
                altf_pct,
                aux_snap.outputs.bitmask(),
                can_st.self_test_passed,
                can_st.tx_ok,
                can_st.tx_drop,
                can_st.rx_count,
                can_st.tec,
                dlc_cnt,
                knk_t.window_count,
                knk_t.capture_overruns,
                knk_events,
                knk_max_ret,
                sd_st.logging_active,
                sd_st.records_logged,
                sd_st.dropped_records,
                sd_st.write_errors,
                sd_st.mark_count,
                snap.faults.0
            );
        }
    }
}
