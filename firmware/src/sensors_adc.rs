//! `ADC1` + `DMA1_CH0` 16-bit multi-channel analog sensor scanner (`_DOCS/stm32h743_pinout.md`).
//!
//! Scans all 7 core engine sensors once per `1 kHz` control tick with `8x` hardware
//! oversampling (`Averaging::Samples8`) and `64.5` ADC clock cycles per sample (~85 µs total
//! sequence time at the `48.000 MHz` `PLL3_R` ADC kernel clock):
//! - `PC0` (`ADC1_INP10`): `MAP` (`D17`)
//! - `PC1` (`ADC1_INP11`): `TPS` (`D11`)
//! - `PA2` (`ADC1_INP14`): `ECT` (`D13`)
//! - `PA3` (`ADC1_INP15`): `IAT` (`D15`)
//! - `PC4` (`ADC1_INP4`):  `VBatt` (`A25`/`B1`)
//! - `PA0` (`ADC1_INP16`): `VCC_5V_SNS`
//! - `PC5` (`ADC1_INP8`):  `ELD` (`D10`)

use ecu_types::RawSensorAdc;
use embassy_stm32::Peri;
use embassy_stm32::adc::{
    Adc, AdcChannel, AdcConfig, AnyAdcChannel, Averaging, Resolution, SampleTime,
};
use embassy_stm32::dma;
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::peripherals::{ADC1, DMA1_CH0, PA0, PA2, PA3, PC0, PC1, PC4, PC5};

/// Sample time per channel (`64.5` cycles + `8.5` conversion = `73` ADC cycles @ `48 MHz`).
const SENSOR_SAMPLE_TIME: SampleTime = SampleTime::CYCLES64_5;
/// Number of `ADC1` regular-sequence channels scanned per `1 kHz` cycle.
const ADC1_CHANNEL_COUNT: usize = 7;

/// Analog GPIO pins wired to `ADC1` for core Honda OBD1 sensor measurements.
pub struct Adc1SensorPins {
    /// `PC0` (`ADC1_INP10`) -> Manifold Absolute Pressure (`D17`).
    pub map_pc0: Peri<'static, PC0>,
    /// `PC1` (`ADC1_INP11`) -> Throttle Position Sensor (`D11`).
    pub tps_pc1: Peri<'static, PC1>,
    /// `PA2` (`ADC1_INP14`) -> Engine Coolant Temperature (`D13`).
    pub ect_pa2: Peri<'static, PA2>,
    /// `PA3` (`ADC1_INP15`) -> Intake Air Temperature (`D15`).
    pub iat_pa3: Peri<'static, PA3>,
    /// `PC4` (`ADC1_INP4`) -> Switched Battery Voltage (`A25`/`B1`).
    pub vbatt_pc4: Peri<'static, PC4>,
    /// `PA0` (`ADC1_INP16`) -> Ratiometric `+5V` sensor supply monitor (`VCC_5V_SNS`).
    pub vcc_5v_pa0: Peri<'static, PA0>,
    /// `PC5` (`ADC1_INP8`) -> Electrical Load Detector (`D10`).
    pub eld_pc5: Peri<'static, PC5>,
}

/// Hardware `ADC1` + `DMA1_CH0` driver sampling the 7 primary ECU analog inputs.
pub struct Adc1Sensors {
    adc: Adc<'static, ADC1>,
    dma: Peri<'static, DMA1_CH0>,
    channels: [AnyAdcChannel<'static, ADC1>; ADC1_CHANNEL_COUNT],
}

impl Adc1Sensors {
    /// Initializes and self-calibrates `ADC1` in `16-bit` mode with `8x` hardware averaging.
    pub fn new(
        adc1: Peri<'static, ADC1>,
        dma: Peri<'static, DMA1_CH0>,
        pins: Adc1SensorPins,
    ) -> Self {
        let cfg = AdcConfig {
            resolution: Some(Resolution::BITS16),
            averaging: Some(Averaging::Samples8),
        };

        let adc = Adc::new_with_config(adc1, cfg);
        let channels = [
            pins.map_pc0.degrade_adc(),
            pins.tps_pc1.degrade_adc(),
            pins.ect_pa2.degrade_adc(),
            pins.iat_pa3.degrade_adc(),
            pins.vbatt_pc4.degrade_adc(),
            pins.vcc_5v_pa0.degrade_adc(),
            pins.eld_pc5.degrade_adc(),
        ];

        Self { adc, dma, channels }
    }

    /// Performs one DMA-backed 7-channel scan sequence and packs results into [`RawSensorAdc`].
    pub async fn read_raw<I>(&mut self, irq: I) -> RawSensorAdc
    where
        I: Binding<<DMA1_CH0 as dma::ChannelInstance>::Interrupt, dma::InterruptHandler<DMA1_CH0>>,
    {
        let mut readings = [0u16; ADC1_CHANNEL_COUNT];
        let [ch_map, ch_tps, ch_ect, ch_iat, ch_vbatt, ch_vcc, ch_eld] = &mut self.channels;

        self.adc
            .read(
                self.dma.reborrow(),
                irq,
                [
                    (ch_map, SENSOR_SAMPLE_TIME),
                    (ch_tps, SENSOR_SAMPLE_TIME),
                    (ch_ect, SENSOR_SAMPLE_TIME),
                    (ch_iat, SENSOR_SAMPLE_TIME),
                    (ch_vbatt, SENSOR_SAMPLE_TIME),
                    (ch_vcc, SENSOR_SAMPLE_TIME),
                    (ch_eld, SENSOR_SAMPLE_TIME),
                ]
                .into_iter(),
                &mut readings,
            )
            .await;

        RawSensorAdc {
            map_raw: readings[0],
            tps_raw: readings[1],
            ect_raw: readings[2],
            iat_raw: readings[3],
            vbatt_raw: readings[4],
            vcc_5v_raw: readings[5],
            eld_raw: readings[6],
            ..RawSensorAdc::default()
        }
    }
}
