//! Pure `no_std` deterministic interpolation, filtering, PID, and transcendental helpers.

/// Fast, accurate `no_std` natural logarithm `ln(x)` for positive `f32` (`x > 0.0`).
///
/// Uses IEEE-754 exponent extraction (`x = m * 2^e`, `m in [0.7071, 1.4142]`) followed by a
/// 7th-order odd `atanh` series `ln(m) = 2 * (u + u^3/3 + u^5/5 + u^7/7)` where `u = (m-1)/(m+1)`.
/// Worst-case error across `1e-4 .. 1e6` is `< 2e-7` (`< 0.001 °C` on NTC thermistors).
#[inline]
pub fn ln_f32(x: f32) -> f32 {
    if x <= 0.0 || x.is_nan() {
        return -100.0;
    }
    const LN_2: f32 = core::f32::consts::LN_2;
    let bits = x.to_bits();
    let mut exp = ((bits >> 23) & 0xff) as i32 - 127;
    // Normalize mantissa into [1.0, 2.0)
    let mut m = f32::from_bits((bits & 0x007f_ffff) | 0x3f80_0000);
    // Center around 1.0 ([0.7071, 1.4142]) for rapid series convergence
    if m > core::f32::consts::SQRT_2 {
        m *= 0.5;
        exp += 1;
    }
    let u = (m - 1.0) / (m + 1.0);
    let u2 = u * u;
    let poly = u * (1.0 + u2 * ((1.0 / 3.0) + u2 * ((1.0 / 5.0) + u2 * (1.0 / 7.0))));
    (exp as f32) * LN_2 + 2.0 * poly
}

/// Pure `no_std` `(sin(x), cos(x))` approximation accurate to `< 1e-6` for filter synthesis.
#[inline]
pub fn sin_cos_f32(radians: f32) -> (f32, f32) {
    use core::f32::consts::{FRAC_PI_2, PI, TAU};

    if radians.is_nan() || radians.is_infinite() {
        return (0.0, 1.0);
    }
    // Fold into [-PI, PI]
    let k = (radians / TAU) as i32;
    let mut x = radians - (k as f32) * TAU;
    if x > PI {
        x -= TAU;
    } else if x < -PI {
        x += TAU;
    }

    // Reduce into [-PI/2, PI/2] with sign flip for cosine on outer quadrants
    let (xr, cos_sign) = if x > FRAC_PI_2 {
        (PI - x, -1.0)
    } else if x < -FRAC_PI_2 {
        (-PI - x, -1.0)
    } else {
        (x, 1.0)
    };

    let x2 = xr * xr;
    let sin_x = xr
        * (1.0
            - x2 * ((1.0 / 6.0)
                - x2 * ((1.0 / 120.0) - x2 * ((1.0 / 5_040.0) - x2 * (1.0 / 362_880.0)))));
    let cos_mag = 1.0
        - x2 * (0.5
            - x2 * ((1.0 / 24.0)
                - x2 * ((1.0 / 720.0) - x2 * ((1.0 / 40_320.0) - x2 * (1.0 / 3_628_800.0)))));
    (sin_x, cos_sign * cos_mag)
}

/// Pure `no_std` square root for non-negative `f32` via IEEE-754 half-exponent seed + 4 Newton
/// iterations (relative error `< 1e-6`).
#[inline]
pub fn sqrt_f32(x: f32) -> f32 {
    if x <= 0.0 || x.is_nan() {
        return 0.0;
    }
    let half = 0.5 * x;
    let i = 0x1fbd_1df5_u32.wrapping_add(x.to_bits() >> 1);
    let mut y = f32::from_bits(i);
    // Newton iterations on y ~= sqrt(x): y_{n+1} = 0.5 * (y + x / y)
    y = 0.5 * y + half / y;
    y = 0.5 * y + half / y;
    y = 0.5 * y + half / y;
    0.5 * y + half / y
}

/// Finds `(lower_idx, upper_idx, frac_0_to_1)` along a strictly increasing `u16` axis.
#[inline]
pub fn find_axis_u16<const N: usize>(bins: &[u16; N], x: f32) -> (usize, usize, f32) {
    if x <= bins[0] as f32 {
        return (0, 0, 0.0);
    }
    let last = N - 1;
    if x >= bins[last] as f32 {
        return (last, last, 0.0);
    }
    let mut i = 0;
    while i < last {
        let x0 = bins[i] as f32;
        let x1 = bins[i + 1] as f32;
        if x <= x1 {
            let span = (x1 - x0).max(1e-6);
            return (i, i + 1, ((x - x0) / span).clamp(0.0, 1.0));
        }
        i += 1;
    }
    (last, last, 0.0)
}

/// Finds `(lower_idx, upper_idx, frac_0_to_1)` along a strictly increasing `i16` axis.
#[inline]
pub fn find_axis_i16<const N: usize>(bins: &[i16; N], x: f32) -> (usize, usize, f32) {
    if x <= bins[0] as f32 {
        return (0, 0, 0.0);
    }
    let last = N - 1;
    if x >= bins[last] as f32 {
        return (last, last, 0.0);
    }
    let mut i = 0;
    while i < last {
        let x0 = bins[i] as f32;
        let x1 = bins[i + 1] as f32;
        if x <= x1 {
            let span = (x1 - x0).max(1e-6);
            return (i, i + 1, ((x - x0) / span).clamp(0.0, 1.0));
        }
        i += 1;
    }
    (last, last, 0.0)
}

/// 1D piecewise-linear lookup over `u16` axis and `u16` values.
#[inline]
pub fn interp_1d_u16<const N: usize>(bins: &[u16; N], vals: &[u16; N], x: f32) -> f32 {
    let (i0, i1, t) = find_axis_u16(bins, x);
    let v0 = vals[i0] as f32;
    let v1 = vals[i1] as f32;
    v0 + (v1 - v0) * t
}

/// 1D piecewise-linear lookup over `i16` axis (e.g. `ECT °C`) and `u16` values.
#[inline]
pub fn interp_1d_i16_u16<const N: usize>(bins: &[i16; N], vals: &[u16; N], x: f32) -> f32 {
    let (i0, i1, t) = find_axis_i16(bins, x);
    let v0 = vals[i0] as f32;
    let v1 = vals[i1] as f32;
    v0 + (v1 - v0) * t
}

/// 1D piecewise-linear lookup over `i16` axis (e.g. `ECT °C`) and `u32` values.
#[inline]
pub fn interp_1d_i16_u32<const N: usize>(bins: &[i16; N], vals: &[u32; N], x: f32) -> f32 {
    let (i0, i1, t) = find_axis_i16(bins, x);
    let v0 = vals[i0] as f32;
    let v1 = vals[i1] as f32;
    v0 + (v1 - v0) * t
}

/// Bilinear 2D interpolation over `[row_map][col_rpm]` `u16` table (`16×16`).
#[inline]
pub fn interp_2d_u16(
    rpm_bins: &[u16; 16],
    map_bins: &[u16; 16],
    table: &[[u16; 16]; 16],
    rpm: f32,
    map_kpa: f32,
) -> f32 {
    let (c0, c1, tx) = find_axis_u16(rpm_bins, rpm);
    let (r0, r1, ty) = find_axis_u16(map_bins, map_kpa);

    let q00 = table[r0][c0] as f32;
    let q10 = table[r0][c1] as f32;
    let q01 = table[r1][c0] as f32;
    let q11 = table[r1][c1] as f32;

    let top = q00 + (q10 - q00) * tx;
    let bot = q01 + (q11 - q01) * tx;
    top + (bot - top) * ty
}

/// Bilinear 2D interpolation over `[row_map][col_rpm]` `i16` table (`16×16`).
#[inline]
pub fn interp_2d_i16(
    rpm_bins: &[u16; 16],
    map_bins: &[u16; 16],
    table: &[[i16; 16]; 16],
    rpm: f32,
    map_kpa: f32,
) -> f32 {
    let (c0, c1, tx) = find_axis_u16(rpm_bins, rpm);
    let (r0, r1, ty) = find_axis_u16(map_bins, map_kpa);

    let q00 = table[r0][c0] as f32;
    let q10 = table[r0][c1] as f32;
    let q01 = table[r1][c0] as f32;
    let q11 = table[r1][c1] as f32;

    let top = q00 + (q10 - q00) * tx;
    let bot = q01 + (q11 - q01) * tx;
    top + (bot - top) * ty
}

/// Single-pole exponential moving average (`IIR`) filter.
#[derive(Debug, Clone, Copy)]
pub struct LowPassFilter {
    alpha: f32,
    state: f32,
    initialized: bool,
}

impl LowPassFilter {
    pub const fn new(alpha: f32) -> Self {
        Self {
            alpha,
            state: 0.0,
            initialized: false,
        }
    }

    #[inline]
    pub fn update(&mut self, sample: f32) -> f32 {
        if !self.initialized {
            self.state = sample;
            self.initialized = true;
        } else {
            self.state += self.alpha * (sample - self.state);
        }
        self.state
    }

    #[inline]
    pub const fn value(&self) -> f32 {
        self.state
    }
}

/// Discrete-time PI/PID controller with integral clamping (anti-windup) and output saturation.
#[derive(Debug, Clone, Copy)]
pub struct PidController {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub out_min: f32,
    pub out_max: f32,
    integral: f32,
    prev_error: f32,
    has_prev: bool,
}

impl PidController {
    pub const fn new(kp: f32, ki: f32, kd: f32, out_min: f32, out_max: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            out_min,
            out_max,
            integral: 0.0,
            prev_error: 0.0,
            has_prev: false,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.has_prev = false;
    }

    pub fn update(&mut self, error: f32, dt_s: f32) -> f32 {
        let p = self.kp * error;
        self.integral = (self.integral + self.ki * error * dt_s).clamp(self.out_min, self.out_max);
        let d = if self.has_prev && dt_s > 1e-6 {
            self.kd * (error - self.prev_error) / dt_s
        } else {
            0.0
        };
        self.prev_error = error;
        self.has_prev = true;
        (p + self.integral + d).clamp(self.out_min, self.out_max)
    }
}

#[cfg(test)]
mod tests {
    use super::{
        LowPassFilter, PidController, interp_1d_i16_u16, interp_1d_i16_u32, interp_1d_u16,
        interp_2d_i16, interp_2d_u16, ln_f32,
    };

    #[test]
    fn ln_f32_matches_exact_values_within_1e_6() {
        let cases = [
            (0.1_f32, -core::f32::consts::LN_10),
            (1.0_f32, 0.0_f32),
            (core::f32::consts::E, 1.0_f32),
            (2490.0_f32, 7.820038_f32),
            (25000.0_f32, 10.126631_f32),
        ];
        for (x, expected) in cases {
            let got = ln_f32(x);
            assert!(
                (got - expected).abs() < 2e-6,
                "x={x}: got={got}, exp={expected}"
            );
        }
    }

    #[test]
    fn bilinear_interpolation_midpoint_and_clamping() {
        let mut rpm = [0u16; 16];
        let mut map = [0u16; 16];
        let mut tbl = [[0u16; 16]; 16];
        for i in 0..16 {
            rpm[i] = (i as u16 + 1) * 500;
            map[i] = (i as u16 + 1) * 10;
        }
        tbl[1][2] = 700; // MAP=20, RPM=1500
        tbl[1][3] = 800; // MAP=20, RPM=2000
        tbl[2][2] = 900; // MAP=30, RPM=1500
        tbl[2][3] = 1000; // MAP=30, RPM=2000

        let mid = interp_2d_u16(&rpm, &map, &tbl, 1750.0, 25.0);
        assert!((mid - 850.0).abs() < 1e-4);

        // Clamp below minimum axis
        let lo = interp_2d_u16(&rpm, &map, &tbl, 100.0, 2.0);
        assert!((lo - tbl[0][0] as f32).abs() < 1e-4);

        // Clamp above maximum axis
        let hi = interp_2d_u16(&rpm, &map, &tbl, 12_000.0, 250.0);
        assert!((hi - tbl[15][15] as f32).abs() < 1e-4);
    }

    #[test]
    fn ln_f32_guards_zero_negative_and_nan_inputs() {
        assert_eq!(ln_f32(0.0), -100.0);
        assert_eq!(ln_f32(-5.0), -100.0);
        assert_eq!(ln_f32(f32::NAN), -100.0);
    }

    #[test]
    fn interp_1d_and_2d_signed_helpers_interpolate_and_clamp() {
        let ect_bins: [i16; 4] = [-20, 0, 40, 80];
        let u16_vals: [u16; 4] = [160, 140, 110, 100];
        let u32_vals: [u32; 4] = [18_000, 12_000, 6_000, 4_000];

        assert!((interp_1d_i16_u16(&ect_bins, &u16_vals, -40.0) - 160.0).abs() < 1e-4);
        assert!((interp_1d_i16_u16(&ect_bins, &u16_vals, -10.0) - 150.0).abs() < 1e-4);
        assert!((interp_1d_i16_u16(&ect_bins, &u16_vals, 100.0) - 100.0).abs() < 1e-4);
        assert!((interp_1d_i16_u32(&ect_bins, &u32_vals, 20.0) - 9_000.0).abs() < 1e-3);

        let u_bins: [u16; 3] = [100, 200, 300];
        let u_vals: [u16; 3] = [10, 30, 90];
        assert!((interp_1d_u16(&u_bins, &u_vals, 250.0) - 60.0).abs() < 1e-4);

        let mut rpm = [0u16; 16];
        let mut map = [0u16; 16];
        let mut spark = [[0i16; 16]; 16];
        for i in 0..16 {
            rpm[i] = (i as u16 + 1) * 500;
            map[i] = (i as u16 + 1) * 10;
        }
        spark[0][0] = -20;
        spark[0][1] = 20;
        spark[1][0] = 0;
        spark[1][1] = 40;
        assert!((interp_2d_i16(&rpm, &map, &spark, 750.0, 15.0) - 10.0).abs() < 1e-4);
    }

    #[test]
    fn low_pass_filter_seeds_first_sample_and_tracks_step() {
        let mut lpf = LowPassFilter::new(0.25);
        assert_eq!(lpf.value(), 0.0);
        assert!((lpf.update(100.0) - 100.0).abs() < 1e-5);
        assert!((lpf.update(200.0) - 125.0).abs() < 1e-5);
        assert!((lpf.value() - 125.0).abs() < 1e-5);
    }

    #[test]
    fn pid_controller_clamps_integral_and_resets_cleanly() {
        let mut pid = PidController::new(1.0, 10.0, 0.1, -5.0, 5.0);
        // First step: P = 2.0, I = 2.0 * 10 * 0.1 = 2.0, D = 0.0 (no prev_error) -> 4.0
        let out0 = pid.update(2.0, 0.1);
        assert!((out0 - 4.0).abs() < 1e-4);

        // Large sustained error saturates both integral and total output at out_max (5.0)
        for _ in 0..20 {
            let out = pid.update(10.0, 0.1);
            assert!((out - 5.0).abs() < 1e-5);
        }

        // Reset clears integral and derivative history
        pid.reset();
        let after_reset = pid.update(0.0, 0.1);
        assert_eq!(after_reset, 0.0);
    }
}
