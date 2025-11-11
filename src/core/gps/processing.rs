// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2022 Ivan <ivan.grubisic at gmail>

use nalgebra::Vector3;

// ============================================================================
// Coordinate math and conversions
// ============================================================================

const EARTH_RADIUS_M: f64 = 6_371_000.0;

pub fn haversine_distance_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let to_rad = |d: f64| d.to_radians();
    let dlat = to_rad(lat2 - lat1);
    let dlon = to_rad(lon2 - lon1);
    let a = (dlat / 2.0).sin().powi(2) + lat1.to_radians().cos() * lat2.to_radians().cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    EARTH_RADIUS_M * c
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, ::serde::Serialize, ::serde::Deserialize)]
pub enum BearingConvention {
    #[default]
    EastCCW,  // ENU, ROS-REP 103
    NorthCW,  // NED, azimuth
}

pub fn initial_bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64, convention: BearingConvention) -> f64 {
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let y = dlon.sin() * phi2.cos();
    let x = phi1.cos() * phi2.sin() - phi1.sin() * phi2.cos() * dlon.cos();
    let north_cw_bearing = (y.atan2(x).to_degrees() + 360.0) % 360.0;
    match convention {
        BearingConvention::EastCCW => (360.0 - north_cw_bearing + 90.0) % 360.0,
        BearingConvention::NorthCW => north_cw_bearing,
    }
}

/// Project a GPS point to ENU coordinates (East, North)
pub fn latlon_to_enu(lat_deg: f64, lon_deg: f64, lat0: f64, lon0: f64) -> (f64, f64) {
    let d_n = haversine_distance_m(lat0, lon0, lat_deg, lon0) * if lat_deg > lat0 { 1.0 } else { -1.0 };
    let d_e = haversine_distance_m(lat0, lon0, lat0, lon_deg) * if lon_deg > lon0 { 1.0 } else { -1.0 };
    (d_e, d_n)
}

/// Interpolate lat/lon at a specific time using binary search
pub fn linear_interpolate_latlon(time: &[f64], lat: &[f64], lon: &[f64], t_s: f64) -> (f64, f64) {
    let n = time.len();
    let mut lo = 0usize;
    let mut hi = n - 1;
    if t_s <= time[0] {
        lo = 0; hi = 0;
    } else if t_s >= time[n - 1] {
        lo = n - 1; hi = n - 1;
    } else {
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if time[mid] <= t_s { lo = mid; } else { hi = mid; }
        }
    }
    if lo == hi {
        (lat[lo], lon[lo])
    } else {
        let t0 = time[lo];
        let t1 = time[hi];
        let frac = ((t_s - t0) / (t1 - t0)).clamp(0.0, 1.0);
        let latp = lat[lo] + (lat[hi] - lat[lo]) * frac;
        let lonp = lon[lo] + (lon[hi] - lon[lo]) * frac;
        (latp, lonp)
    }
}


// ============================================================================
// Signal processing
// ============================================================================

/// Unwrap circular angles (degrees) to be continuous over time by removing 360° jumps.
pub fn unwrap_angles_deg(data: &[f64]) -> Vec<f64> {
    if data.is_empty() { return vec![]; }
    let mut out = Vec::with_capacity(data.len());
    let mut prev = data[0];
    out.push(prev);
    for &v in &data[1..] {
        let mut d = v - prev;
        while d > 180.0 { d -= 360.0; }
        while d < -180.0 { d += 360.0; }
        let unwrapped = prev + d;
        out.push(unwrapped);
        prev = unwrapped;
    }
    out
}

/// Linearly resample a time series at a fixed rate on [t0, t1]. Returns (times_ms, values).
pub fn resample_linear(times_ms: &[f64], values: &[f64], t0_ms: f64, t1_ms: f64, sample_rate_hz: f64) -> (Vec<f64>, Vec<f64>) {
    let n = times_ms.len().min(values.len());
    if n == 0 || t1_ms <= t0_ms || sample_rate_hz <= 0.0 { return (vec![], vec![]); }
    let dt = 1000.0 / sample_rate_hz;
    let mut out_times = Vec::new();
    let mut out_vals = Vec::new();
    let mut i = 0usize;
    let mut t = t0_ms;
    while t <= t1_ms {
        // advance i so that times[i] <= t <= times[i+1]
        while i + 1 < n && times_ms[i + 1] < t { i += 1; }
        if i + 1 >= n { break; }
        let t0 = times_ms[i];
        let t1 = times_ms[i + 1];
        if t1 <= t0 { i += 1; continue; }
        let f = ((t - t0) / (t1 - t0)).clamp(0.0, 1.0);
        let v0 = values[i];
        let v1 = values[i + 1];
        out_times.push(t);
        out_vals.push(v0 + (v1 - v0) * f);
        t += dt;
    }
    (out_times, out_vals)
}

// ============================================================================
// Distance sampling
// ============================================================================

/// Compute cumulative distance (meters) while skipping segments whose instantaneous
/// speed is below `min_speed_mps`.
///
/// - `times_s`, `lat`, `lon` must be same length (>= 2) and monotonically increasing in time.
/// - If `min_speed_mps` <= 0, all segments are counted.
pub fn compute_pruned_cumulative_distance(times_s: &[f64], lat: &[f64], lon: &[f64], min_speed: f64) -> Vec<f64> {
    // TODO: use better interpolated estimates of speed and positionlike for synchronization
    let n = times_s.len().min(lat.len()).min(lon.len());
    if n == 0 { return vec![]; }
    if n == 1 { return vec![0.0]; }

    let mut cum = vec![0.0f64; n];
    let use_threshold = min_speed > 0.0;
    for i in 1..n {
        let dt = (times_s[i] - times_s[i - 1]).max(0.0);
        if dt <= 0.0 { cum[i] = cum[i - 1]; continue; }
        let d = haversine_distance_m(lat[i - 1], lon[i - 1], lat[i], lon[i]);
        let speed = d / dt.max(1e-6);
        let valid = !use_threshold || speed >= min_speed;
        cum[i] = if valid { cum[i - 1] + d } else { cum[i - 1] };
    }
    cum
}

/// Find track times (seconds) where cumulative distance crosses multiples of `step_m`.
/// Assumes `cumdist_m` and `times_s` are same-length arrays, monotonically non-decreasing.
pub fn find_distance_crossings(cumdist_m: &[f64], times_s: &[f64], step_m: f64) -> Vec<f64> {
    let n = cumdist_m.len().min(times_s.len());
    if n == 0 || step_m <= 0.0 { return vec![]; }
    if n == 1 { return vec![]; }

    let total = cumdist_m[n - 1];
    if total < step_m { return vec![]; }

    let mut out: Vec<f64> = Vec::new();
    let mut target = step_m;

    let mut i = 0usize;
    while i + 1 < n && target <= total {
        let c0 = cumdist_m[i];
        let c1 = cumdist_m[i + 1];
        if c1 > c0 {
            while target <= c1 {
                if target < c0 { break; }
                let f = ((target - c0) / (c1 - c0)).clamp(0.0, 1.0);
                let t = times_s[i] + (times_s[i + 1] - times_s[i]) * f;
                out.push(t);
                target += step_m;
                if target > total { break; }
            }
        }
        i += 1;
    }
    out
}

