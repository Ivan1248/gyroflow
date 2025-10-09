// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2022 Ivan Grubišić <ivan.grubisic at gmail>

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

pub fn latlon_to_enu(lat0: f64, lon0: f64, lat: f64, lon: f64) -> Vector3<f64> {
    let d_n = haversine_distance_m(lat0, lon0, lat, lon0) * if lat > lat0 { 1.0 } else { -1.0 };
    let d_e = haversine_distance_m(lat0, lon0, lat0, lon) * if lon > lon0 { 1.0 } else { -1.0 };
    Vector3::new(d_e, 0.0, d_n)
}

// ============================================================================
// Signal processing utilities
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

