// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2021-2025 Gyroflow contributors
//
//! GPS-Gyro Synchronization
//!
//! This module provides a unified API for synchronizing GPS course (heading) data with
//! gyroscope yaw measurements. The main entry point is [`synchronize_gps_gyro()`].
//!
//! # Quick Start
//!
//! 1. Resample your GPS and gyro data to uniform sampling (typically 10 Hz)
//! 2. Call `synchronize_gps_gyro()` with the resampled data
//! 3. Use the returned time offset to align your GPS track
//!
//! ```ignore
//! use gyroflow_core::gps::synchronization::*;
//!
//! let result = synchronize_gps_gyro(
//!     &gps_course_deg,       // GPS heading in degrees
//!     &gyro_yaw_deg,         // Gyro yaw in degrees
//!     DEFAULT_SAMPLE_RATE_HZ, // 10 Hz
//!     DEFAULT_MAX_TIME_SHIFT_S, // ±60 seconds
//!     Some(&speed_mps),      // Optional speed for filtering
//!     false,                 // Usually no scale estimation needed
//! )?;
//!
//! println!("Offset: {:.3}s, RMS: {:.2}°", result.time_offset_s, result.residual_rms_deg);
//! ```

use crate::gyro_source::GyroSource;

/// Recommended maximum time offset search range (±60 seconds)
pub const DEFAULT_MAX_TIME_SHIFT_S: f64 = 60.0;

/// Recommended sample rate for GPS-gyro synchronization (10 Hz)
pub const DEFAULT_SAMPLE_RATE_HZ: f64 = 10.0;

/// Signal type for synchronization
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalType {
    /// Use heading angles directly (default, suitable for smooth trajectories)
    Angles,
    /// Use angular rates (better for removing heading offset and drift, more robust)
    Rates,
}

impl Default for SignalType {
    fn default() -> Self {
        SignalType::Angles
    }
}

/// Sample gyro yaw angles at specified video timestamps.
///
/// Extracts yaw (heading) from gyro quaternions at given video times in milliseconds.
///
/// # Arguments
/// * `gyro` - The gyro source containing quaternion data
/// * `times_ms` - Video timestamps in milliseconds where yaw should be sampled
/// * `use_smoothed` - If true, use smoothed quaternions; if false, use original quaternions
///
/// # Returns
/// Vector of yaw angles in degrees at the requested timestamps
pub fn sample_gyro_yaw_at_times(gyro: &GyroSource, times_ms: &[f64], use_smoothed: bool) -> Vec<f64> {
    times_ms.iter().map(|&t| {
        let q = if use_smoothed {
            gyro.actual_smoothed_quat_at_timestamp(t)
        } else {
            gyro.org_quat_at_timestamp(t)
        };
        let (_roll, _pitch, yaw) = q.euler_angles();
        yaw.to_degrees()
    }).collect()
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

/// Result of GPS-gyro synchronization
#[derive(Debug, Clone, Copy)]
pub struct SyncResult {
    /// Time offset in seconds to ADD to GPS timestamps to align with gyro
    /// Positive: GPS is late (add time), Negative: GPS is early (subtract time)
    pub time_offset_s: f64,
    pub yaw_scale: Option<f64>,  // scale factor if estimated (None if not requested)
    pub residual_rms_deg: f64,   // RMS angular error in degrees
    pub valid_samples: usize,    // number of samples used after masking
    pub signal_type: SignalType, // which signal type was used
}

/// Synchronize GPS course with gyro yaw to find time offset and optional scale factor.
///
/// This function correlates uniformly sampled GPS course (heading) with gyro yaw angles
/// to determine the time offset needed to align them. It automatically:
/// - Filters out low-speed segments where GPS heading is unreliable (if speed provided)
/// - Downweights GPS segments with low variance (stationary periods, when using angles)
/// - Smooths gyro data to reduce noise
/// - Computes quality metrics (RMS error, valid sample count)
///
/// # Arguments
/// * `gps_course_deg` - GPS course angles in degrees [0-360) (uniformly sampled)
/// * `gyro_yaw_deg` - Gyro yaw angles in degrees (uniformly sampled)
/// * `sample_rate_hz` - Sampling rate in Hz (typically 10 Hz, see `DEFAULT_SAMPLE_RATE_HZ`)
/// * `max_time_shift_s` - Maximum search range ±seconds (typically 60s, see `DEFAULT_MAX_TIME_SHIFT_S`)
/// * `speed_mps` - Optional GPS speed in m/s for masking (same sampling as course/yaw)
/// * `estimate_scale` - Enable scale factor estimation (typically `false` unless sensors differ)
/// * `signal_type` - Use angles or angular rates for correlation (see `SignalType`)
///
/// # Returns
/// `Some(SyncResult)` with time offset, optional scale, RMS error, and valid sample count.
/// `None` if insufficient data (< 8 valid samples after masking).
///
/// # Signal Type Selection
/// - `SignalType::Angles` (default): Best for smooth trajectories, existing behavior
/// - `SignalType::Rates`: More robust to heading offsets and drift, requires speed masking
///
/// # Example
/// ```ignore
/// use gyroflow_core::gps::synchronization::*;
///
/// // After resampling GPS and gyro data to 10 Hz...
/// let result = synchronize_gps_gyro(
///     &gps_course_deg,
///     &gyro_yaw_deg,
///     DEFAULT_SAMPLE_RATE_HZ,
///     DEFAULT_MAX_TIME_SHIFT_S,
///     Some(&speed_mps),         // Enable speed masking (recommended for rates)
///     false,                     // No scale estimation
///     SignalType::Rates,         // Use angular rates
/// )?;
///
/// println!("Time offset: {:.3}s", result.time_offset_s);
/// println!("RMS error: {:.2}°", result.residual_rms_deg);
/// println!("Valid samples: {}", result.valid_samples);
/// ```
pub fn synchronize_gps_gyro(
    gps_course_deg: &[f64],
    gyro_yaw_deg: &[f64],
    sample_rate_hz: f64,
    max_time_shift_s: f64,
    speed_mps: Option<&[f64]>,
    estimate_scale: bool,
    signal_type: SignalType,
) -> Option<SyncResult> {
    const MIN_SAMPLES: usize = 8;
    
    if gps_course_deg.is_empty() || gyro_yaw_deg.is_empty() || sample_rate_hz <= 0.0 {
        return None; 
    }
    
    let n = gps_course_deg.len().min(gyro_yaw_deg.len());
    if n < MIN_SAMPLES {
        return None;
    }
    
    // Step 1: Apply speed-based masking if provided
    let (course_masked, yaw_masked, valid_count) = if let Some(speed) = speed_mps {
        let (c, y) = apply_speed_based_masking(&gps_course_deg[..n], &gyro_yaw_deg[..n], speed);
        let count = c.len();
        (c, y, count)
    } else {
        (gps_course_deg[..n].to_vec(), gyro_yaw_deg[..n].to_vec(), n)
    };
    
    if valid_count < MIN_SAMPLES {
        return None;
    }

    // Step 2: Preprocess signals (unwrap + variance weighting/rate computation + smoothing)
    let (course_processed, yaw_processed) = preprocess_angular_signals(
        &course_masked,
        &yaw_masked,
        sample_rate_hz,
        signal_type,
    );
    
    if course_processed.len() < MIN_SAMPLES || yaw_processed.len() < MIN_SAMPLES {
        return None;
    }
    
    // Step 3: Find time offset via cross-correlation
    let time_offset_s = find_time_offset_via_correlation(
        &course_processed,
        &yaw_processed,
        sample_rate_hz,
        max_time_shift_s,
    )?;
    
    // Step 4: Optionally refine with scale estimation
    let (final_offset_s, yaw_scale) = if estimate_scale {
        refine_offset_and_scale(
            &course_masked,
            &yaw_masked,
            time_offset_s,
            sample_rate_hz,
            max_time_shift_s,
        )
        } else {
        (time_offset_s, None)
    };
    
    // Step 5: Compute quality metric (residual RMS)
    let residual_rms_deg = compute_alignment_residual(
        &course_masked,
        &yaw_masked,
        final_offset_s,
        yaw_scale.unwrap_or(1.0),
        sample_rate_hz,
    );
    
    Some(SyncResult {
        time_offset_s: final_offset_s,
        yaw_scale,
        residual_rms_deg,
        valid_samples: valid_count,
        signal_type,
    })
}



/// Simple unweighted linear regression y ≈ a*x + c
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len().min(y.len());
    if n == 0 { return (1.0, 0.0); }
    let mean = |v: &[f64]| v.iter().copied().sum::<f64>() / (v.len() as f64);
    let mx = mean(&x[..n]);
    let my = mean(&y[..n]);
    let mut num = 0.0;
    let mut den = 0.0;
    for i in 0..n {
        let dx = x[i] - mx;
        num += dx * (y[i] - my);
        den += dx * dx;
    }
    let a = if den.abs() > 1e-12 { num / den } else { 1.0 };
    let c = my - a * mx;
    (a, c)
}

/// Compute mean squared wrapped angular error between a*x - angle_offset and y
fn mean_squared_wrapped_error(x: &[f64], y: &[f64], a: f64, angle_offset_deg: f64) -> f64 {
    let n = x.len().min(y.len());
    if n == 0 { return f64::INFINITY; }
    let mut sum = 0.0;
    for i in 0..n {
        let pred = a * x[i] - angle_offset_deg;
        let mut d = y[i] - pred;
        while d > 180.0 { d -= 360.0; }
        while d < -180.0 { d += 360.0; }
        sum += d * d;
    }
    sum / (n as f64)
}

// ============================================================================
// Single-responsibility helper functions for GPS-gyro synchronization
// ============================================================================

/// Preprocess angular signals: unwrap angles, optionally convert to rates, apply conditioning, smooth.
/// Responsibility: Signal conditioning for robust correlation.
fn preprocess_angular_signals(
    course_deg: &[f64],
    yaw_deg: &[f64],
    sample_rate_hz: f64,
    signal_type: SignalType,
) -> (Vec<f64>, Vec<f64>) {
    // Step 1: Unwrap angles for continuous processing
    let course_unwrapped = super::unwrap_angles_deg(course_deg);
    let yaw_unwrapped = super::unwrap_angles_deg(yaw_deg);
    
    // Step 2: Convert to rates if requested, otherwise keep as angles
    let (course_signal, yaw_signal) = match signal_type {
        SignalType::Angles => (course_unwrapped, yaw_unwrapped),
        SignalType::Rates => {
            let course_rates = compute_angular_rates(&course_unwrapped, sample_rate_hz);
            let yaw_rates = compute_angular_rates(&yaw_unwrapped, sample_rate_hz);
            (course_rates, yaw_rates)
        }
    };
    
    // Step 3: Apply signal-specific conditioning
    let course_conditioned = match signal_type {
        SignalType::Angles => apply_variance_weighting_to_course(&course_signal),
        SignalType::Rates => course_signal, // No variance weighting for rates
    };
    
    // Step 4: Smooth both signals to reduce noise
    let yaw_smoothed = smooth_angular_signal(&yaw_signal, sample_rate_hz);
    let course_smoothed = smooth_angular_signal(&course_conditioned, sample_rate_hz);
    
    (course_smoothed, yaw_smoothed)
}

/// Find time offset between preprocessed signals using cross-correlation.
/// Responsibility: Time offset estimation via correlation peak detection.
fn find_time_offset_via_correlation(
    signal_a: &[f64],
    signal_b: &[f64],
    sample_rate_hz: f64,
    max_offset_s: f64,
) -> Option<f64> {
    cross_correlate_angular_signals(signal_a, signal_b, sample_rate_hz, max_offset_s)
}

/// Refine time offset and estimate scale factor using optimization.
/// Responsibility: Joint offset-scale parameter estimation.
fn refine_offset_and_scale(
    course_deg: &[f64],
    yaw_deg: &[f64],
    initial_offset_s: f64,
    sample_rate_hz: f64,
    max_offset_s: f64,
) -> (f64, Option<f64>) {
    // Unwrap for linear regression
    let course_unwrapped = super::unwrap_angles_deg(course_deg);
    let yaw_unwrapped = super::unwrap_angles_deg(yaw_deg);
    
    // Search around initial offset
    let search_range_s = (max_offset_s * 0.2).max(0.5); // ±20% or ±0.5s, whichever is larger
    let max_lag = (search_range_s * sample_rate_hz).round() as isize;
    let initial_lag = (initial_offset_s * sample_rate_hz).round() as isize;
    
    let mut best_mse = f64::INFINITY;
    let mut best_offset_s = initial_offset_s;
    let mut best_scale = 1.0;
    
    let n = course_unwrapped.len().min(yaw_unwrapped.len());
    
    for lag_offset in -max_lag..=max_lag {
        let lag = initial_lag + lag_offset;
        
        // Build aligned slices
        let (yaw_aligned, course_aligned) = if lag >= 0 {
            let start = lag as usize;
            if start >= n { continue; }
            let len = n - start;
            if len < 8 { continue; }
            (&yaw_unwrapped[0..len], &course_unwrapped[start..start + len])
        } else {
            let start = (-lag) as usize;
            if start >= n { continue; }
            let len = n - start;
            if len < 8 { continue; }
            (&yaw_unwrapped[start..start + len], &course_unwrapped[0..len])
        };
        
        // Linear regression: course ≈ scale * yaw + offset
        let (scale, angle_offset_deg) = linear_regression(yaw_aligned, course_aligned);
        
        // Compute MSE
        let mse = mean_squared_wrapped_error(yaw_aligned, course_aligned, scale, angle_offset_deg);
        
        if mse < best_mse {
            best_mse = mse;
            // Positive lag means course is shifted forward (course is later/gyro is earlier)
            // To align, we subtract from GPS times, so negate the lag
            best_offset_s = -(lag as f64) / sample_rate_hz;
            best_scale = scale;
        }
    }
    
    // Only return scale if significantly different from 1.0
    let scale_result = if (best_scale - 1.0).abs() > 0.05 {
        Some(best_scale)
    } else {
        None
    };
    
    (best_offset_s, scale_result)
}

/// Compute RMS angular error between aligned signals.
/// Responsibility: Quality metric computation.
fn compute_alignment_residual(
    course_deg: &[f64],
    yaw_deg: &[f64],
    offset_s: f64,
    scale: f64,
    sample_rate_hz: f64,
) -> f64 {
    let n = course_deg.len().min(yaw_deg.len());
    if n == 0 {
        return f64::INFINITY;
    }
    
    let lag = (offset_s * sample_rate_hz).round() as isize;
    
    let mut sum_sq = 0.0;
    let mut count = 0;
    
    for i in 0..n {
        let j = i as isize + lag;
        if j >= 0 && (j as usize) < n {
            let course_val = course_deg[i];
            let yaw_val = yaw_deg[j as usize] * scale;
            
            // Compute wrapped angular difference
            let mut diff = course_val - yaw_val;
            while diff > 180.0 { diff -= 360.0; }
            while diff < -180.0 { diff += 360.0; }
            
            sum_sq += diff * diff;
            count += 1;
        }
    }
    
    if count > 0 {
        (sum_sq / count as f64).sqrt()
    } else {
        f64::INFINITY
    }
}

/// Apply speed-based masking to filter out low-speed segments for better correlation.
/// Responsibility: Speed-based data filtering.
fn apply_speed_based_masking(
    values_a: &[f64], 
    values_b: &[f64], 
    speed_u_mps: &[f64]
) -> (Vec<f64>, Vec<f64>) {
    const MIN_SPEED_MPS: f64 = 0.5; // Minimum speed threshold (0.5 m/s ≈ 1.8 km/h)
    if speed_u_mps.is_empty() || speed_u_mps.len() != values_a.len() {
        return (values_a.to_vec(), values_b.to_vec());
    }
    
    // Create speed-based mask
    let speed_mask: Vec<bool> = speed_u_mps.iter().map(|&s| s >= MIN_SPEED_MPS).collect();
    
    // Count valid samples
    let valid_count = speed_mask.iter().filter(|&&valid| valid).count();
    if valid_count < 8 {
        // Not enough high-speed samples, return original data
        return (values_a.to_vec(), values_b.to_vec());
    }
    
    // Apply mask to both signals
    let masked_a: Vec<f64> = values_a.iter()
        .zip(speed_mask.iter())
        .filter_map(|(&val, &valid)| if valid { Some(val) } else { None })
        .collect();
        
    let masked_b: Vec<f64> = values_b.iter()
        .zip(speed_mask.iter())
        .filter_map(|(&val, &valid)| if valid { Some(val) } else { None })
        .collect();
    
    (masked_a, masked_b)
}

/// Compute angular rates from unwrapped angles using central differences.
/// Responsibility: Convert angles to angular velocities (deg/s).
fn compute_angular_rates(angles_deg: &[f64], sample_rate_hz: f64) -> Vec<f64> {
    let n = angles_deg.len();
    if n < 2 {
        return vec![];
    }
    
    let dt = 1.0 / sample_rate_hz;
    let mut rates = Vec::with_capacity(n);
    
    // Use central differences for interior points, forward/backward for endpoints
    for i in 0..n {
        let rate = if i == 0 {
            // Forward difference
            (angles_deg[1] - angles_deg[0]) / dt
        } else if i == n - 1 {
            // Backward difference
            (angles_deg[n - 1] - angles_deg[n - 2]) / dt
        } else {
            // Central difference (more accurate)
            (angles_deg[i + 1] - angles_deg[i - 1]) / (2.0 * dt)
        };
        rates.push(rate);
    }
    
    rates
}

/// Apply variance-based weighting to GPS course data to downweight flat segments.
fn apply_variance_weighting_to_course(course_deg: &[f64]) -> Vec<f64> {
    let n = course_deg.len();
    if n < 8 { return course_deg.to_vec(); }
    
    let wrap_angle = |a: f64| ((a + 180.0) % 360.0) - 180.0;
    let wrapped_course: Vec<f64> = course_deg.iter().map(|&v| wrap_angle(v)).collect();
    
    // Compute local variance weights (5-sample window)
    let window_len = 5usize.min(n.max(1));
    let mut weights: Vec<f64> = vec![1.0; n];
    if window_len >= 3 {
        for i in 0..n {
            let start = i.saturating_sub(window_len/2);
            let end = (i + window_len/2 + 1).min(n);
            let segment = &wrapped_course[start..end];
            let mean = segment.iter().copied().sum::<f64>() / (segment.len() as f64);
            let variance = segment.iter().map(|v| (v - mean)*(v - mean)).sum::<f64>() / (segment.len() as f64);
            weights[i] = (variance / 25.0).min(1.0); // promote segments with >~5 deg^2 variance
        }
    }
    
    wrapped_course.iter().enumerate().map(|(i, v)| *v * weights[i]).collect()
}

/// Smooth angular signal using moving average to improve correlation robustness.
fn smooth_angular_signal(angles_deg: &[f64], sample_rate_hz: f64) -> Vec<f64> {
    let n = angles_deg.len();
    if n < 5 || sample_rate_hz <= 0.0 { return angles_deg.to_vec(); }
    
    let wrap_angle = |a: f64| ((a + 180.0) % 360.0) - 180.0;
    let wrapped_angles: Vec<f64> = angles_deg.iter().map(|&v| wrap_angle(v)).collect();
    
    // Apply moving average smoothing (~0.5s window)
    let window_size = ((sample_rate_hz * 0.5).round() as usize).max(3).min(n);
    let half_window = window_size / 2;
    
    // Compute prefix sums for efficient moving average
    let mut prefix_sums = vec![0.0f64; n + 1];
    for i in 0..n { 
        prefix_sums[i + 1] = prefix_sums[i] + wrapped_angles[i]; 
    }
    
    // Apply moving average
    let mut smoothed = vec![0.0f64; n];
    for i in 0..n {
        let left = i.saturating_sub(half_window);
        let right = (i + half_window + 1).min(n);
        let count = (right - left).max(1) as f64;
        smoothed[i] = (prefix_sums[right] - prefix_sums[left]) / count;
    }
    smoothed
}

/// Perform cross-correlation between two angular signals to find optimal time offset.
fn cross_correlate_angular_signals(
    signal_a: &[f64], 
    signal_b: &[f64], 
    sample_rate_hz: f64, 
    max_offset: f64
) -> Option<f64> {
    let n = signal_a.len().min(signal_b.len());
    if n < 8 { return None; }
    
    let max_lag = (max_offset * sample_rate_hz).round() as isize;
    let mut best_correlation = f64::NEG_INFINITY;
    let mut best_lag = 0isize;
    
    // Test all possible lags within the offset range
    for lag in -max_lag..=max_lag {
        let mut correlation_sum = 0.0; 
        let mut valid_samples = 0;
        
        for i in 0..n {
            let j = i as isize + lag;
            if j >= 0 && (j as usize) < n { 
                correlation_sum += signal_a[i] * signal_b[j as usize]; 
                valid_samples += 1; 
            }
        }
        
        if valid_samples > 0 { 
            let correlation = correlation_sum / valid_samples as f64; 
            if correlation > best_correlation { 
                best_correlation = correlation; 
                best_lag = lag; 
            } 
        }
    }
    
    // Convert lag to time offset
    // Positive lag means signal_b (gyro) is delayed relative to signal_a (GPS)
    // To align them, we need to shift GPS forward by the lag amount
    // Therefore return positive lag directly
    Some((best_lag as f64) / sample_rate_hz)
}


#[cfg(test)]
mod tests {
    use super::*;

    /// Generate synthetic test data with optional time offset, scale, heading offset, and drift
    fn generate_test_data(
        n: usize,
        sample_rate: f64,
        time_offset: f64,
        scale: f64,
        heading_offset: f64,
        drift_rate: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut gps_course = Vec::with_capacity(n);
        let mut gyro_yaw = Vec::with_capacity(n);
        
        for i in 0..n {
            let t_gps = i as f64 / sample_rate;
            let t_gyro = t_gps - time_offset;
            
            let base_angle = (t_gps * 2.0).sin() * 60.0 + 90.0;
            let drift = t_gps * drift_rate;
            
            gps_course.push(base_angle + heading_offset + drift);
            gyro_yaw.push((t_gyro * 2.0).sin() * 60.0 / scale + 90.0);
        }
        
        (gps_course, gyro_yaw)
    }

    #[test]
    fn test_synchronize_gps_gyro_basic() {
        // Create synthetic uniformly sampled data with known offset
        let sample_rate = 10.0; // 10 Hz
        let n = 100;
        let time_offset = 2.0; // 2 second offset
        
        let (gps_course, gyro_yaw) = generate_test_data(n, sample_rate, time_offset, 1.0, 0.0, 0.0);
        
        // Test without scale estimation, using angles
        let result = synchronize_gps_gyro(
            &gps_course,
            &gyro_yaw,
            sample_rate,
            5.0, // max offset
            None, // no speed masking
            false, // no scale estimation
            SignalType::Angles,
        );

        assert!(result.is_some());
        let result = result.unwrap();

        println!("Found offset: {:.3}s, expected: {:.3}s", result.time_offset_s, time_offset);
        println!("RMS error: {:.3}°", result.residual_rms_deg);
        println!("Valid samples: {}", result.valid_samples);
        
        // Gyro is earlier by time_offset, so we need to shift GPS forward (positive offset)
        let expected_offset = time_offset;
        assert!((result.time_offset_s - expected_offset).abs() < 0.2);
        assert!(result.yaw_scale.is_none()); // Should not estimate scale
        assert!(result.residual_rms_deg < 5.0); // Low error expected
        assert_eq!(result.valid_samples, n); // All samples used
        assert_eq!(result.signal_type, SignalType::Angles);
    }
    
    #[test]
    fn test_synchronize_gps_gyro_with_speed_masking() {
        let sample_rate = 10.0;
        let n = 100;
        
        let gps_course: Vec<f64> = (0..n).map(|i| (i as f64 * 0.5).sin() * 30.0).collect();
        let gyro_yaw: Vec<f64> = (0..n).map(|i| (i as f64 * 0.5).sin() * 30.0).collect();
        
        // Create speed data with low speeds at the beginning
        let speed: Vec<f64> = (0..n).map(|i| {
            if i < 20 { 0.2 } else { 2.0 } // Low speed for first 20 samples
        }).collect();
        
        let result = synchronize_gps_gyro(
            &gps_course,
            &gyro_yaw,
            sample_rate,
            5.0,
            Some(&speed),
            false,
            SignalType::Angles,
        );

        assert!(result.is_some());
        let result = result.unwrap();

        // Should have fewer valid samples due to masking
        assert!(result.valid_samples < n);
        println!("Masked: {} of {} samples removed", n - result.valid_samples, n);
    }

    #[test]
    fn test_synchronize_gps_gyro_with_scale() {
        let sample_rate = 10.0;
        let n = 100;
        let scale = 1.2; // 20% scale difference
        
        let (gps_course, gyro_yaw) = generate_test_data(n, sample_rate, 0.0, scale, 0.0, 0.0);
        
        let result = synchronize_gps_gyro(
            &gps_course,
            &gyro_yaw,
            sample_rate,
            5.0,
            None,
            true, // Enable scale estimation
            SignalType::Angles,
        );

        assert!(result.is_some());
        let result = result.unwrap();

        if let Some(estimated_scale) = result.yaw_scale {
            println!("Found scale: {:.3}, expected: {:.3}", estimated_scale, scale);
            assert!((estimated_scale - scale).abs() < 0.15); // Within 15% tolerance
        }
    }

    #[test]
    fn test_synchronize_gps_gyro_with_rates() {
        // Test using angular rates instead of angles
        let sample_rate = 10.0;
        let n = 100;
        let time_offset = 1.5; // 1.5 second offset
        let heading_offset = 45.0; // 45° absolute offset
        let drift_rate = 2.0; // Linear drift rate (deg/s)
        
        let (gps_course, gyro_yaw) = generate_test_data(
            n, 
            sample_rate, 
            time_offset, 
            1.0, 
            heading_offset, 
            drift_rate
        );
        
        // Generate speed data (vehicle is moving)
        let speed: Vec<f64> = vec![2.0; n];
        
        // Test with rates - should handle heading offset and drift
        let result_rates = synchronize_gps_gyro(
            &gps_course,
            &gyro_yaw,
            sample_rate,
            5.0,
            Some(&speed),
            false,
            SignalType::Rates,
        );

        assert!(result_rates.is_some());
        let result_rates = result_rates.unwrap();

        let expected_offset = time_offset;  // Gyro is earlier, so GPS offset is positive
        println!("Rates - Found offset: {:.3}s, expected: {:.3}s", result_rates.time_offset_s, expected_offset);
        println!("Rates - RMS error: {:.3}°", result_rates.residual_rms_deg);
        
        // Rates should find the offset despite heading offset and drift
        assert!((result_rates.time_offset_s - expected_offset).abs() < 0.3);
        assert_eq!(result_rates.signal_type, SignalType::Rates);
        
        // Compare with angles - angles might struggle with the offset and drift
        let result_angles = synchronize_gps_gyro(
            &gps_course,
            &gyro_yaw,
            sample_rate,
            5.0,
            Some(&speed),
            false,
            SignalType::Angles,
        );
        
        if let Some(result_angles) = result_angles {
            println!("Angles - Found offset: {:.3}s, RMS error: {:.3}°", 
                     result_angles.time_offset_s, result_angles.residual_rms_deg);
            // Rates should generally be more robust (lower RMS) in this scenario
        }
    }

}
