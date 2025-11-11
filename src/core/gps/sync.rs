// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2025 Ivan <ivan.grubisic at gmail>

use super::processing::unwrap_angles_deg;
use crate::gyro_source::GyroSource;
use crate::util::median;

/// Macro to validate that all arrays have equal lengths
/// Usage: assert_equal_lengths!(array1, array2, Some(mask));
/// First two arguments are required, rest can be Option<&[T]>
macro_rules! assert_equal_lengths {
    ($first:expr, $second:expr $(, $($rest:expr),*)?) => {
        {
            let first_len = $first.len();
            let second_len = $second.len();
            if second_len != first_len {
                panic!("Length mismatch: {} has length {}, but {} has length {}", 
                       stringify!($first), first_len, stringify!($second), second_len);
            }
            $(
                $(
                    if let Some(rest_slice) = $rest {
                        if rest_slice.len() != first_len {
                            panic!("Length mismatch: {} has length {}, but {} has length {}", 
                                   stringify!($first), first_len, stringify!($rest), rest_slice.len());
                        }
                    }
                )*
            )?
            first_len
        }
    };
}

pub const DEFAULT_SPEED_THRESHOLD: f64 = 1.5;
pub const DEFAULT_MAX_TIME_OFFSET_S: f64 = 6.0;
pub const DEFAULT_SAMPLE_RATE: f64 = 10.0;
const MIN_NUM_SYNC_SAMPLES: usize = 8;

/// GPS synchronization method
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum GpsSyncMethod {
    L1,  // L1 distance method
    MedL2,  // Median L2 distance method
    TrimmedL2,  // Trimmed L2 distance method (uses 90% of shortest squared distances)
}

impl Default for GpsSyncMethod {
    fn default() -> Self {
        Self::L1
    }
}

/// GPS synchronization settings
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct GpsSyncSettings {
    pub speed_threshold: f64,  // m/s
    pub time_offset_range_s: (f64, f64),  // (min_offset_s, max_offset_s) in seconds
    pub sample_rate: f64,  // Hz
    pub method: GpsSyncMethod,  // synchronization method
}

impl Default for GpsSyncSettings {
    fn default() -> Self {
        Self {
            speed_threshold: DEFAULT_SPEED_THRESHOLD, // m/s
            time_offset_range_s: (-DEFAULT_MAX_TIME_OFFSET_S, DEFAULT_MAX_TIME_OFFSET_S),
            sample_rate: DEFAULT_SAMPLE_RATE,
            method: GpsSyncMethod::default(),
        }
    }
}

/// Result of GPS-gyro synchronization
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct GpsSyncResult {
    /// Time offset in seconds to add to GPS timestamps to align with gyro
    /// Positive: GPS is late (add time), negative: GPS is early (subtract time)
    pub time_offset_s: f64,
    pub valid_count: usize,   // number of samples used after masking
    pub error: f64,  // RMS angular error in degrees
    pub correlation: f64,       // correlation coefficient from cross-correlation
    pub course_range_deg: f64,  // Range of course angles over valid points (degrees)
}

pub fn sample_gyro_yaw_at_times(
    gyro: &GyroSource,
    times_ms: &[f64],
    use_processed_motion: bool,
) -> Vec<f64> {
    times_ms
        .iter()
        .map(|&t| {
            let q = if use_processed_motion {
                gyro.actual_smoothed_quat_at_timestamp(t)
            } else {
                gyro.org_quat_at_timestamp(t)
            };
            let (_roll, _pitch, yaw) = q.euler_angles();
            yaw.to_degrees()
        })
        .collect()
}

/// Synchronize GPS course with gyro yaw to find time offset and optional scale factor.
///
/// This function correlates uniformly sampled GPS course (heading) with gyro yaw angles
/// to determine the time offset needed to align them. It automatically:
/// - Validates that all input arrays have equal lengths (panics on mismatch)
/// - Filters out low-speed segments where GPS heading is unreliable (if speed provided)
/// - Preprocesses signals (unwrapping, rate computation, smoothing) before masking
/// - Computes quality metrics (RMS error, valid sample count)
///
/// # Arguments
/// * `gps_course_deg` - GPS course angles in degrees [0-360) (uniformly sampled)
/// * `gyro_yaw_deg` - Gyro yaw angles in degrees (uniformly sampled)
/// * `speed_mps` - Optional GPS speed in m/s for masking (must match course/yaw length if provided)
/// * `estimate_scale` - Enable scale factor estimation (typically `false` unless sensors differ)
/// * `settings` - Synchronization settings (thresholds, sample rate, max offset)
///
/// # Returns
/// `Some(GpsSyncResult)` with time offset, optional scale, RMS error, and valid sample count.
/// `None` if insufficient data (< 8 valid samples after masking).
///
/// # Panics
/// Panics if input arrays have different lengths or if speed array length doesn't match data arrays.
///
/// # Example
/// ```ignore
/// use gyroflow_core::gps::sync::*;
///
/// // After resampling GPS and gyro data to 10 Hz...
/// let settings = GpsSyncSettings::default();
/// let result = synchronize_gps_gyro(
///     &gps_course_deg,
///     &gyro_yaw_deg,
///     Some(&speed_mps),         // Enable speed masking (recommended)
///     false,                     // No scale estimation
///     &settings,
/// )?;
///
/// println!("Time offset: {:.3}s", result.time_offset_s);
/// println!("RMS error: {:.2}°", result.residual_rms_deg);
/// println!("Valid samples: {}", result.valid_samples);
/// ```
pub fn synchronize_gps_gyro(
    gps_course_deg: &[f64],
    gyro_yaw_deg: &[f64],
    speed_mps: Option<&[f64]>,
    estimate_scale: bool,
    settings: &GpsSyncSettings,
) -> Option<GpsSyncResult> {
    if gps_course_deg.is_empty() || gyro_yaw_deg.is_empty() || settings.sample_rate <= 0.0 {
        return None;
    }
    let n = assert_equal_lengths!(gps_course_deg, gyro_yaw_deg);
    if n < MIN_NUM_SYNC_SAMPLES {
        return None;
    }

    // Preprocess signals
    let course_processed = preprocess_angular_signal(&gps_course_deg[..n], settings.sample_rate);
    let yaw_processed = preprocess_angular_signal(&gyro_yaw_deg[..n], settings.sample_rate);

    // Create validity mask (optional, based on speed)
    let mask = speed_mps.map(|speed| create_validity_mask(speed, settings.speed_threshold));

    // Validate minimum valid samples after masking
    let valid_count = mask.as_ref().map_or(n, |m| m.iter().filter(|&&v| v).count());
    if valid_count < MIN_NUM_SYNC_SAMPLES {
        return None;
    }

    // Find time offset via cross-correlation (using mask)
    let error_fn = match settings.method {
        GpsSyncMethod::L1 => l1_distance,
        GpsSyncMethod::MedL2 => median_squared_error,
        GpsSyncMethod::TrimmedL2 => trimmed_mse,
    };
    let (offset, error) = find_time_offset(
        &course_processed,
        &yaw_processed,
        mask.as_deref(),
        settings.sample_rate,
        settings.time_offset_range_s,
        error_fn,
    );

    let correlation = compute_error(
        &course_processed,
        &yaw_processed,
        mask.as_deref(),
        settings.sample_rate,
        offset,
        signal_pearson_correlation
    );

    let course_range_deg = compute_course_range(gps_course_deg, mask.as_deref());

    Some(GpsSyncResult {
        time_offset_s: offset,
        valid_count: valid_count,
        error,
        correlation: correlation,
        course_range_deg: course_range_deg,
    })
}

/// Wrap angle to -180 to 180 degrees
#[inline]
fn wrap_angle_deg(d: f64) -> f64 {
    (d + 180.0).rem_euclid(360.0) - 180.0
}

/// Create validity mask from speed data.
fn create_validity_mask(speed_mps: &[f64], speed_threshold: f64) -> Vec<bool> {
    speed_mps.iter().map(|&s| s >= speed_threshold).collect()
}

/// Unwrap angles, convert to rates, smooth.
pub fn preprocess_angular_signal(angles: &[f64], sample_rate_hz: f64) -> Vec<f64> {
    let unwrapped = unwrap_angles_deg(angles);
    let rates = compute_angular_rates(&unwrapped, sample_rate_hz);
    smooth_angular_signal(&rates, sample_rate_hz)
}

/// Compute correlation between two aligned signals with optional masking.
fn signal_correlation(
    sig_a: &[f64],
    sig_b: &[f64],
    mask_a: Option<&[bool]>,
) -> f64 {
    let n = assert_equal_lengths!(sig_a, sig_b, mask_a);
    let mut correlation_sum = 0.0;
    let mut valid_samples = 0;

    for i in 0..n {
        if mask_a.map_or(true, |m| m[i]) {
            correlation_sum += sig_a[i] * sig_b[i];
            valid_samples += 1;
        }
    }

    correlation_sum / valid_samples as f64
}

/// Compute the range of course angles over valid points with optional masking.
fn compute_course_range(gps_course_deg: &[f64], mask: Option<&[bool]>) -> f64 {
    let n = gps_course_deg.len();
    if let Some(mask) = mask {
        assert_eq!(mask.len(), n, "Mask length {} doesn't match course length {}", mask.len(), n);
        let mut min_course = f64::INFINITY;
        let mut max_course = f64::NEG_INFINITY;
        for i in 0..n {
            if mask[i] {
                let course = gps_course_deg[i];
                min_course = min_course.min(course);
                max_course = max_course.max(course);
            }
        }
        if min_course == f64::INFINITY { 0.0 } else { max_course - min_course }
    } else {
        let min_course = gps_course_deg.iter().fold(f64::INFINITY, |a, &b| a.min(b));
        let max_course = gps_course_deg.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
        max_course - min_course
    }
}

/// Compute Pearson correlation coefficient between two aligned signals with optional masking.
pub fn signal_pearson_correlation(
    sig_a: &[f64],
    sig_b: &[f64],
    mask_a: Option<&[bool]>,
) -> f64 {
    let n = assert_equal_lengths!(sig_a, sig_b, mask_a);

    // Compute means, respecting mask
    let (mean_a, mean_b, valid_samples) = if let Some(mask) = mask_a {
        let mut sum_a = 0.0;
        let mut sum_b = 0.0;
        let mut count = 0;
        for i in 0..n {
            if mask[i] {
                sum_a += sig_a[i];
                sum_b += sig_b[i];
                count += 1;
            }
        }
        (sum_a / count as f64, sum_b / count as f64, count)
    } else {
        let sum_a: f64 = sig_a.iter().sum();
        let sum_b: f64 = sig_b.iter().sum();
        (sum_a / n as f64, sum_b / n as f64, n)
    };

    // Compute covariance and variances
    let mut covariance = 0.0;
    let mut var_a = 0.0;
    let mut var_b = 0.0;

    for i in 0..n {
        if mask_a.map_or(true, |m| m[i]) {
            let diff_a = sig_a[i] - mean_a;
            let diff_b = sig_b[i] - mean_b;
            covariance += diff_a * diff_b;
            var_a += diff_a * diff_a;
            var_b += diff_b * diff_b;
        }
    }

    covariance / (var_a.sqrt() * var_b.sqrt())
}

pub fn l1_distance(
    sig_a: &[f64],
    sig_b: &[f64],
    mask_a: Option<&[bool]>,
) -> f64 {
    let n = assert_equal_lengths!(sig_a, sig_b, mask_a);
    let mut l1_distance = 0.0;
    for i in 0..n {
        if mask_a.map_or(true, |m| m[i]) {
            l1_distance += (sig_a[i] - sig_b[i]).abs();
        }
    }
    l1_distance / n as f64
}

fn squared_diffs(sig_a: &[f64], sig_b: &[f64], mask_a: Option<&[bool]>) -> Vec<f64> {
    let n = assert_equal_lengths!(sig_a, sig_b, mask_a);
    let mut squared_diffs: Vec<f64> = (0..n)
        .filter(|&i| mask_a.map_or(true, |m| m[i]))
        .map(|i| (sig_a[i] - sig_b[i]).powi(2))
        .collect();
    squared_diffs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    squared_diffs
}

pub fn median_squared_error(
    sig_a: &[f64],
    sig_b: &[f64],
    mask_a: Option<&[bool]>,
) -> f64 {
    let squared_diffs = squared_diffs(sig_a, sig_b, mask_a);
    if squared_diffs.is_empty() { 0.0 } else {median(squared_diffs)}
}

pub fn trimmed_mse(
    sig_a: &[f64],
    sig_b: &[f64],
    mask_a: Option<&[bool]>,
) -> f64 {
    let squared_diffs = squared_diffs(sig_a, sig_b, mask_a);
    if squared_diffs.is_empty() { 
        return 0.0;
    }
    // Average 90% of the shortest distances
    let used_count = (squared_diffs.len() as f64 * 0.9) as usize;
    squared_diffs[..used_count].iter().sum::<f64>() 
}

/// Helper function to compute ranks with proper tie handling
fn compute_ranks(pairs: &[(f64, usize)]) -> Vec<f64> {
    let mut ranks = vec![0.0; pairs.len()];
    let mut i = 0;

    while i < pairs.len() {
        let mut j = i + 1;
        // Find all tied values
        while j < pairs.len() && pairs[j].0 == pairs[i].0 {
            j += 1;
        }

        // Assign average rank to tied values
        let avg_rank = (i + j - 1) as f64 / 2.0 + 1.0;
        for k in i..j {
            ranks[k] = avg_rank;
        }

        i = j;
    }

    ranks
}

/// Find time offset between signals using cross-correlation.
fn find_time_offset<F>(
    sig_a: &[f64],
    sig_b: &[f64],
    mask_a: Option<&[bool]>,
    sample_rate: f64,
    time_offset_range_s: (f64, f64),
    error_fn: F,
) -> (f64, f64)
where
    F: Fn(&[f64], &[f64], Option<&[bool]>) -> f64,
{
    let n = assert_equal_lengths!(sig_a, sig_b, mask_a);
    let min_offset = (time_offset_range_s.0 * sample_rate).round() as isize;
    let max_offset = (time_offset_range_s.1 * sample_rate).round() as isize;
    let mut best_error = f64::INFINITY;
    let mut best_offset = 0isize;

    // Test all possible lags within the offset range
    for offset in min_offset..=max_offset {
        // Create aligned slices for this offset
        let abs_offset = offset.unsigned_abs();
        if abs_offset >= n { continue; }
        let len = n - abs_offset;

        let error = if offset >= 0 {
            error_fn(&sig_a[..len], &sig_b[abs_offset..], mask_a.map(|m| &m[..len]))
        } else {
            error_fn(&sig_a[abs_offset..], &sig_b[..len], mask_a.map(|m| &m[abs_offset..]))
        };
        if error < best_error || (error == best_error && offset.abs() < best_offset.abs()) {
            best_error = error;
            best_offset = offset;
        }
    }

    // To align the signals, we need to shift signal_b forward by the offset
    ((best_offset as f64) / sample_rate, best_error)
}

pub fn compute_error<F>(
    sig_a: &[f64],
    sig_b: &[f64],
    mask_a: Option<&[bool]>,
    sample_rate: f64,
    time_offset_s: f64,
    error_fn: F
) -> f64
where
    F: Fn(&[f64], &[f64], Option<&[bool]>) -> f64,
{
    let (offset, error) = find_time_offset(sig_a, sig_b, mask_a, sample_rate, (time_offset_s, time_offset_s), error_fn);
    assert_eq!(offset, time_offset_s);
    -error
}

/// Refine time offset and estimate scale factor using optimization.
fn refine_offset_and_scale(
    course_deg: &[f64],
    yaw_deg: &[f64],
    course_mask: Option<&[bool]>,
    initial_offset_s: f64,
    sample_rate_hz: f64,
    time_offset_range_s: (f64, f64),
) -> (f64, Option<f64>) {
    let n = assert_equal_lengths!(course_deg, yaw_deg, course_mask);
    let course_unwrapped = unwrap_angles_deg(course_deg);
    let yaw_unwrapped = unwrap_angles_deg(yaw_deg);

    // Search within the provided offset range
    let min_offset = ((initial_offset_s + time_offset_range_s.0) * sample_rate_hz).round() as isize;
    let max_offset = ((initial_offset_s + time_offset_range_s.1) * sample_rate_hz + 0.5).round() as isize;

    let mut best_mse = f64::INFINITY;
    let mut best_offset_s = initial_offset_s;
    let mut best_scale = 1.0;

    for offset in min_offset..=max_offset {
        // Build aligned slices and mask
        let abs_offset = offset.unsigned_abs();
        if abs_offset >= n { continue; }
        let len = n - abs_offset;
        if len < MIN_NUM_SYNC_SAMPLES { continue; }
        let (yaw_range, course_range) = if offset >= 0 {
            (0..len, abs_offset..n)
        } else {
            (abs_offset..n, 0..len)
        };
        let yaw_slice = &yaw_unwrapped[yaw_range];
        let course_slice = &course_unwrapped[course_range.clone()];
        let course_mask_slice = course_mask.map(|m| &m[course_range]);

        let (scale, angle_offset_deg) = linear_regression(yaw_slice, course_slice, course_mask_slice);

        // Apply affine transformation: course = scale * yaw + offset
        let transformed_yaw: Vec<f64> = yaw_slice.iter().map(|&y| scale * y + angle_offset_deg).collect();
        let mse = angular_mean_squared_error(&transformed_yaw, course_slice, course_mask_slice);

        if mse < best_mse {
            best_mse = mse;
            best_offset_s = -(offset as f64) / sample_rate_hz;
            best_scale = scale;
        }
    }
    println!("initial_offset_s: {}, best_offset_s: {}, best_scale: {}, best_mse: {}", initial_offset_s, best_offset_s, best_scale, best_mse);
    (best_offset_s, Some(best_scale))
}

/// Compute RMS angular error between aligned signals.
fn compute_alignment_residual(
    course_deg: &[f64],
    yaw_deg: &[f64],
    mask: Option<&[bool]>,
    time_offset_s: f64,
    scale: f64,
    sample_rate_hz: f64,
) -> f64 {
    let n = assert_equal_lengths!(course_deg, yaw_deg, mask);
    if n == 0 {
        return f64::INFINITY;
    }

    let offset = (time_offset_s * sample_rate_hz).round() as isize;

    let mut sum_sq = 0.0;
    let mut count = 0;

    for i in 0..n {
        let j = i as isize + offset;
        if j >= 0 && (j as usize) < n {
            // Include valid points
            if mask.map_or(true, |m| m[i]) {
                let course_val = course_deg[i];
                let yaw_val = yaw_deg[j as usize] * scale;

                let mut diff = wrap_angle_deg(course_val - yaw_val);
                sum_sq += diff * diff;
                count += 1;
            }
        }
    }

    (sum_sq / count as f64).sqrt()
}

/// Compute angular rates from unwrapped angles using central differences.
pub fn compute_angular_rates(angles_deg: &[f64], sample_rate_hz: f64) -> Vec<f64> {
    let n = angles_deg.len();
    if n < 2 {
        return vec![];
    }

    let dt = 1.0 / sample_rate_hz;
    let mut rates = vec![0.0; n];

    rates[0] = (angles_deg[1] - angles_deg[0]) / dt;  // forward difference
    for i in 1..n-2 {
        rates[i] = (angles_deg[i + 1] - angles_deg[i - 1]) / (2.0 * dt); // central difference
    }
    rates[n - 1] = (angles_deg[n - 1] - angles_deg[n - 2]) / dt;  // backward difference

    rates
}

/// Apply variance-based weighting to GPS course data to downweight flat segments.
fn apply_variance_weighting_to_course(course_deg: &[f64]) -> Vec<f64> {
    let n = course_deg.len();
    if n < 8 {
        return course_deg.to_vec();
    }

    let wrapped_course: Vec<f64> = course_deg.iter().map(|&v| wrap_angle_deg(v)).collect();

    // Compute local variance weights (5-sample window)
    let window_len = 5usize.min(n.max(1));
    let mut weights: Vec<f64> = vec![1.0; n];
    if window_len >= 3 {
        for i in 0..n {
            let start = i.saturating_sub(window_len / 2);
            let end = (i + window_len / 2 + 1).min(n);
            let segment = &wrapped_course[start..end];
            let mean = segment.iter().copied().sum::<f64>() / (segment.len() as f64);
            let variance = segment.iter().map(|v| (v - mean) * (v - mean)).sum::<f64>()
                / (segment.len() as f64);
            weights[i] = (variance / 25.0).min(1.0); // promote segments with >~5 deg^2 variance
        }
    }

    wrapped_course.iter().enumerate().map(|(i, v)| *v * weights[i]).collect()
}

/// Smooth angular signal using moving average to improve correlation robustness.
pub fn smooth_angular_signal(angles_deg: &[f64], sample_rate_hz: f64) -> Vec<f64> {
    let n = angles_deg.len();
    if n < 5 || sample_rate_hz <= 0.0 {
        return angles_deg.to_vec();
    }

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


/// Simple linear regression y ≈ a*x + c (optionally masked)
fn linear_regression(x: &[f64], y: &[f64], x_mask: Option<&[bool]>) -> (f64, f64) {
    let n = assert_equal_lengths!(x, y, x_mask);
    if n == 0 {
        return (1.0, 0.0);
    }
    
    // Compute mean, respecting mask
    let (mx, my, count) = if let Some(m) = x_mask {
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut cnt = 0;
        for i in 0..n {
            if i < m.len() && m[i] {
                sum_x += x[i];
                sum_y += y[i];
                cnt += 1;
            }
        }
        if cnt == 0 { return (1.0, 0.0); }
        (sum_x / cnt as f64, sum_y / cnt as f64, cnt)
    } else {
        let sum_x: f64 = x[..n].iter().sum();
        let sum_y: f64 = y[..n].iter().sum();
        (sum_x / n as f64, sum_y / n as f64, n)
    };
    
    // Compute regression coefficients
    let mut num = 0.0;
    let mut den = 0.0;
    for i in 0..n {
        if x_mask.map_or(true, |m| m[i]) {
            let dx = x[i] - mx;
            num += dx * (y[i] - my);
            den += dx * dx;
        }
    }
        
    let a: f64 = if den.abs() > 1e-12 { num / den } else { 1.0 };
    let c: f64 = my - a * mx;
    (a, c)
}

/// Compute a mean error
fn mean_error<F>(x: &[f64], y: &[f64], mask: Option<&[bool]>, error_fn: F) -> f64
where F: Fn(f64) -> f64
{
    let n = assert_equal_lengths!(x, y, mask);
    
    let mut sum = 0.0;
    let mut count = 0;
    for i in 0..n {
        if mask.map_or(true, |m| m[i]) {
            sum += error_fn(y[i] - x[i]);
            count += 1;
        }
    }
    
    sum / count as f64
}

/// Compute wrapped angular mean squared error between x and y (optionally masked)
fn angular_mean_squared_error(x: &[f64], y: &[f64], mask: Option<&[bool]>) -> f64 {
    mean_error(x, y, mask, |d| {
        let wrapped = wrap_angle_deg(d);
        wrapped * wrapped
    })
}

/// Compute wrapped angular mean absolute error between x and y (optionally masked)
fn angular_mean_absolute_error(x: &[f64], y: &[f64], mask: Option<&[bool]>) -> f64 {
    mean_error(x, y, mask, |d| wrap_angle_deg(d).abs())
}