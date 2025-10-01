// SPDX-License-Identifier: GPL-3.0-or-later

use crate::gps::GPSTrack;
use crate::gps::synchronization::{ resample_linear, synchronize_gps_gyro, sample_gyro_yaw_at_times, SignalType };
use crate::gyro_source::GyroSource;

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum GPSSyncMode { Off, Auto, Manual }
impl Default for GPSSyncMode { fn default() -> Self { GPSSyncMode::Off } }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GPSAnchor { Absolute, Relative }

impl GPSAnchor {
    pub fn as_label(&self) -> &'static str {
        match self { GPSAnchor::Absolute => "video creation time", GPSAnchor::Relative => "GPS start time" }
    }
}

/// Time alignment information between GPS track and video.
#[derive(Debug, Clone, Copy)]
pub struct TimeAlignment {
    pub anchor: GPSAnchor,
    pub anchor_time_s: f64,
    pub overlap_ms: f64,
    pub overlap_ratio: f64,
}

impl TimeAlignment {
    /// Compute time alignment between GPS track and video.
    pub fn compute(track: &GPSTrack, video_duration_ms: f64, apply_offset: bool, offset_ms: f64) -> Self {
        if track.time.is_empty() {
            return Self {
                anchor: GPSAnchor::Absolute,
                anchor_time_s: 0.0,
                overlap_ms: 0.0,
                overlap_ratio: 0.0,
            };
        }

        // First determine anchor based on absolute overlap
        let anchor_time_abs = 0.0;
        let offset_to_apply = if apply_offset { offset_ms } else { 0.0 };
        
        let start_abs = (track.time.first().copied().unwrap_or(0.0) - anchor_time_abs) * 1000.0 + offset_to_apply;
        let end_abs = (track.time.last().copied().unwrap_or(0.0) - anchor_time_abs) * 1000.0 + offset_to_apply;
        
        let tmin = start_abs.min(end_abs).max(0.0);
        let tmax = start_abs.max(end_abs).min(video_duration_ms);
        let overlap_ms_abs = if tmax > tmin { tmax - tmin } else { 0.0 };
        let overlap_ratio_abs = if video_duration_ms > 0.0 { 
            (overlap_ms_abs / video_duration_ms).clamp(0.0, 1.0) 
        } else { 
            0.0 
        };

        // Choose anchor: if >= 10% overlap with absolute, use Absolute, otherwise Relative
        let anchor = if overlap_ratio_abs >= 0.1 {
            GPSAnchor::Absolute
        } else {
            GPSAnchor::Relative
        };

        // Now compute with chosen anchor
        let anchor_time_s = match anchor {
            GPSAnchor::Absolute => 0.0,
            GPSAnchor::Relative => track.time.first().copied().unwrap_or(0.0),
        };

        let start = (track.time.first().copied().unwrap_or(0.0) - anchor_time_s) * 1000.0 + offset_to_apply;
        let end = (track.time.last().copied().unwrap_or(0.0) - anchor_time_s) * 1000.0 + offset_to_apply;
        
        let tmin = start.min(end).max(0.0);
        let tmax = start.max(end).min(video_duration_ms);
        let overlap_ms = if tmax > tmin { tmax - tmin } else { 0.0 };
        let overlap_ratio = if video_duration_ms > 0.0 {
            (overlap_ms / video_duration_ms).clamp(0.0, 1.0)
        } else {
            0.0
        };

        Self {
            anchor,
            anchor_time_s,
            overlap_ms,
            overlap_ratio,
        }
    }
}

/// Prepared GPS data ready for synchronization (unwrapped and resampled).
pub struct PreparedGpsData {
    pub times_ms: Vec<f64>,
    pub course_deg: Vec<f64>,  // Unwrapped
    pub speed_mps: Vec<f64>,
}

impl PreparedGpsData {
    /// Prepare GPS data for synchronization: unwrap angles, then resample uniformly.
    pub fn prepare(
        track: &GPSTrack,
        anchor_time_s: f64,
        video_duration_ms: f64,
        sample_rate_hz: f64,
    ) -> Self {
        let gps_times_ms: Vec<f64> = track.time.iter()
            .map(|t| (*t - anchor_time_s) * 1000.0)
            .collect();

        if gps_times_ms.is_empty() {
            return Self {
                times_ms: Vec::new(),
                course_deg: Vec::new(),
                speed_mps: Vec::new(),
            };
        }

        let t0_ms = gps_times_ms.first().copied().unwrap().max(0.0);
        let t1_ms = gps_times_ms.last().copied().unwrap().min(video_duration_ms);

        if t1_ms <= t0_ms {
            return Self {
                times_ms: Vec::new(),
                course_deg: Vec::new(),
                speed_mps: Vec::new(),
            };
        }

        // Get raw data
        let course_raw = track.course_deg();
        let speed_raw = track.speed();

        // CRITICAL: Unwrap angles BEFORE resampling
        let course_unwrapped = super::unwrap_angles_deg(&course_raw);

        // Resample uniformly
        let (times_ms, course_deg) = resample_linear(&gps_times_ms, &course_unwrapped, t0_ms, t1_ms, sample_rate_hz);
        let (_, speed_mps) = resample_linear(&gps_times_ms, &speed_raw, t0_ms, t1_ms, sample_rate_hz);

        Self { times_ms, course_deg, speed_mps }
    }
}

#[derive(Default, Debug, Clone)]
pub struct GpsSource {
    pub track: Option<GPSTrack>,  // .time in seconds since video creation time
    pub sync_mode: GPSSyncMode,
    pub offset_ms: f64,
}

impl GpsSource {
    pub fn clear(&mut self) {
        self.track = None;
        self.sync_mode = GPSSyncMode::default();
        self.offset_ms = 0.0;
    }

    pub fn set_track(&mut self, track: GPSTrack) {
        self.track = Some(track);
        self.offset_ms = 0.0;
    }

    pub fn set_sync_mode(&mut self, mode: GPSSyncMode) {
        if self.sync_mode != mode {
            self.sync_mode = mode;
            if matches!(mode, GPSSyncMode::Off) {
                self.offset_ms = 0.0;
            }
        }
    }

    pub fn synchronize_with_gyro(&mut self, gyro: &GyroSource, sample_rate_hz: f64, max_offset: f64) {
        let track = match self.track.as_ref() {
            Some(t) => t,
            None => return,
        };

        // Step 1: Compute time alignment
        let alignment = TimeAlignment::compute(track, gyro.duration_ms, false, 0.0);
        if alignment.overlap_ms < 2000.0 { 
            return;  // Require at least 2s overlap
        }

        // Step 2: Prepare GPS data (unwraps angles before resampling)
        let prepared = PreparedGpsData::prepare(track, alignment.anchor_time_s, gyro.duration_ms, sample_rate_hz);
        if prepared.times_ms.is_empty() { 
            return; 
        }

        // Step 3: Sample gyro yaw at same timestamps
        let gyro_yaw_deg = sample_gyro_yaw_at_times(gyro, &prepared.times_ms, false);

        // Step 4: Synchronize via cross-correlation
        if let Some(result) = synchronize_gps_gyro(
            &prepared.course_deg,
            &gyro_yaw_deg,
            sample_rate_hz,
            max_offset,
            Some(&prepared.speed_mps),
            false,  // No scale estimation needed
            SignalType::Rates,  // Use angles for now
        ) {
            self.offset_ms = (result.time_offset_s - alignment.anchor_time_s) * 1000.0;
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.sync_mode != GPSSyncMode::Off
    }

    pub fn summary_json(&self, video_duration_ms: f64) -> serde_json::Value {
        match self.track.as_ref() {
            None => serde_json::json!({ "loaded": false }),
            Some(track) => {
                let alignment = TimeAlignment::compute(track, video_duration_ms, true, self.offset_ms);
                serde_json::json!({
                    "loaded": true,
                    "points": track.len(),
                    "epoch_start_s": track.get_start_time(),
                    "epoch_end_s": track.time.last(),
                    "anchor": alignment.anchor.as_label(),
                    "overlap_ms": alignment.overlap_ms,
                    "overlap_ratio": alignment.overlap_ratio,
                })
            }
        }
    }
}


