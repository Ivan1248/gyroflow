// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2022 Ivan Grubišić <ivan.grubisic at gmail>

use super::data::GPSTrack;
use super::processing::{resample_linear, unwrap_angles_deg};
use super::sync::{synchronize_gps_gyro, sample_gyro_yaw_at_times, GpsSyncSettings};
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
    /// TODO: review and improve
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
        let gps_duration_ms = end_abs - start_abs;
        
        let tmin = start_abs.min(end_abs).max(0.0);
        let tmax = start_abs.max(end_abs).min(video_duration_ms);
        let overlap_ms_abs = if tmax > tmin { tmax - tmin } else { 0.0 };
        let overlap_ratio_abs = if video_duration_ms > 0.0 { 
            (overlap_ms_abs / gps_duration_ms).clamp(0.0, 1.0) 
        } else { 
            0.0 
        };

        // Choose anchor: if >= 10% overlap with absolute, use Absolute, otherwise Relative
        let anchor = if overlap_ratio_abs >= 0.1 { GPSAnchor::Absolute } else { GPSAnchor::Relative };

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

        Self { anchor, anchor_time_s, overlap_ms, overlap_ratio }
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
        
        let get_empty_result = || Self {
            times_ms: Vec::new(),
            course_deg: Vec::new(),
            speed_mps: Vec::new(),
        };
        
        if gps_times_ms.is_empty() {
            return get_empty_result();
        }

        let t0_ms = gps_times_ms.first().copied().unwrap().max(0.0);
        let t1_ms = gps_times_ms.last().copied().unwrap().min(video_duration_ms);

        if t1_ms <= t0_ms {
            return get_empty_result();
        }

        // Get raw data
        let course_raw = track.get_course_deg();
        let speed_raw = track.get_speed();

        // CRITICAL: Unwrap angles BEFORE resampling
        let course_unwrapped = unwrap_angles_deg(&course_raw);

        // Resample uniformly
        let (times_ms, course_deg) = resample_linear(&gps_times_ms, &course_unwrapped, t0_ms, t1_ms, sample_rate_hz);
        let (_, speed_mps) = resample_linear(&gps_times_ms, &speed_raw, t0_ms, t1_ms, sample_rate_hz);

        Self { times_ms, course_deg, speed_mps }
    }
}

#[derive(Debug, Clone)]
pub struct GpsSource {
    // computed state
    pub track: Option<GPSTrack>,  // .time in seconds since video creation time
    pub offset_ms: f64,  // setting in case of manual offset
    // settings
    pub sync_mode: GPSSyncMode,
    pub use_processed_motion: bool,
    pub speed_threshold: f64,  // Speed threshold for masking (m/s)
    pub sample_rate_hz: f64,  // GPS synchronization sample rate (Hz)
    pub max_time_offset_s: f64,  // Maximum time shift for GPS sync (seconds)
}

impl Default for GpsSource {
    fn default() -> Self {
        Self {
            track: None,
            offset_ms: 0.0,
            sync_mode: GPSSyncMode::default(),
            use_processed_motion: false,
            speed_threshold: 1.5,  // Default speed threshold
            sample_rate_hz: 10.0,  // Default sample rate
            max_time_offset_s: 15.0,  // Default max time shift
        }
    }
}

impl GpsSource {
    pub fn clear_data(&mut self) {
        self.track = None;
        self.offset_ms = 0.0;
    }
    
    pub fn set_track(&mut self, track: GPSTrack) {
        self.track = Some(track);
        self.offset_ms = 0.0;
    }
    
    pub fn get_speed_threshold(&self) -> f64 {
        self.speed_threshold
    }
    
    pub fn set_speed_threshold(&mut self, threshold: f64) {
        self.speed_threshold = threshold;
    }
    
    pub fn get_sample_rate(&self) -> f64 {
        self.sample_rate_hz
    }
    
    pub fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate_hz = rate;
    }
    
    pub fn get_max_time_offset(&self) -> f64 {
        self.max_time_offset_s
    }
    
    pub fn set_max_time_offset(&mut self, shift: f64) {
        self.max_time_offset_s = shift;
    }

    pub fn set_sync_mode(&mut self, mode: GPSSyncMode) {
        if self.sync_mode != mode {
            self.sync_mode = mode;
            if matches!(mode, GPSSyncMode::Off) {
                self.offset_ms = 0.0;
            }
        }
    }

    pub fn synchronize_with_gyro(&mut self, gyro: &GyroSource) {
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
        let prepared = PreparedGpsData::prepare(track, alignment.anchor_time_s, gyro.duration_ms, self.sample_rate_hz);
        if prepared.times_ms.is_empty() { 
            return; 
        }

        // Step 3: Sample gyro yaw at same timestamps
        let gyro_yaw_deg = sample_gyro_yaw_at_times(gyro, &prepared.times_ms, self.use_processed_motion);

        // Step 4: Synchronize via cross-correlation
        let settings = GpsSyncSettings {
            speed_threshold: self.speed_threshold,  // Use field value
            max_time_offset_s: self.max_time_offset_s,  // Use field value
            sample_rate: self.sample_rate_hz,  // Use field value
        };
        if let Some(result) = synchronize_gps_gyro(
            &prepared.course_deg,
            &gyro_yaw_deg,
            Some(&prepared.speed_mps),
            false,  // No scale estimation needed
            &settings,
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


