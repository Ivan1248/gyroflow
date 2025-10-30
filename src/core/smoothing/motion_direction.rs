// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2025

use super::*;
use std::sync::Mutex;

pub struct MotionDirectionAlignment {
    pub enabled: bool,
    pub min_inlier_ratio: f64,
    pub flip_backward_dir: bool,
    status: Mutex<Vec<serde_json::Value>>,
}

impl Default for MotionDirectionAlignment {
    fn default() -> Self {
        Self {
            enabled: false,
            min_inlier_ratio: Self::DEFAULT_MIN_INLIER_RATIO,
            // TODO: flip_backward_dir=true is more stable when stationary, but looks in the reverse motion direction if going backward
            flip_backward_dir: false,
            status: Mutex::new(vec![serde_json::json!({
                "type": "Label",
                "text": "Cannot apply motion direction alignment: motion directions not estimated."
            })]),
        }
    }
}

impl Clone for MotionDirectionAlignment {
    fn clone(&self) -> Self {
        let status = self.status.lock().unwrap().clone();
        Self {
            enabled: self.enabled,
            min_inlier_ratio: self.min_inlier_ratio,
            flip_backward_dir: self.flip_backward_dir,
            status: Mutex::new(status)
        }
    }
}

impl MotionDirectionAlignment {
    /// Default parameters for motion direction alignment
    const DEFAULT_MIN_INLIER_RATIO: f64 = 0.5;

    /// Time window constants (in microseconds)
    const VALID_TIME_RADIUS_US: i64 = 50_000; // 50ms
    const FALLBACK_TIME_RADIUS_US: i64 = 250_000; // 250ms
    /// Small epsilon for floating point comparisons
    const DEGENERATE_NORM_EPSILON: f64 = 1e-6;
    /// Threshold for converting float to bool
    const BOOL_THRESHOLD: f64 = 0.5;

    /// Convert translation direction to camera motion direction
    fn translation_to_motion_direction(&self, tdir: [f64; 3], compute_params: &ComputeParams) -> nalgebra::Vector3<f64> {
        // Flip sign to get camera motion direction
        let sign_corr = if tdir[2] <= 0.0 { /*forward*/ -1.0} else if self.flip_backward_dir {1.0} else {-1.0};
        // If is_back_camera, convert from back camera frame to front camera frame by 180° rotation about Y and flipping the sign
        let back_corr = if compute_params.is_back_camera { -1.0 } else { 1.0 };
        sign_corr * nalgebra::Vector3::new(tdir[0], back_corr * tdir[1], tdir[2])
    }

    /// Find the closest frame with good quality motion data within an expanded search window
    /// Returns (motion_direction, time_distance_us) where time_distance_us is how far back/forward in time the data came from
    fn find_closest_good_motion_data(&self, compute_params: &ComputeParams, timestamp_us: i64, time_radius_us: i64) -> Option<(nalgebra::Vector3<f64>, i64)> {
        if let Some((tdir, _, dist)) = compute_params.pose_estimator.find_closest_transl_dir(timestamp_us, time_radius_us, Some(self.min_inlier_ratio)) {
            let motion_dir = self.translation_to_motion_direction(tdir, compute_params);
            return Some((motion_dir, dist));
        }
        None
    }

    pub fn set_parameter(&mut self, name: &str, val: f64) {
        match name {
            "min_inlier_ratio" => self.min_inlier_ratio = val,
            "flip_backward_dir" => self.flip_backward_dir = val > Self::BOOL_THRESHOLD,
            _ => log::error!("Invalid parameter name: {}", name)
        }
    }
    pub fn get_parameter(&self, name: &str) -> f64 {
        match name {
            "min_inlier_ratio" => self.min_inlier_ratio,
            "flip_backward_dir" => if self.flip_backward_dir { 1.0 } else { 0.0 },
            _ => 0.0
        }
    }

    pub fn get_parameters_json(&self) -> serde_json::Value {
        serde_json::json!([
        {
            "name": "min_inlier_ratio",
            "description": "Min. inlier ratio",
            "type": "SliderWithField",
            "from": 0.0,
            "to": 1.0,
            "value": self.min_inlier_ratio,
            "default": Self::DEFAULT_MIN_INLIER_RATIO,
            "precision": 2,
            "unit": ""
        },{
            "name": "flip_backward_dir",
            "description": "Flip backward direction",
            "type": "CheckBox",
            "default": self.flip_backward_dir,
            "value": self.flip_backward_dir,
            "advanced": true
        }])
    }
    pub fn get_status_json(&self) -> serde_json::Value { serde_json::Value::Array(self.status.lock().unwrap().clone()) }
    pub fn get_checksum(&self) -> u64 {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        hasher.write_u8(self.enabled as u8);
        hasher.write_u64(self.min_inlier_ratio.to_bits());
        hasher.write_u64(self.flip_backward_dir as u64);
        hasher.finish()
    }

    fn build_status_messages(num_frames: usize, num_used: usize, num_valid: usize) -> Vec<serde_json::Value> {
        let mut msgs: Vec<serde_json::Value> = Vec::new();
        if num_frames > 0 {
            let used_percent = (num_used as f64 * 100.0 / num_frames as f64).round();
            let valid_percent = (num_valid as f64 * 100.0 / num_frames as f64).round();
            msgs.push(serde_json::json!({
                "type": "Label",
                "text": format!(
                    "Motion direction: {:.0}% of frames have good quality motion data ({:.0}% within {} ms)",
                    valid_percent,
                    used_percent,
                    Self::FALLBACK_TIME_RADIUS_US / 1000
                )
            }));
            if num_used == 0 {
                msgs.push(serde_json::json!({
                    "type": "Label",
                    "text": format!("No valid motion directions in video. Consider changing settings.")
                }));
            }
        }
        msgs
    }
    
    fn compute_aligned_quaternion(
        quat: &nalgebra::UnitQuaternion<f64>,
        motion_dir: nalgebra::Vector3<f64>
    ) -> Option<nalgebra::UnitQuaternion<f64>> {
        let norm = motion_dir.norm();
        if norm > Self::DEGENERATE_NORM_EPSILON {
            let ld = motion_dir / norm;
            // X = right, Y = up, Z = forward (left-handed)
            // For some reason, X has to be flipped to match the quaternion-to-image convention
            let dir_body = nalgebra::Vector3::<f64>::new(-ld.x, ld.y, ld.z);
            // Transform motion direction to world frame
            let dir_world = *quat * dir_body;
            // Left-multiplication with quat seems to preserve roll
            let up_world = *quat * nalgebra::Vector3::<f64>::new(0.0, 1.0, 0.0);
            Some(nalgebra::UnitQuaternion::face_towards(&dir_world, &up_world))
        } else {
            None
        }
    }

    pub fn apply(&self, quats: &TimeQuat, duration_ms: f64, compute_params: &ComputeParams) -> TimeQuat {
        // Early return if disabled or if pose estimator has any motion data
        if !self.enabled {
            return quats.clone();
        }
        // Check if pose estimator has any motion data
        if compute_params.pose_estimator.sync_results.try_read().map_or(true, |sr| !sr.values().any(|fr| fr.transl_dir.is_some())) {
            *self.status.lock().unwrap() = MotionDirectionAlignment::default().status.lock().unwrap().clone();
            return quats.clone();
        }

        // Debug counters
        let mut num_used: usize = 0;
        let mut num_valid: usize = 0; // Frames that would be used with just 50ms window
        
        let mut out = TimeQuat::new();
        // Iterate timestamps, use close good quality motion direction, else use original orientation
        for (ts, quat) in quats.iter() {
            // Get motion direction within window, with fallback to closest good quality motion direction
            let motion_data_opt = self.find_closest_good_motion_data(compute_params, *ts, Self::FALLBACK_TIME_RADIUS_US);
            let new_quat = if let Some((motion_dir, time_dist)) = motion_data_opt {
                if time_dist <= Self::VALID_TIME_RADIUS_US { num_valid += 1; }
                num_used += 1;
                Self::compute_aligned_quaternion(quat, motion_dir)
            } else { None };
            out.insert(*ts, new_quat.unwrap_or(*quat));
        }

        // Store status for UI
        *self.status.lock().unwrap() = Self::build_status_messages(quats.len(), num_used, num_valid);

        out
    }
}
