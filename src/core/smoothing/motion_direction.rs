// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2025

use super::*;
use std::sync::Mutex;

pub struct MotionDirectionAlignment {
    pub min_inlier_ratio: f64,
    pub max_epi_err: f64,
    pub flip_backward_dir: bool,
    status: Mutex<Vec<serde_json::Value>>,
}

impl Default for MotionDirectionAlignment {
    fn default() -> Self {
        Self {
            min_inlier_ratio: 0.2,
            max_epi_err: 2.0,
            // TODO: flip_backward_dir=true is more stable when stationary, but looks in the reverse motion direction if going backward
            flip_backward_dir: false,
            status: Mutex::new(vec![serde_json::json!({
                "type": "Label",
                "text": "Cannot apply motion direction alignment: vision-based motion direction data has not been computed."
            })]),
        }
    }
}

impl Clone for MotionDirectionAlignment {
    fn clone(&self) -> Self {
        let status = self.status.lock().unwrap().clone();
        Self {
            min_inlier_ratio: self.min_inlier_ratio,
            max_epi_err: self.max_epi_err,
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
            "max_epi_err" => self.max_epi_err = val,
            "flip_backward_dir" => self.flip_backward_dir = val > 0.1,
            _ => log::error!("Invalid parameter name: {}", name)
        }
    }
    pub fn get_parameter(&self, name: &str) -> f64 {
        match name {
            "min_inlier_ratio" => self.min_inlier_ratio,
            "max_epi_err" => self.max_epi_err,
            "flip_backward_dir" => if self.flip_backward_dir { 1.0 } else { 0.0 },
            _ => 0.0
        }
    }

    pub fn get_parameters_json(&self) -> serde_json::Value {
        serde_json::json!([
        {
            "name": "min_inlier_ratio",
            "description": "Min inlier ratio",
            "type": "SliderWithField",
            "from": 0.0,
            "to": 1.0,
            "value": self.min_inlier_ratio,
            "default": 0.2,
            "precision": 3,
            "unit": ""
        },{
            "name": "max_epi_err",
            "description": "Max epipolar error",
            "type": "SliderWithField",
            "from": 0.0,
            "to": 5.0,
            "value": self.max_epi_err,
            "default": 2.0,
            "precision": 3,
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
        hasher.write_u64(self.min_inlier_ratio.to_bits());
        hasher.write_u64(self.max_epi_err.to_bits());
        hasher.write_u64(self.flip_backward_dir as u64);
        hasher.finish()
    }

    pub fn apply(&self, quats: &TimeQuat, duration_ms: f64, compute_params: &ComputeParams) -> TimeQuat {
        //if quats.is_empty() || duration_ms <= 0.0 { 
        //    return quats.clone(); 
        //}
        // Check if pose estimator has any motion data
        if compute_params.pose_estimator.sync_results.try_read().map_or(true, |sr| !sr.values().any(|fr| fr.transl_dir.is_some())) {
            *self.status.lock().unwrap() = MotionDirectionAlignment::default().status.lock().unwrap().clone();
            return quats.clone();
        }

        let mut out = TimeQuat::new();

        // Debug counters
        let mut num_frames: usize = 0;
        let mut num_used: usize = 0;
        let mut num_valid: usize = 0; // Frames that would be used with just 50ms window

        // Iterate timestamps, blend towards closest good quality motion direction, else keep gyro orientation
        for (ts, quat) in quats.iter() {
            num_frames += 1;

            // Get motion direction within window, with fallback to closest good quality motion direction
            let motion_data_opt = self.find_closest_good_motion_data(compute_params, *ts, Self::FALLBACK_TIME_RADIUS_US);

            let target = if let Some((motion_dir, time_dist)) = motion_data_opt {
                // Track frames that used data from within the 50ms window
                if time_dist <= Self::VALID_TIME_RADIUS_US {
                    num_valid += 1;
                }
                // Rotate current camera orientation so its local Z axis aligns with motion direction (-tdir is in camera coords)
                let n = motion_dir.norm();
                if n > Self::DEGENERATE_NORM_EPSILON {
                    let ld = motion_dir / n;
                    // X = right, Y = up, Z = forward (left-handed)
                    // For some reason, X has to be flipped to match the quaternion-to-image convention
                    let look_direction = nalgebra::Vector3::<f64>::new(-ld.x, ld.y, ld.z);
                    // Derive world-space up from current orientation to preserve roll.
                    let dir_world = *quat * look_direction;
                    let up_world = *quat * nalgebra::Vector3::<f64>::new(0.0, 1.0, 0.0);
                    let target_q = nalgebra::UnitQuaternion::face_towards(&dir_world, &up_world);
                    Some(target_q)
                } else {
                    None
                }
            } else {
                None
            };

            let new_q = if let Some(target_q) = target { 
                num_used += 1;
                quat.slerp(&target_q, 1.0) 
            } else { *quat };
            out.insert(*ts, new_q);
        }

        // Store status for UI
        let mut msgs: Vec<serde_json::Value> = Vec::new();
        if num_frames > 0 {
            let used_percent = (num_used as f64 * 100.0 / num_frames as f64).round();
            let valid_percent = (num_valid as f64 * 100.0 / num_frames as f64).round();
            msgs.push(serde_json::json!({
                "type": "Label",
                "text": format!("Motion direction: {:.0}% of frames have good quality motion data ({:.0}% within {} ms)", valid_percent, used_percent, Self::FALLBACK_TIME_RADIUS_US / 1000)
            }));
            if num_used == 0 {
                msgs.push(serde_json::json!({
                    "type": "Label",
                    "text": format!("No valid motion directions in video. Consider changing settings.")
                }));
            }
        }
        *self.status.lock().unwrap() = msgs;

        out
    }
}
