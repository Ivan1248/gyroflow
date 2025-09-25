// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2025

use super::*;
use std::sync::Mutex;

pub struct MotionDirection {
    pub time_constant: f64,
    pub min_inlier_ratio: f64,
    pub max_epi_err: f64,
    pub motion_window_ms: f64,
    pub smoothing_enabled: bool,
    status: Mutex<Vec<serde_json::Value>>,
}

impl Default for MotionDirection {
    fn default() -> Self {
        Self {
            time_constant: 0.25,
            min_inlier_ratio: 0.2,
            max_epi_err: 2.0,
            motion_window_ms: 50.0,
            smoothing_enabled: true,
            status: Mutex::new(Vec::new()),
        }
    }
}

impl Clone for MotionDirection {
    fn clone(&self) -> Self {
        let status = self.status.lock().unwrap().clone();
        Self {
            time_constant: self.time_constant,
            min_inlier_ratio: self.min_inlier_ratio,
            max_epi_err: self.max_epi_err,
            motion_window_ms: self.motion_window_ms,
            smoothing_enabled: self.smoothing_enabled,
            status: Mutex::new(status)
        }
    }
}

impl SmoothingAlgorithm for MotionDirection {
    fn get_name(&self) -> String { "Motion direction".to_owned() }

    fn set_parameter(&mut self, name: &str, val: f64) {
        match name {
            "time_constant" => self.time_constant = val,
            "min_inlier_ratio" => self.min_inlier_ratio = val,
            "max_epi_err" => self.max_epi_err = val,
            "motion_window_ms" => self.motion_window_ms = val,
            "smoothing_enabled" => self.smoothing_enabled = val >= 0.5,
            _ => log::error!("Invalid parameter name: {}", name)
        }
    }
    fn get_parameter(&self, name: &str) -> f64 {
        match name {
            "time_constant" => self.time_constant,
            "min_inlier_ratio" => self.min_inlier_ratio,
            "max_epi_err" => self.max_epi_err,
            "motion_window_ms" => self.motion_window_ms,
            "smoothing_enabled" => if self.smoothing_enabled { 1.0 } else { 0.0 },
            _ => 0.0
        }
    }

    fn get_parameters_json(&self) -> serde_json::Value {
        serde_json::json!([
        {
            "name": "smoothing_enabled",
            "description": "Enable smoothing (blend towards target)",
            "type": "CheckBox",
            "value": self.smoothing_enabled,
            "default": true,
            "custom_qml": "id: mdSmoothingEnabled"
        },
        {
            "name": "time_constant",
            "description": "Smoothness",
            "type": "SliderWithField",
            "from": 0.01,
            "to": 10.0,
            "value": self.time_constant,
            "default": 0.25,
            "unit": "s",
            "keyframe": "SmoothingParamTimeConstant",
            "custom_qml": "enabled: mdSmoothingEnabled.checked"
        },{
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
            "name": "motion_window_ms",
            "description": "Motion sampling window",
            "type": "SliderWithField",
            "from": 5.0,
            "to": 500.0,
            "value": self.motion_window_ms,
            "default": 50.0,
            "precision": 0,
            "unit": "ms"
        }])
    }
    fn get_status_json(&self) -> serde_json::Value { serde_json::Value::Array(self.status.lock().unwrap().clone()) }
    fn get_checksum(&self) -> u64 {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        hasher.write_u64(self.time_constant.to_bits());
        hasher.write_u64(self.min_inlier_ratio.to_bits());
        hasher.write_u64(self.max_epi_err.to_bits());
        hasher.write_u64(self.motion_window_ms.to_bits());
        hasher.write_u64(if self.smoothing_enabled { 1u64 } else { 0u64 });
        hasher.finish()
    }

    fn smooth(&self, quats: &TimeQuat, duration_ms: f64, compute_params: &ComputeParams) -> TimeQuat {
        //if quats.is_empty() || duration_ms <= 0.0 { 
        //    return quats.clone(); 
        //}
        // Check if pose estimator has any motion data
        let _ = {
            if let Some(sync_results) = compute_params.pose_estimator.sync_results.try_read() {
                sync_results.values().any(|fr| fr.transl_dir.is_some())
            } else {
                *self.status.lock().unwrap() = vec![serde_json::json!({
                    "type": "Label",
                    "text": "Motion direction: no motion samples available. Check pose estimation settings or re-run synchronization."
                })];
                return quats.clone();
            }
        };

        let sample_rate: f64 = quats.len() as f64 / (duration_ms / 1000.0);
        let get_alpha = |time_constant: f64| 1.0 - (-(1.0 / sample_rate) / time_constant).exp();
        let alpha = if self.smoothing_enabled {
            if self.time_constant > 0.0 { get_alpha(self.time_constant) } else { 1.0 }
        } else { 1.0 };

        let mut out = TimeQuat::new();


        // Debug counters
        let mut frames_total: usize = 0;
        let mut targets_used: usize = 0;
        let mut skipped_no_sample: usize = 0;
        let mut skipped_no_gyro: usize = 0;
        let mut skipped_degenerate: usize = 0;

        // Iterate timestamps, blend towards motion direction look-at if quality OK, else keep gyro orientation
        for (ts, quat) in quats.iter() {
            frames_total += 1;
            
            // Get motion direction within window
            let window_us: i64 = (self.motion_window_ms * 1000.0) as i64;
            let motion_dir_opt: Option<nalgebra::Vector3<f64>> = if let Some((tdir, qual)) = compute_params.pose_estimator.get_transl_dir_near(*ts, window_us, false) {
                if qual.meets_thresholds(self.min_inlier_ratio, self.max_epi_err) {
                    // Flip sign to get motion direction
                    Some(-nalgebra::Vector3::new(tdir[0], tdir[1], tdir[2]))
                } else { None }
            } else { skipped_no_sample += 1; None };

            let target = if let Some(motion_dir) = motion_dir_opt {
                // Rotate current camera orientation so its local Z axis aligns with motion direction (-tdir is in camera coords)
                let n = motion_dir.norm();
                if n > 1e-6 {
                    let ld = motion_dir / n;
                    // Mirror X to match quaternion-to-image convention (yaw sign)
                    let look_direction = nalgebra::Vector3::<f64>::new(-ld.x, ld.y, ld.z);
                    let z_axis = nalgebra::Vector3::<f64>::new(0.0, 0.0, 1.0);
                    let delta = if let Some(q) = nalgebra::UnitQuaternion::rotation_between(&z_axis, &look_direction) {
                        q
                    } else {
                        // 180° rotation around any axis orthogonal to Z; choose X
                        nalgebra::UnitQuaternion::from_axis_angle(&nalgebra::Vector3::<f64>::x_axis(), std::f64::consts::PI)
                    };
                    Some(*quat * delta)
                } else { skipped_degenerate += 1; None }
            } else { None };

            let new_q = if let Some(target_q) = target { 
                targets_used += 1;
                quat.slerp(&target_q, alpha) 
            } else { *quat };
            out.insert(*ts, new_q);
        }

        // Store status for UI
        let mut msgs: Vec<serde_json::Value> = Vec::new();
        if frames_total > 0 {
            let used_percent = (targets_used as f64 * 100.0 / frames_total as f64).round();
            msgs.push(serde_json::json!({
                "type": "Label",
                "text": format!("Motion direction: used {}% of frames ({} / {})", used_percent as i64, targets_used, frames_total)
            }));
            if targets_used == 0 {
                msgs.push(serde_json::json!({
                    "type": "Label",
                    "text": "No valid motion directions found within the window. Consider increasing the window or adjusting pose estimation method."
                }));
            }
            if skipped_no_sample > 0 {
                msgs.push(serde_json::json!({
                    "type": "Label",
                    "text": format!("No motion sample near timestamp for {} frames. Increase motion window.", skipped_no_sample)
                }));
            }
            if skipped_no_gyro > 0 {
                msgs.push(serde_json::json!({
                    "type": "Label",
                    "text": format!("No gyro orientation near timestamp for {} frames.", skipped_no_gyro)
                }));
            }
            if skipped_degenerate > 0 {
                msgs.push(serde_json::json!({
                    "type": "Label",
                    "text": format!("Degenerate motion direction for {} frames.", skipped_degenerate)
                }));
            }
        }
        *self.status.lock().unwrap() = msgs;

        out
    }
}


