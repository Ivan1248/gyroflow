// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2021-2022 Adrian <adrian.eddy at gmail>

use super::*;

#[derive(Clone)]
pub struct Simple {
    pub time_constant: f64,
    pub reverse_first: bool,
}

impl Default for Simple {
    fn default() -> Self { 
        Self {
            time_constant: 0.25,
            reverse_first: false,
        } 
    }
}

impl SmoothingAlgorithm for Simple {
    fn get_name(&self) -> String { "Simple 3D".to_owned() }

    fn set_parameter(&mut self, name: &str, val: f64) {
        match name {
            "time_constant" => self.time_constant = val,
            "reverse_first" => self.reverse_first = val > 0.1,
            _ => log::error!("Invalid parameter name: {}", name)
        }
    }
    
    fn get_parameter(&self, name: &str) -> f64 {
        match name {
            "time_constant" => self.time_constant,
            "reverse_first" => if self.reverse_first { 1.0 } else { 0.0 },
            _ => 0.0
        }
    }

    fn get_parameters_json(&self) -> serde_json::Value {
        serde_json::json!([
            {
                "name": "time_constant",
                "description": "Smoothness",
                "type": "SliderWithField",
                "from": 0.01,
                "to": 10.0,
                "value": self.time_constant,
                "default": 0.25,
                "unit": "s",
            },
            {
                "name": "reverse_first",
                "description": "Reverse-first smoothing",
                "type": "CheckBox",
                "default": self.reverse_first,
                "value": if self.reverse_first { 1.0 } else { 0.0 },
            },
        ])
    }
    
    fn get_status_json(&self) -> serde_json::Value {
        serde_json::json!([])
    }

    fn get_checksum(&self) -> u64 {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        hasher.write_u64(self.time_constant.to_bits());
        hasher.write_u8(self.reverse_first as u8);
        hasher.finish()
    }

    fn smooth(&self, quats: &TimeQuat, duration_ms: f64, _compute_params: &ComputeParams) -> TimeQuat {
        if quats.is_empty() || duration_ms <= 0.0 { 
            return quats.clone(); 
        }

        // Calculate constant alpha from time constant
        let sample_rate: f64 = quats.len() as f64 / (duration_ms / 1000.0);
        let alpha = if self.time_constant > 0.0 {
            1.0 - (-(1.0 / sample_rate) / self.time_constant).exp()
        } else {
            1.0
        };

        // Helper function to perform a smoothing pass
        let smooth_pass = |input_quats: &TimeQuat, forward: bool| -> TimeQuat {
            if forward {
                let mut q = *input_quats.iter().next().unwrap().1;
                input_quats.iter().map(|x| {
                    q = q.slerp(x.1, alpha);
                    (*x.0, q)
                }).collect()
            } else {
                let mut q = *input_quats.iter().next_back().unwrap().1;
                input_quats.iter().rev().map(|x| {
                    q = q.slerp(x.1, alpha);
                    (*x.0, q)
                }).collect()
            }
        };

        let smoothed1 = smooth_pass(quats, !self.reverse_first);
        smooth_pass(&smoothed1, self.reverse_first)
    }
}

