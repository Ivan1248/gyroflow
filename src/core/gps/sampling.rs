// SPDX-License-Identifier: GPL-3.0-or-later

pub const DEFAULT_DISTANCE_INTERVAL: f64 = 10.0;
pub const DEFAULT_DISTANCE_MIN_SPEED: f64 = 0.0;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct GPSFrameSamplingSettings {
    pub method: GPSFrameSamplingMethod,
}

impl Default for GPSFrameSamplingSettings {
    fn default() -> Self {
        Self { 
            method: GPSFrameSamplingMethod::DistanceInterval { 
                step_m: DEFAULT_DISTANCE_INTERVAL, 
                min_speed: DEFAULT_DISTANCE_MIN_SPEED 
            }
        }
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum GPSFrameSamplingMethod {
    DistanceInterval { step_m: f64, min_speed: f64 },
    // Anticipated method: map a list of GPS coordinates to closest timestamps
    CoordinatesList { coords: Vec<(f64, f64)> },
}


