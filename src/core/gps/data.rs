// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2022 Ivan <ivan.grubisic at gmail>

use super::processing::{haversine_distance_m, initial_bearing_deg, BearingConvention};

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize, PartialEq)]
pub struct TrackPoint {
    pub time: f64,  // seconds since Unix epoch (UTC)
    pub lat: f64,
    pub lon: f64,
    pub altitude: f64,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq)]
pub struct GpsTrack {
    pub time: Vec<f64>,  // seconds since Unix epoch (UTC)
    pub lat: Vec<f64>,
    pub lon: Vec<f64>,
    pub altitude: Vec<f64>,
}

impl GpsTrack {
    pub fn len(&self) -> usize { self.time.len() }
    pub fn is_empty(&self) -> bool { self.len() == 0 }

    pub fn from_track_points(mut pts: Vec<TrackPoint>) -> Self {
        pts.sort_by(|a, b| a.time.partial_cmp(&b.time).unwrap_or(std::cmp::Ordering::Equal));
        let mut time = Vec::with_capacity(pts.len());
        let mut lat = Vec::with_capacity(pts.len());
        let mut lon = Vec::with_capacity(pts.len());
        let mut altitude = Vec::with_capacity(pts.len());
        for p in pts.iter() {
            time.push(p.time);  // Preserve absolute timestamps
            lat.push(p.lat);
            lon.push(p.lon);
            altitude.push(p.altitude);
        }
        Self { time, lat, lon, altitude }
    }

    pub fn get_start_time(&self) -> Option<f64> {
        self.time.first().copied()  // Returns absolute Unix epoch seconds
    }

    pub fn to_track_points(&self, time_offset: f64) -> Vec<TrackPoint> {
        (0..self.len()).map(|i| TrackPoint { 
            time: time_offset + self.time[i],
            lat: self.lat[i], 
            lon: self.lon[i], 
            altitude: self.altitude[i] 
        }).collect()
    }

    pub fn with_subtracted_time(&self, subtracted_time: f64) -> Self {
        Self { 
            time: self.time.iter().map(|t| t - subtracted_time).collect(), 
            lat: self.lat.clone(), 
            lon: self.lon.clone(), 
            altitude: self.altitude.clone(), 
        }
    }

    pub fn get_speed(&self) -> Vec<f64> {
        let n = self.len();
        if n <= 1 { return vec![0.0; n]; }
        let compute_speed = |a: usize, b: usize| {
            let d = haversine_distance_m(self.lat[a], self.lon[a], self.lat[b], self.lon[b]);
            d / (self.time[b] - self.time[a]).max(1e-6)
        };
        let mut out = vec![0.0; n];
        out[0] = compute_speed(0, 1);  // forward difference
        for i in 1..n-1 {
            out[i] = compute_speed(i-1, i+1);  // central difference
        }
        out[n-1] = compute_speed(n-2, n-1);  // backward difference
        out
    }

    pub fn get_course_deg(&self) -> Vec<f64> {
        // TODO: return with timestamps with points between original points for sharpness
        let n = self.len();
        if n <= 1 { return vec![0.0; n]; }
        let mut out = vec![0.0; n];
        out[0] = initial_bearing_deg(self.lat[0], self.lon[0], self.lat[1], self.lon[1], BearingConvention::EastCCW);
        for i in 1..n-1 {
            out[i] = initial_bearing_deg(self.lat[i-1], self.lon[i-1], self.lat[i+1], self.lon[i+1], BearingConvention::EastCCW);
        }
        out[n-1] = initial_bearing_deg(self.lat[n-2], self.lon[n-2], self.lat[n-1], self.lon[n-1], BearingConvention::EastCCW);
        out
    }

    pub fn get_distance(&self) -> Vec<f64> {
        let n = self.len();
        let mut out = vec![0.0; n];
        for i in 1..n {
            out[i] = out[i-1] + haversine_distance_m(self.lat[i-1], self.lon[i-1], self.lat[i], self.lon[i]);
        }
        out
    }
}

