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

    fn interpolate_at_time(&self, t_s: f64) -> Option<TrackPoint> {
        let n = self.len();
        if n == 0 {
            return None;
        }
        
        // Use same binary search logic as linear_interpolate_latlon
        let mut lo = 0usize;
        let mut hi = n - 1;
        
        if t_s <= self.time[0] {
            // Before first point - return first point
            lo = 0; hi = 0;
        } else if t_s >= self.time[n - 1] {
            // After last point - return last point
            lo = n - 1; hi = n - 1;
        } else {
            // Binary search to find the two points
            while hi - lo > 1 {
                let mid = (lo + hi) / 2;
                if self.time[mid] <= t_s {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
        }

        if lo == hi {
            return Some(TrackPoint {
                time: t_s,
                lat: self.lat[lo],
                lon: self.lon[lo],
                altitude: self.altitude[lo],
            });
        }
        
        // Interpolate between lo and hi
        let t0 = self.time[lo];
        let t1 = self.time[hi];
        let frac = ((t_s - t0) / (t1 - t0)).clamp(0.0, 1.0);
        
        let lat_interp = self.lat[lo] + (self.lat[hi] - self.lat[lo]) * frac;
        let lon_interp = self.lon[lo] + (self.lon[hi] - self.lon[lo]) * frac;
        let alt_interp = self.altitude[lo] + (self.altitude[hi] - self.altitude[lo]) * frac;
        
        Some(TrackPoint {
            time: t_s,
            lat: lat_interp,
            lon: lon_interp,
            altitude: alt_interp,
        })
    }

    /// Crop the track to a specific time range with optional padding.
    /// 
    /// # Arguments
    /// * `start_time` - Start time in seconds (relative to video creation time)
    /// * `duration` - Duration in seconds
    /// * `pad_start` - If true, add a point at start_time when the track starts after start_time
    /// * `pad_end` - If true, add a point at end_time when the track ends before end_time
    /// 
    /// # Returns
    /// A new `GpsTrack` with cropped data. Boundary points are always interpolated.
    pub fn fit_to_time_range(&self, start_time: f64, duration: f64, pad_start: bool, pad_end: bool) -> Self {
        // Early return for empty track
        if self.is_empty() {
            return Self {
                time: Vec::new(),
                lat: Vec::new(),
                lon: Vec::new(),
                altitude: Vec::new(),
            };
        }
        
        let end_time = start_time + duration;
        let n = self.len();
        
        // Determine boundary requirements by checking original track boundaries
        let track_starts_before = self.time[0] < start_time;
        let track_ends_after = self.time[n - 1] > end_time;
        let needs_start_point = track_starts_before || pad_start;
        let needs_end_point = track_ends_after || pad_end;
        
        // Filter points within the time range
        let filtered_indices: Vec<usize> = (0..self.len())
            .filter(|&i| self.time[i] >= start_time && self.time[i] <= end_time)
            .collect();
        
        // Calculate capacity
        let capacity = filtered_indices.len() + needs_start_point as usize + needs_end_point as usize;
        let mut time = Vec::with_capacity(capacity);
        let mut lat = Vec::with_capacity(capacity);
        let mut lon = Vec::with_capacity(capacity);
        let mut altitude = Vec::with_capacity(capacity);
        
        // Add start boundary point if needed (interpolated)
        if needs_start_point {
            if let Some(point) = self.interpolate_at_time(start_time) {
                time.push(point.time);
                lat.push(point.lat);
                lon.push(point.lon);
                altitude.push(point.altitude);
            }
        }
        
        // Add filtered points
        for &idx in &filtered_indices {
            time.push(self.time[idx]);
            lat.push(self.lat[idx]);
            lon.push(self.lon[idx]);
            altitude.push(self.altitude[idx]);
        }
        
        // Add end boundary point if needed (interpolated)
        if needs_end_point {
            if let Some(point) = self.interpolate_at_time(end_time) {
                time.push(point.time);
                lat.push(point.lat);
                lon.push(point.lon);
                altitude.push(point.altitude);
            }
        }
        
        Self { time, lat, lon, altitude }
    }
}
