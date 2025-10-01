// SPDX-License-Identifier: GPL-3.0-or-later

pub mod source;
pub mod synchronization;

use std::path::Path;
use std::fs;
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize, PartialEq)]
pub struct TrackPoint {
    pub time: f64,  // seconds since Unix epoch (UTC)
    pub lat: f64,
    pub lon: f64,
    pub altitude: f64,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq)]
pub struct GPSTrack {
    pub time: Vec<f64>,  // seconds since Unix epoch (UTC)
    pub lat: Vec<f64>,
    pub lon: Vec<f64>,
    pub altitude: Vec<f64>,
}

impl GPSTrack {
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

    /// Get speed values (computed on each access)
    pub fn speed(&self) -> Vec<f64> {
        let n = self.len();
        if n <= 1 { return vec![0.0; n]; }
        let mut out = vec![0.0; n];
        for i in 1..n {
            let dt = (self.time[i] - self.time[i-1]).max(1e-6);
            let d = haversine_distance_m(self.lat[i-1], self.lon[i-1], self.lat[i], self.lon[i]);
            out[i] = d / dt;
        }
        out
    }

    /// Get course values in degrees (computed on each access)
    pub fn course_deg(&self) -> Vec<f64> {
        let n = self.len();
        if n <= 1 { return vec![0.0; n]; }
        let mut out = vec![0.0; n];
        for i in 1..n {
            out[i] = initial_bearing_deg(self.lat[i-1], self.lon[i-1], self.lat[i], self.lon[i], BearingConvention::EastCCW);
        }
        out
    }

    /// Get distance values (computed on each access)
    pub fn distance(&self) -> Vec<f64> {
        let n = self.len();
        let mut out = vec![0.0; n];
        for i in 1..n {
            out[i] = out[i-1] + haversine_distance_m(self.lat[i-1], self.lon[i-1], self.lat[i], self.lon[i]);
        }
        out
    }
}

// ---------------- GPX IO ----------------

pub fn parse_gpx_from_str(s: &str) -> Result<GPSTrack, crate::GyroflowCoreError> {
    use quick_xml::events::Event;
    use quick_xml::Reader;
    use chrono::{DateTime, FixedOffset};

    let mut reader = Reader::from_str(s);
    reader.config_mut().trim_text(true);
    let mut buf = Vec::new();

    let mut lat = 0.0;
    let mut lon = 0.0;
    let mut ele: Option<f64> = None;
    let mut time_s: Option<f64> = None;

    let mut points: Vec<TrackPoint> = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Eof) => break,
            Ok(Event::Start(e)) | Ok(Event::Empty(e)) => {
                let name_vec = e.name().as_ref().to_vec();
                let name = name_vec.as_slice();
                if name.ends_with(b"trkpt") {
                    lat = 0.0; lon = 0.0; ele = None; time_s = None;
                    for a in e.attributes().flatten() {
                        if a.key.as_ref().ends_with(b"lat") { lat = std::str::from_utf8(&a.value).ok().and_then(|x| x.parse::<f64>().ok()).unwrap_or(0.0); }
                        if a.key.as_ref().ends_with(b"lon") { lon = std::str::from_utf8(&a.value).ok().and_then(|x| x.parse::<f64>().ok()).unwrap_or(0.0); }
                    }
                } else if name.ends_with(b"ele") {
                    if let Ok(Event::Text(t)) = reader.read_event_into(&mut buf) {
                        ele = t.unescape().ok().and_then(|x| x.parse::<f64>().ok());
                    }
                } else if name.ends_with(b"time") {
                    if let Ok(Event::Text(t)) = reader.read_event_into(&mut buf) {
                        let txt = t.unescape().unwrap_or_default().to_string();
                        // Accept Z or offset
                        if let Ok(dt) = DateTime::parse_from_rfc3339(&txt) {
                            time_s = Some(dt.timestamp() as f64 + (dt.timestamp_subsec_micros() as f64)/1e6);
                        } else if let Ok(dt) = DateTime::parse_from_str(&txt.replace("Z", "+00:00"), "%Y-%m-%dT%H:%M:%S%:z") {
                            let dt: DateTime<FixedOffset> = dt;
                            time_s = Some(dt.timestamp() as f64 + (dt.timestamp_subsec_micros() as f64)/1e6);
                        }
                    }
                }
            }
            Ok(Event::End(e)) => {
                let name_vec = e.name().as_ref().to_vec();
                let name = name_vec.as_slice();
                if name.ends_with(b"trkpt") {
                    if let (Some(ele), Some(ts)) = (ele, time_s) {
                        points.push(TrackPoint { time: ts, lat: lat, lon: lon, altitude: ele });
                    }
                }
            }
            Err(_) => break,
            _ => {}
        }
        buf.clear();
    }

    Ok(GPSTrack::from_track_points(points))
}

pub fn parse_gpx_file<P: AsRef<Path>>(path: P) -> Result<GPSTrack, crate::GyroflowCoreError> {
    let s = fs::read_to_string(path)?;
    parse_gpx_from_str(&s)
}

pub fn save_gpx_file<P: AsRef<Path>>(path: P, time_offset: f64, track: &GPSTrack) -> Result<(), crate::GyroflowCoreError> {
    use chrono::{TimeZone, Utc};
    use quick_xml::Writer;
    use quick_xml::events::{Event, BytesDecl, BytesStart};

    let mut w = Writer::new(Vec::new());
    w.write_event(Event::Decl(BytesDecl::new("1.0", Some("UTF-8"), None)))?;
    let mut gpx = BytesStart::new("gpx");
    gpx.push_attribute(("version", "1.1"));
    gpx.push_attribute(("creator", "gyroflow"));
    gpx.push_attribute(("xmlns", "http://www.topografix.com/GPX/1/1"));
    w.write_event(Event::Start(gpx.clone()))?;

    w.write_event(Event::Start(BytesStart::new("trk")))?;
    w.write_event(Event::Start(BytesStart::new("trkseg")))?;

    for p in track.to_track_points(time_offset).iter() {
        let mut trkpt = BytesStart::new("trkpt");
        trkpt.push_attribute(("lat", &*format!("{}", p.lat)));
        trkpt.push_attribute(("lon", &*format!("{}", p.lon)));
        w.write_event(Event::Start(trkpt))?;

        let ele = BytesStart::new("ele");
        w.write_event(Event::Start(ele.borrow()))?;
        w.write_event(Event::Text(quick_xml::events::BytesText::new(&format!("{}", p.altitude))))?;
        w.write_event(Event::End(ele.to_end()))?;

        let time = BytesStart::new("time");
        w.write_event(Event::Start(time.borrow()))?;
        let dt = Utc.timestamp_opt(p.time as i64, ((p.time.fract() * 1e9) as u32).min(999_999_999)).unwrap();
        w.write_event(Event::Text(quick_xml::events::BytesText::new(&dt.to_rfc3339_opts(chrono::SecondsFormat::Millis, true))))?;
        w.write_event(Event::End(time.to_end()))?;

        w.write_event(Event::End(BytesStart::new("trkpt").to_end()))?;
    }

    w.write_event(Event::End(BytesStart::new("trkseg").to_end()))?;
    w.write_event(Event::End(BytesStart::new("trk").to_end()))?;
    w.write_event(Event::End(gpx.to_end()))?;

    let data = w.into_inner();
    fs::write(path, data)?;
    Ok(())
}

// ---------------- Coordinates & derivations ----------------

const EARTH_RADIUS_M: f64 = 6_371_000.0;

pub fn haversine_distance_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let to_rad = |d: f64| d.to_radians();
    let dlat = to_rad(lat2 - lat1);
    let dlon = to_rad(lon2 - lon1);
    let a = (dlat / 2.0).sin().powi(2) + lat1.to_radians().cos() * lat2.to_radians().cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    EARTH_RADIUS_M * c
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, ::serde::Serialize, ::serde::Deserialize)]
pub enum BearingConvention {
    #[default]
    EastCCW,  // ENU, ROS-REP 103
    NorthCW,  // NED, azimuth
}

pub fn initial_bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64, convention: BearingConvention) -> f64 {
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let y = dlon.sin() * phi2.cos();
    let x = phi1.cos() * phi2.sin() - phi1.sin() * phi2.cos() * dlon.cos();
    let north_cw_bearing = (y.atan2(x).to_degrees() + 360.0) % 360.0;
    match convention {
        BearingConvention::EastCCW => (360.0 - north_cw_bearing + 90.0) % 360.0,
        BearingConvention::NorthCW => north_cw_bearing,
    }
}

pub fn latlon_to_enu(lat0: f64, lon0: f64, lat: f64, lon: f64) -> Vector3<f64> {
    let d_n = haversine_distance_m(lat0, lon0, lat, lon0) * if lat > lat0 { 1.0 } else { -1.0 };
    let d_e = haversine_distance_m(lat0, lon0, lat0, lon) * if lon > lon0 { 1.0 } else { -1.0 };
    Vector3::new(d_e, 0.0, d_n)
}

/// Unwrap circular angles (degrees) to be continuous over time by removing 360° jumps.
pub fn unwrap_angles_deg(data: &[f64]) -> Vec<f64> {
    if data.is_empty() { return vec![]; }
    let mut out = Vec::with_capacity(data.len());
    let mut prev = data[0];
    out.push(prev);
    for &v in &data[1..] {
        let mut d = v - prev;
        while d > 180.0 { d -= 360.0; }
        while d < -180.0 { d += 360.0; }
        let unwrapped = prev + d;
        out.push(unwrapped);
        prev = unwrapped;
    }
    out
}




