// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2022 Ivan Grubišić <ivan.grubisic at gmail>

use std::path::Path;
use std::fs;
use super::data::{TrackPoint, GpsTrack};

pub fn parse_gpx_from_str(s: &str) -> Result<GpsTrack, crate::GyroflowCoreError> {
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

    Ok(GpsTrack::from_track_points(points))
}

pub fn parse_gpx_file<P: AsRef<Path>>(path: P) -> Result<GpsTrack, crate::GyroflowCoreError> {
    let s = fs::read_to_string(path)?;
    parse_gpx_from_str(&s)
}

pub fn save_gpx_file<P: AsRef<Path>>(path: P, time_offset: f64, track: &GpsTrack) -> Result<(), crate::GyroflowCoreError> {
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

