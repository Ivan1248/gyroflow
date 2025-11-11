// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2025 Ivan <ivan.grubisic at gmail>

pub mod data;
pub mod io;
pub mod processing;
pub mod source;
pub mod sync;
pub mod sampling;

// Re-export commonly used types
pub use data::{TrackPoint, GpsTrack};
pub use io::{parse_gpx_from_str, parse_gpx_file, save_gpx_file, save_waypoints_with_timestamps_csv};
pub use processing::{
    haversine_distance_m,
    initial_bearing_deg,
    BearingConvention,
    latlon_to_enu,
    unwrap_angles_deg,
    resample_linear,
    linear_interpolate_latlon,
};
pub use source::{GpsSource, GPSSyncMode, GPSAnchor, TimeAlignment, PreparedGpsData};
pub use sampling::{GPSFrameSamplingSettings, GPSFrameSamplingMethod};
pub use sync::{
    synchronize_gps_gyro,
    sample_gyro_yaw_at_times,
    GpsSyncSettings,
    GpsSyncResult,
    GpsSyncMethod,
    DEFAULT_MAX_TIME_OFFSET_S,
    DEFAULT_SAMPLE_RATE,
};
