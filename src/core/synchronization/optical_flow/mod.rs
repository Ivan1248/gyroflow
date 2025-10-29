// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2022 Adrian <adrian.eddy at gmail>

use super::OpticalFlowPair;
use std::sync::Arc;

mod akaze;        pub use self::akaze::*;
mod opencv_dis;   pub use opencv_dis::*;
mod opencv_pyrlk; pub use opencv_pyrlk::*;

#[enum_delegate::register]
pub trait OpticalFlowTrait {
    fn size(&self) -> (u32, u32);
    fn features(&self) -> &Vec<(f32, f32)>;
    fn optical_flow_to(&self, to: &OpticalFlowMethod, patch_filter: Option<&PatchFilterFn>) -> OpticalFlowPair;
    fn cleanup(&mut self);
    fn can_cleanup(&self) -> bool;
}

#[enum_delegate::implement(OpticalFlowTrait)]
#[derive(Clone)]
pub enum OpticalFlowMethod {
    OFAkaze(OFAkaze),
    OFOpenCVPyrLK(OFOpenCVPyrLK),
    OFOpenCVDis(OFOpenCVDis),
}
impl OpticalFlowMethod {
    pub fn detect_features(method: u32, timestamp_us: i64, img: Arc<image::GrayImage>, width: u32, height: u32) -> Self {
        match method {
            0 => Self::OFAkaze(OFAkaze::detect_features(timestamp_us, img, width, height)),
            1 => Self::OFOpenCVPyrLK(OFOpenCVPyrLK::detect_features(timestamp_us, img, width, height)),
            2 => Self::OFOpenCVDis(OFOpenCVDis::detect_features(timestamp_us, img, width, height)),
            _ => { log::error!("Unknown OF method {method}", ); Self::OFAkaze(OFAkaze::detect_features(timestamp_us, img, width, height)) }
        }
    }
}

impl Default for OpticalFlowMethod {
    fn default() -> Self { Self::OFAkaze(OFAkaze::default()) }
}

// ---------------- Projection-aware filtering types ----------------
use std::sync::Arc as StdArc;

/// Function type for filtering optical flow patches based on projection validity
/// Parameters: (x, y, radius, img_w, img_h) -> is_valid
pub type PatchFilterFn = StdArc<dyn Fn(f32, f32, f32, f32, f32) -> bool + Send + Sync>;

/// Helper function to extract and validate camera parameters from a camera matrix
fn extract_camera_params(camera_matrix: &nalgebra::Matrix3<f64>) -> Option<(f32, f32, f32, f32)> {
    let fx = camera_matrix[(0, 0)] as f32;
    let fy = camera_matrix[(1, 1)] as f32;
    let cx = camera_matrix[(0, 2)] as f32;
    let cy = camera_matrix[(1, 2)] as f32;

    if !fx.is_finite() || !fy.is_finite() || fx == 0.0 || fy == 0.0 { return None; }
    if !cx.is_finite() || !cy.is_finite() { return None; }

    Some((fx, fy, cx, cy))
}


/// Helper function to create projection filter with common patch validation logic
fn create_patch_filter_with_validator<F>(point_validator: F) -> PatchFilterFn
where
    F: Fn(f32, f32, f32, f32) -> bool + Send + Sync + 'static,
{
    StdArc::new(move |x: f32, y: f32, radius: f32, img_w: f32, img_h: f32| -> bool {
        if img_w <= 0.0 || img_h <= 0.0 { return false; }
        // In-bounds check for the square patch
        if x - radius < 0.0 || x + radius >= img_w { return false; }
        if y - radius < 0.0 || y + radius >= img_h { return false; }

        // Check all four corners of the square patch
        let a = point_validator(x - radius, y - radius, img_w, img_h);
        let b = point_validator(x + radius, y - radius, img_w, img_h);
        let c = point_validator(x - radius, y + radius, img_w, img_h);
        let d = point_validator(x + radius, y + radius, img_w, img_h);
        a && b && c && d
    })
}

/// Construct a projection filter function from lens profile.
/// Returns None if lens doesn't require projection filtering
pub fn patch_filter_from_lens_profile(lens: &crate::lens_profile::LensProfile) -> Option<PatchFilterFn> {
    let radial_limit = lens.fisheye_params.radial_distortion_limit? as f32;

    // Center in normalized coords; prefer actual principal point if available
    let mut center = (0.5_f32, 0.5_f32);
    if lens.fisheye_params.camera_matrix.len() == 3 {
        let cx = lens.fisheye_params.camera_matrix[0][2] as f32 / lens.calib_dimension.w as f32;
        let cy = lens.fisheye_params.camera_matrix[1][2] as f32 / lens.calib_dimension.h as f32;
        if cx.is_finite() && cy.is_finite() { center = (cx, cy); }
    }

    Some(create_patch_filter_with_validator(move |px: f32, py: f32, img_w: f32, img_h: f32| -> bool {
        let nx = px / img_w; // normalize to [0,1]
        let ny = py / img_h;
        let dx = nx - center.0;
        let dy = ny - center.1;
        (dx * dx + dy * dy).sqrt() <= radial_limit
    }))
}

/// Construct a projection filter from a camera matrix and radial limit
pub fn patch_filter_from_camera(camera_matrix: &nalgebra::Matrix3<f64>, radial_limit: f32) -> Option<PatchFilterFn> {
    if !(radial_limit.is_finite()) || radial_limit <= 0.0 { return None; }

    let (fx, fy, cx, cy) = extract_camera_params(camera_matrix)?;

    Some(create_patch_filter_with_validator(move |px: f32, py: f32, _img_w: f32, _img_h: f32| -> bool {
        let dx = (px - cx) / fx;
        let dy = (py - cy) / fy;
        (dx * dx + dy * dy).sqrt() <= radial_limit
    }))
}

/// Construct a projection filter using the distortion model's undistortion domain
pub fn patch_filter_from_model(
    dm: &crate::stabilization::distortion_models::DistortionModel,
    camera_matrix: &nalgebra::Matrix3<f64>,
    distortion_coeffs: &[f64; 12],
    width: i32,
    height: i32,
) -> Option<PatchFilterFn> {
    if width <= 0 || height <= 0 { return None; }

    let (fx, fy, cx, cy) = extract_camera_params(camera_matrix)?;

    let k: [f32; 12] = distortion_coeffs.iter().map(|v| *v as f32).collect::<Vec<_>>().try_into().ok()?;
    let mut kp = crate::stabilization::KernelParams::default();
    kp.width = width;
    kp.height = height;
    kp.f = [fx, fy];
    kp.c = [cx, cy];
    kp.k = k;
    let dm_cloned = dm.clone();
    Some(create_patch_filter_with_validator(move |px: f32, py: f32, _img_w: f32, _img_h: f32| -> bool {
        let dx = (px - kp.c[0]) / kp.f[0];
        let dy = (py - kp.c[1]) / kp.f[1];
        dm_cloned.undistort_point((dx, dy), &kp).is_some()
    }))
}