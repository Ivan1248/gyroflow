// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2025

use super::ffmpeg_processor::FFmpegError;
use ffmpeg_next::{ frame, format::Pixel };
use std::f64::consts::PI;
use gyroflow_core::lens_profile::LensProfile;

/// Basic CPU-based dual-fisheye to equirectangular stitcher for validation
pub struct CpuStitcher {
    pub output_width: u32,
    pub output_height: u32,
    pub fov: f64,            // Shared full FOV (deg)
    // Global (rig) orientation
    pub rig_yaw: f64,
    pub rig_pitch: f64,
    pub rig_roll: f64,
    // Back camera relative to front (front is the rig itself)
    pub back_delta_yaw: f64,   // Added on top of 180°
    pub back_delta_pitch: f64,
    pub back_delta_roll: f64,
    pub seam_width_deg: f64,   // Overlap blend width in degrees
    // Lens calibration data
    pub front_lens_profile: Option<LensProfile>,
    pub back_lens_profile: Option<LensProfile>,
}

impl CpuStitcher {
    pub fn new(output_width: u32, output_height: u32) -> Self {
        Self {
            output_width,
            output_height,
            // Default values - will be overridden by camera-specific settings
            fov: 200.0,
            rig_yaw: 0.0,
            rig_pitch: 0.0,
            rig_roll: 0.0,
            back_delta_yaw: 0.0,   // Effective back yaw = rig_yaw + 180° + delta
            back_delta_pitch: 0.0,
            back_delta_roll: 0.0,
            seam_width_deg: 20.0,
            front_lens_profile: None,
            back_lens_profile: None,
        }
    }

    pub fn configure_for_lens_profile(&mut self, lens_profile: &LensProfile) {
        // Always have a lens profile (even if default), check for calibration data
        self.front_lens_profile = Some(lens_profile.clone());
        self.back_lens_profile = Some(lens_profile.clone());

        if !lens_profile.fisheye_params.camera_matrix.is_empty() && lens_profile.fisheye_params.camera_matrix.len() >= 2 {
            let fx = lens_profile.fisheye_params.camera_matrix[0][0];
            let fy = lens_profile.fisheye_params.camera_matrix[1][1];
            let cx = lens_profile.fisheye_params.camera_matrix[0][2];
            let cy = lens_profile.fisheye_params.camera_matrix[1][2];
            let sensor_width = lens_profile.calib_dimension.w as f64;
            let sensor_height = lens_profile.calib_dimension.h as f64;

            // Use the corrected camera matrix from telemetry parser (cx_fix already applied)
            // For fisheye lenses, the camera matrix focal lengths are for the undistorted view
            // We need to estimate the raw fisheye FOV - Insta360 fisheye lenses typically cover ~150-200°
            if lens_profile.camera_brand.to_lowercase() == "insta360" {
                // Use model-specific FOV estimates based on known Insta360 specifications
                match lens_profile.camera_model.to_lowercase().as_str() {
                    "x4" => {
                        self.fov = 192.5; // X4 has ~200° fisheye FOV per lens
                    }
                    "x3" => {
                        self.fov = 190.0;
                    }
                    "one x2" | "one x3" | "oner" | "oners" => {
                        self.fov = 180.0;
                    }
                    _ => {
                        self.fov = 190.0; // Default Insta360 fisheye FOV
                    }
                }

                // Log the actual camera matrix values for debugging
                ::log::debug!("[cpu_stitcher] Insta360 camera matrix: fx={:.1}, fy={:.1}, cx={:.1}, cy={:.1}, sensor={}x{}",
                             fx, fy, cx, cy, sensor_width, sensor_height);
                ::log::info!("[cpu_stitcher] Using Insta360 {} fisheye FOV: {:.1}° (telemetry-corrected)",
                            lens_profile.camera_model, self.fov);
            } else {
                // For non-fisheye cameras, use the pinhole calculation
                let fov_x = 2.0 * ((sensor_width / (2.0 * fx)).atan()).to_degrees();
                let fov_y = 2.0 * ((sensor_height / (2.0 * fy)).atan()).to_degrees();
                self.fov = fov_x.max(fov_y);
                ::log::info!("[cpu_stitcher] Using pinhole FOV calculation: {:.1}°", self.fov);
            }

            // Adjust seam width based on calibration quality
            self.seam_width_deg = if lens_profile.official { 25.0 } else { 20.0 };

            // Configure camera orientations based on lens profile brand
            if lens_profile.camera_brand.to_lowercase() == "insta360" {
                // Standard 180° yaw separation for Insta360 360° cameras
                self.back_delta_yaw = 0.0;
                self.back_delta_roll = 0.0; // Try without roll rotation first
            } else {
                // Default orientations for other cameras
                self.back_delta_yaw = 0.0;
                self.back_delta_roll = 0.0;
            }

            ::log::info!("[cpu_stitcher] Lens profile from {} ({} {}): FOV={:.1}° (fx={:.1}, fy={:.1}, cx={:.1}, cy={:.1}, sensor={}x{})",
                        lens_profile.calibrated_by, lens_profile.camera_brand, lens_profile.camera_model,
                        self.fov, fx, fy, cx, cy, sensor_width, sensor_height);
        } else {
            // Lens profile exists but no camera matrix - use brand-specific defaults
            if lens_profile.camera_brand.to_lowercase() == "insta360" {
                // Conservative defaults for Insta360 when no calibration data
                self.fov = 200.0;
                self.seam_width_deg = 25.0;
                self.back_delta_yaw = 0.0;
                self.back_delta_roll = 180.0;
                ::log::info!("[cpu_stitcher] Insta360 lens profile without calibration, using defaults: FOV={}°", self.fov);
            } else {
                // Generic defaults for other cameras
                self.fov = 180.0;
                self.seam_width_deg = 20.0;
                self.back_delta_yaw = 0.0;
                self.back_delta_roll = 0.0;
                ::log::info!("[cpu_stitcher] Generic lens profile without calibration: FOV={}°", self.fov);
            }
        }
    }

    /// Stitch two fisheye frames into an equirectangular output
    pub fn stitch(&self, front_frame: &frame::Video, back_frame: &frame::Video) -> Result<frame::Video, FFmpegError> {
        if front_frame.width() != back_frame.width() || front_frame.height() != back_frame.height() {
            return Err(FFmpegError::InternalError(ffmpeg_next::Error::InvalidData));
        }

        let input_width = front_frame.width() as usize;
        let input_height = front_frame.height() as usize;
        let output_width = self.output_width as usize;
        let output_height = self.output_height as usize;

        // Create output frame
        let mut output_frame = frame::Video::new(Pixel::RGBA, self.output_width, self.output_height);

        // Get frame data and strides
        let front_data = front_frame.data(0);
        let back_data = back_frame.data(0);
        let front_stride = front_frame.stride(0);
        let back_stride = back_frame.stride(0);
        let output_stride = output_frame.stride(0);

        let output_data = output_frame.data_mut(0);

        let mut front_pixels = 0u64;
        let mut back_pixels = 0u64;

        // For each pixel in the output equirectangular image
        for y in 0..output_height {
            for x in 0..output_width {
                // Convert pixel coordinates to spherical coordinates
                let theta = (x as f64 / output_width as f64) * 2.0 * PI - PI; // longitude: -π to π
                let phi = (y as f64 / output_height as f64) * PI - PI/2.0;   // latitude: -π/2 to π/2

                // Compute samples from both cameras using equidistant fisheye mapping
                // Effective orientations
                let fy = (self.rig_yaw).to_radians();
                let fp = (self.rig_pitch).to_radians();
                let fr = (self.rig_roll).to_radians();
                let by = (self.rig_yaw + 180.0 + self.back_delta_yaw).to_radians();
                let bp = (self.rig_pitch + self.back_delta_pitch).to_radians();
                let br = (self.rig_roll + self.back_delta_roll).to_radians();

                let (front_col_opt, front_theta_cam) = self.sample_fisheye_equidistant(
                    front_data, input_width, input_height, front_stride,
                    theta, phi,
                    fy, fp, fr,
                    self.fov.to_radians(),
                    false, // front not flipped
                );
                let (back_col_opt, back_theta_cam) = self.sample_fisheye_equidistant(
                    back_data, input_width, input_height, back_stride,
                    theta, phi,
                    by, bp, br,
                    self.fov.to_radians(),
                    false, // back flipped horizontally
                );

                // Determine which camera(s) can see this direction and blend if in overlap
                let half_fov = self.fov.to_radians() * 0.5;

                let color = match (front_col_opt, back_col_opt) {
                    (Some(fc), Some(bc)) => {
                        // Both cameras can see this direction - blend based on overlap region
                        front_pixels += 1;
                        back_pixels += 1;

                        // Define overlap boundaries for cameras at 180° apart
                        // Front camera covers [-90°, +90°], back camera covers [+90°, +270°]
                        // Overlap occurs around ±90° boundaries
                        let boundary1 = std::f64::consts::FRAC_PI_2;  // +90°
                        let boundary2 = 3.0 * std::f64::consts::FRAC_PI_2;  // +270° (same as -90°)

                        // Find minimum angular distance to any boundary
                        let dist_to_boundary1 = (theta - boundary1).abs();
                        let dist_to_boundary2 = (theta - boundary2).abs().min((theta - boundary2 + 2.0 * std::f64::consts::PI).abs());
                        let min_dist_to_boundary = dist_to_boundary1.min(dist_to_boundary2);

                        // Define blend width in radians (20° overlap region)
                        let blend_width = (20.0_f64).to_radians();

                        if min_dist_to_boundary <= blend_width / 2.0 {
                            // In blend zone - use the camera with better view quality
                            // Quality is better when theta_cam is smaller (closer to camera center)
                            if front_theta_cam <= back_theta_cam {
                                fc
                            } else {
                                bc
                            }
                        } else {
                            // Outside blend zone - use the camera with better view quality
                            // Quality is better when theta_cam is smaller (closer to camera center)
                            if front_theta_cam <= back_theta_cam {
                                fc
                            } else {
                                bc
                            }
                        }
                    },
                    (Some(fc), None) => {
                        // Only front camera can see this
                        front_pixels += 1;
                        fc
                    },
                    (None, Some(bc)) => {
                        // Only back camera can see this
                        back_pixels += 1;
                        bc
                    },
                    (None, None) => {
                        // Neither camera can see this direction
                        [0, 0, 0, 255]
                    }
                };

                // Write to output
                let out_idx = y * output_stride + x * 4;
                if out_idx + 3 < output_data.len() {
                    output_data[out_idx..out_idx+4].copy_from_slice(&color);
                }
            }
        }

        Ok(output_frame)
    }

    /// Sample a color from a fisheye frame using equidistant model and proper spherical mapping.
    /// Returns (color_if_visible, theta_cam)
    fn sample_fisheye_equidistant(&self, data: &[u8], width: usize, height: usize, stride: usize,
                     theta: f64, phi: f64, yaw: f64, pitch: f64, roll: f64, fov_full: f64, flip_horizontal: bool) -> (Option<[u8; 4]>, f64) {
        // Build 3D ray from equirect (longitude=theta, latitude=phi)
        let vx = phi.cos() * theta.cos();
        let vy = phi.sin();
        let vz = phi.cos() * theta.sin();

        // Rotate world ray into camera coordinates by inverse camera orientation
        let (sx, sy, sz) = Self::rotate_ypr(vx, vy, vz, -yaw, -pitch, -roll);

        // Angle from camera forward axis (+X)
        let dot = sx.max(-1.0).min(1.0);
        let theta_cam = dot.acos();
        let half_fov = fov_full * 0.5;
        if theta_cam > half_fov {
            return (None, theta_cam);
        }

        // Project to fisheye image using equidistant: r = k * theta
        // Project to image plane perpendicular to +X: use (Z,Y) as plane axes
        let px = sz;
        let py = sy;
        let norm = (px*px + py*py).sqrt();
        let (mut ux, mut uy) = if norm > 1e-12 { (px / norm, py / norm) } else { (0.0, 0.0) };
        if flip_horizontal {
            ux = -ux;
        }
        let r_norm = (theta_cam / half_fov).max(0.0).min(1.0);

        // Convert to pixel coordinates (assuming circular fisheye)
        let center_x = width as f64 / 2.0;
        let center_y = height as f64 / 2.0;
        let max_radius = (width.min(height) as f64 / 2.0) * 1.0; // Use full image for Insta360 wide FOV
        let pixel_x = center_x + ux * r_norm * max_radius;
        let pixel_y = center_y + uy * r_norm * max_radius;

        // Bilinear sample
        if pixel_x < 0.0 || pixel_x >= width as f64 || pixel_y < 0.0 || pixel_y >= height as f64 {
            return (None, theta_cam);
        }
        let x0 = pixel_x.floor() as usize;
        let y0 = pixel_y.floor() as usize;
        let x1 = (x0 + 1).min(width - 1);
        let y1 = (y0 + 1).min(height - 1);
        let wx = pixel_x - x0 as f64;
        let wy = pixel_y - y0 as f64;
        let idx00 = y0 * stride + x0 * 4;
        let idx01 = y0 * stride + x1 * 4;
        let idx10 = y1 * stride + x0 * 4;
        let idx11 = y1 * stride + x1 * 4;
        if idx11 + 3 >= data.len() { return (None, theta_cam); }
        let c00 = &data[idx00..idx00+4];
        let c01 = &data[idx01..idx01+4];
        let c10 = &data[idx10..idx10+4];
        let c11 = &data[idx11..idx11+4];
        let mut result = [0u8; 4];
        for i in 0..4 {
            let v00 = c00[i] as f64;
            let v01 = c01[i] as f64;
            let v10 = c10[i] as f64;
            let v11 = c11[i] as f64;
            let top = v00 * (1.0 - wx) + v01 * wx;
            let bottom = v10 * (1.0 - wx) + v11 * wx;
            result[i] = (top * (1.0 - wy) + bottom * wy).round() as u8;
        }
        result[3] = 255;
        (Some(result), theta_cam)
    }

    #[inline]
    fn rotate_ypr(x: f64, y: f64, z: f64, yaw: f64, pitch: f64, roll: f64) -> (f64, f64, f64) {
        // Yaw around Y axis (original working order)
        let cy = yaw.cos(); let sy = yaw.sin();
        let mut rx =  x * cy + z * sy;
        let mut ry =  y;
        let mut rz = -x * sy + z * cy;

        // Pitch around X axis
        let cp = pitch.cos(); let sp = pitch.sin();
        let r2x = rx;
        let r2y = ry * cp - rz * sp;
        let r2z = ry * sp + rz * cp;
        rx = r2x; ry = r2y; rz = r2z;

        // Roll around Z axis
        let cr = roll.cos(); let sr = roll.sin();
        let r3x = rx * cr - ry * sr;
        let r3y = rx * sr + ry * cr;
        let r3z = rz;
        (r3x, r3y, r3z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stitcher_creation() {
        let stitcher = CpuStitcher::new(4096, 2048);
        assert_eq!(stitcher.output_width, 4096);
        assert_eq!(stitcher.output_height, 2048);
    }
}
