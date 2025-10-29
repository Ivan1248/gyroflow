// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2022 Adrian <adrian.eddy at gmail>

use crate::stabilization::KernelParams;

#[derive(Default, Clone)]
pub struct Insta360 { }

impl Insta360 {
    // NOTE ABOUT VALIDITY AND POSE ESTIMATION:
    // - This CPU implementation returns `None` when the iterative solve fails to converge
    //   (see `converged` below). The projection filter for optical flow uses this to
    //   reject invalid points during relative pose estimation.
    // - The GPU implementations in `insta360.glsl`, `insta360.wgsl`, and `insta360.cl`
    //   DO NOT produce a validity flag; they always return the last iterate and are used
    //   only for rendering/undistortion.
    pub fn undistort_point(&self, point: (f32, f32), params: &KernelParams) -> Option<(f32, f32)> {
        let mut px = point.0;
        let mut py = point.1;
        let mut converged = false;

        for _ in 0..200 {
            let dp = self.distort_point(px, py, 1.0, params);
            let diff = (dp.0 - point.0, dp.1 - point.1);
            let max_diff = diff.0.abs().max(diff.1.abs());
            // This threshold is intentionally looser than the final break condition to improve 
            // validity checks for peripheral points without increasing the number of iterations.
            // Making it too loose (>5e-2) causes invalid points to be accepted.
            if max_diff < 2e-2 {
                converged = true;
                if max_diff < 1e-6 {
                    break;
                }
            }

            px -= diff.0;
            py -= diff.1;
        }

        if converged { Some((px, py)) } else { None }
    }

    pub fn distort_point(&self, mut x: f32, mut y: f32, z: f32, params: &KernelParams) -> (f32, f32) {
        let k1 = params.k[0];
        let k2 = params.k[1];
        let k3 = params.k[2];
        let p1 = params.k[3];
        let p2 = params.k[4];
        let xi = params.k[5];

        let len = (x.powi(2) + y.powi(2) + z.powi(2)).sqrt();

        x = (x / len) / ((z / len) + xi);
        y = (y / len) / ((z / len) + xi);

        let r2 = x*x + y*y;
        let r4 = r2 * r2;
        let r6 = r4 * r2;

        (
            x * (1.0 + k1*r2 + k2*r4 + k3*r6) + 2.0*p1*x*y + p2*(r2 + 2.0*x*x),
            y * (1.0 + k1*r2 + k2*r4 + k3*r6) + 2.0*p2*x*y + p1*(r2 + 2.0*y*y)
        )
    }
    pub fn adjust_lens_profile(&self, _profile: &mut crate::LensProfile) { }

    pub fn distortion_derivative(&self, _theta: f64, _k: &[f64]) -> Option<f64> {
        None
    }

    pub fn id() -> &'static str { "insta360" }
    pub fn name() -> &'static str { "Insta360" }

    pub fn opencl_functions(&self) -> &'static str { include_str!("insta360.cl") }
    pub fn wgsl_functions(&self)   -> &'static str { include_str!("insta360.wgsl") }
}
