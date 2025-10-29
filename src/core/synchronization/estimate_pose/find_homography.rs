// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2021-2022 Adrian <adrian.eddy at gmail>

#![allow(unused_variables, dead_code)]

//! Homography-based pose estimation for camera stabilization
//!
//! This module implements pose estimation using homography decomposition, which is effective for 
//! scenes with dominant planar surfaces (e.g., ground, walls, buildings).

use super::super::OpticalFlowPair;
use super::{ EstimateRelativePoseTrait, RelativePose };

use nalgebra::Rotation3;
#[cfg(feature = "use-opencv")]
use opencv::{ core::{ Mat, Point2f, Vector }, prelude::MatTraitConst };

use crate::stabilization::*;

#[derive(Default, Clone)]
pub struct PoseFindHomography;

// TODO: reduce code duplication with respect to find_essential_mat.rs
impl EstimateRelativePoseTrait for PoseFindHomography {
    fn init(&mut self, _: &ComputeParams) { }
    fn estimate_relative_pose(&self, pairs: &OpticalFlowPair, size: (u32, u32), params: &ComputeParams, timestamp_us: i64, next_timestamp_us: i64) -> Option<RelativePose> {
        let (pts1, pts2) = pairs.as_ref()?;

        #[cfg(feature = "use-opencv")]
        let result = || -> Result<RelativePose, opencv::Error> {
            let pts11 = undistort_points_for_optical_flow(&pts1, timestamp_us, params, size);
            let pts22 = undistort_points_for_optical_flow(&pts2, next_timestamp_us, params, size);

            let pts1 = pts11.into_iter().map(|(x, y)| Point2f::new(x, y)).collect::<Vec<Point2f>>();
            let pts2 = pts22.into_iter().map(|(x, y)| Point2f::new(x, y)).collect::<Vec<Point2f>>();

            let a1_pts = Mat::from_slice(&pts1)?;
            let a2_pts = Mat::from_slice(&pts2)?;

            // Find homography matrix using RANSAC
            // Homography relates points between two views: x2 = H * x1
            let identity = Mat::eye(3, 3, opencv::core::CV_64F)?;  // identity camera matrix
            let mut mask = Mat::default();
            let homography = opencv::calib3d::find_homography_ext(&a1_pts, &a2_pts, opencv::calib3d::RANSAC, 0.001, &mut mask, 2000, 0.999)?;
            
            // Decompose homography into rotation, translation, and plane normal
            // For planar scenes: H = K * (R + t * n^T / d) * K^(-1)
            let mut r: Vector<Mat> = Default::default();  // Rotation matrices
            let mut t: Vector<Mat> = Default::default();  // Translation vectors
            let mut n: Vector<Mat> = Default::default();  // Plane normals
            opencv::calib3d::decompose_homography_mat(&homography, &identity, &mut r, &mut t, &mut n)?;

            // Select the solution with the smallest translation (assuming smallest movement)
            if let Some((r, t, _)) = r.iter().zip(t.iter()).fold(None, |cr, (r, t)| {
                    let dot = t.dot(&t).unwrap_or(0.0);  // Calculate ||t||^2
                    match cr {
                        Some((cr, ct, m)) if m < dot => Some((cr, ct, m)),  // Keep solution with smaller translation
                        _ => Some((r, t, dot)),  // First solution or better solution
                    }
                }) {
                // Extract rotation and translation direction
                let rotation = Rotation3::from_matrix_unchecked(crate::util::cv_to_mat3(&r)?);
                let t = nalgebra::Vector3::new(*t.at_2d::<f64>(0, 0)?, *t.at_2d::<f64>(1, 0)?, *t.at_2d::<f64>(2, 0)?);
                let transl_dir = if t.norm() > 0.0 { Some(nalgebra::Unit::new_normalize(t)) } else { None };

                // Calculate inlier ratio and inlier mask
                let num_inliers = opencv::core::count_non_zero(&mask)? as f64;
                let num_points = mask.rows(); // Number of points is number of columns for 1xN point array
                let inlier_ratio = Some((num_inliers as f64 / num_points as f64).min(1.0));
                let mut inlier_mask_vec = Vec::with_capacity(mask.rows() as usize);
                for i in 0..mask.rows() {
                    inlier_mask_vec.push(*mask.at_2d::<u8>(i, 0)?);
                }

                Ok(RelativePose { rotation, transl_dir, inlier_ratio, inlier_mask: Some(inlier_mask_vec) })
            } else {
                Err(opencv::Error::new(0, "Model not found".to_string()))
            }
        }();
        #[cfg(not(feature = "use-opencv"))]
        let result = Err(());

        match result {
            Ok(res) => Some(res),
            Err(e) => {
                log::error!("OpenCV error: {:?}", e);
                None
            }
        }
    }
}
