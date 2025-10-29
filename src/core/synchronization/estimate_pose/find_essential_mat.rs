// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2021-2022 Adrian <adrian.eddy at gmail>

#![allow(unused_variables, dead_code)]
use super::super::OpticalFlowPair;
use super::{ EstimateRelativePoseTrait, RelativePose };

use nalgebra::Rotation3;
#[cfg(feature = "use-opencv")]
use opencv::{ core::{ Mat, Point2f }, prelude::MatTraitConst };

use crate::stabilization::*;

/// Robust estimation method for essential matrix calculation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobustMethod {
    /// Least Median of Squares - more robust to outliers, slower
    LMEDS,
    /// Random Sample Consensus - faster
    RANSAC,
}

impl Default for RobustMethod {
    fn default() -> Self { Self::LMEDS }
}

/// Wrapper for opencv::calib3d::find_essential_mat with hard-coded parameters
/// Returns the essential matrix and inlier mask
pub fn find_essential_mat_wrapper(
    pts1: &impl opencv::core::ToInputArray,
    pts2: &impl opencv::core::ToInputArray,
    camera_matrix: &impl opencv::core::ToInputArray,
    robust_method: RobustMethod,
) -> Result<(Mat, Mat), opencv::Error> {
    let mut mask = Mat::default();
    let (method, confidence, threshold, max_iters) = match robust_method {
        RobustMethod::LMEDS => (opencv::calib3d::LMEDS, 0.999, 0.00001, 4000),
        RobustMethod::RANSAC => (opencv::calib3d::RANSAC, 0.999, 1.0, 4000),
    };
    let essential_mat = opencv::calib3d::find_essential_mat(
        pts1, pts2, camera_matrix, method, confidence, threshold, max_iters, &mut mask
    )?;
    Ok((essential_mat, mask))
}

#[derive(Clone)]
pub struct PoseFindEssentialMat {
    pub robust_method: RobustMethod,
}
impl Default for PoseFindEssentialMat {
    fn default() -> Self { Self { robust_method: RobustMethod::LMEDS } }
}

impl EstimateRelativePoseTrait for PoseFindEssentialMat {
    fn init(&mut self, _: &ComputeParams) { }
    fn estimate_relative_pose(&self, pairs: &OpticalFlowPair, size: (u32, u32), params: &ComputeParams, timestamp_us: i64, next_timestamp_us: i64) -> Option<RelativePose> {
        let (pts1, pts2) = pairs.as_ref()?;

        #[cfg(feature = "use-opencv")]
        let result = || -> Result<RelativePose, opencv::Error> {
            use crate::util::cv_to_mat3;

            let pts11 = undistort_points_for_optical_flow(&pts1, timestamp_us, params, size);
            let pts22 = undistort_points_for_optical_flow(&pts2, next_timestamp_us, params, size);

            let pts1 = pts11.into_iter().map(|(x, y)| Point2f::new(x, y)).collect::<Vec<Point2f>>();
            let pts2 = pts22.into_iter().map(|(x, y)| Point2f::new(x, y)).collect::<Vec<Point2f>>();

            let a1_pts = Mat::from_slice(&pts1)?;
            let a2_pts = Mat::from_slice(&pts2)?;
            let identity = Mat::eye(3, 3, opencv::core::CV_64F)?;  // identity camera matrix
            let (e, mut mask) = find_essential_mat_wrapper(&a1_pts, &a2_pts, &identity, self.robust_method)?;

            let mut r1 = Mat::default();
            let mut t = Mat::default();
            // Note: camera coordinates; +Z forward convention
            let num_inliers = opencv::calib3d::recover_pose_triangulated(&e, &a1_pts, &a2_pts, &identity, &mut r1, &mut t, 100000.0, &mut mask, &mut Mat::default())?;
            if num_inliers < 10 { return Err(opencv::Error::new(0, "Model not found".to_string())); }
            
            // Extract rotation and translation direction
            let rotation = Rotation3::from_matrix_unchecked(crate::util::cv_to_mat3(&r1)?);
            let t = nalgebra::Vector3::new(*t.at_2d::<f64>(0, 0)?, *t.at_2d::<f64>(1, 0)?, *t.at_2d::<f64>(2, 0)?);
            let transl_dir = if t.norm() > 0.0 { Some(nalgebra::Unit::new_normalize(t)) } else { None };

            // Inlier ratio and mask (after recover_pose_triangulated)
            let num_points = mask.rows(); // Number of points is number of columns for 1xN point array
            let inlier_ratio = Some((num_inliers as f64 / num_points as f64).min(1.0));
            let mut inlier_mask_vec = Vec::with_capacity(mask.rows() as usize);
            for i in 0..mask.rows() {
                inlier_mask_vec.push(*mask.at_2d::<u8>(i, 0)?);
            }

            Ok(RelativePose { rotation, transl_dir, inlier_ratio, inlier_mask: Some(inlier_mask_vec) })
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
