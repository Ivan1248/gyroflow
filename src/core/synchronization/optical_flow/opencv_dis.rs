// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2021-2022 Adrian <adrian.eddy at gmail>

#![allow(unused_variables, dead_code)]
use super::super::OpticalFlowPair;
use super::{ OpticalFlowTrait, OpticalFlowMethod };

use std::collections::BTreeMap;
use std::sync::atomic::AtomicU32;
use std::sync::Arc;
use parking_lot::RwLock;
#[cfg(feature = "use-opencv")]
use opencv::{ core::{ Mat, Size, CV_8UC1, Vec2f }, prelude::{ MatTraitConst, DenseOpticalFlowTrait } };

#[derive(Clone)]
pub struct OFOpenCVDis {
    features: Vec<(f32, f32)>,
    img: Arc<image::GrayImage>,
    matched_points: Arc<RwLock<BTreeMap<i64, (Vec<(f32, f32)>, Vec<(f32, f32)>)>>>,
    timestamp_us: i64,
    size: (i32, i32),
    used: Arc<AtomicU32>,
}

impl OFOpenCVDis {
    pub fn detect_features(timestamp_us: i64, img: Arc<image::GrayImage>, width: u32, height: u32) -> Self {
        Self {
            features: Vec::new(),
            timestamp_us,
            size: (width as i32, height as i32),
            matched_points: Default::default(),
            img,
            used: Default::default()
        }
    }
}

impl OpticalFlowTrait for OFOpenCVDis {
    fn size(&self) -> (u32, u32) {
        (self.size.0 as u32, self.size.1 as u32)
    }
    fn features(&self) -> &Vec<(f32, f32)> { &self.features }

    fn optical_flow_to(&self, _to: &OpticalFlowMethod) -> OpticalFlowPair {
        #[cfg(feature = "use-opencv")]
        if let OpticalFlowMethod::OFOpenCVDis(next) = _to {
            let (w, h) = self.size;
            if let Some(matched) = self.matched_points.read().get(&next.timestamp_us) {
                return Some(matched.clone());
            }
            if self.img.is_empty() || next.img.is_empty() || w <= 0 || h <= 0 { return None; }


            let result = || -> Result<(Vec<(f32, f32)>, Vec<(f32, f32)>), opencv::Error> {
                let a1_img = unsafe { Mat::new_size_with_data_unsafe(Size::new(self.img.width() as i32, self.img.height() as i32), CV_8UC1, self.img.as_raw().as_ptr() as *mut std::ffi::c_void, 0) }?;
                let a2_img = unsafe { Mat::new_size_with_data_unsafe(Size::new(next.img.width() as i32, next.img.height() as i32), CV_8UC1, next.img.as_raw().as_ptr() as *mut std::ffi::c_void, 0) }?;

                let mut of = Mat::default();
                let mut optflow = opencv::video::DISOpticalFlow::create(opencv::video::DISOpticalFlow_PRESET_FAST)?;  // stride 4, patsh size 8
                //use opencv::prelude::DISOpticalFlowTraitConst;
                //log::debug!("DIS patch stride: {}, patch size: {}", optflow.get_patch_stride()?, optflow.get_patch_size()?);
                optflow.calc(&a1_img, &a2_img, &mut of)?;

                let mut points_a = Vec::new();
                let mut points_b = Vec::new();
                // hard-coded grid step size
                let num_points_over_width = 15_i32;  // TODO: make this a parameter
                let step = w / num_points_over_width;
                // offsets for centering the grid
                let w_offset = step / 2 + ((w - step) % step) / 2;
                let h_offset = step / 2 + ((h - step) % step) / 2;
                for i in (h_offset..a1_img.cols()).step_by(step as usize) {
                    for j in (w_offset..a1_img.rows()).step_by(step as usize) {
                        let pt = of.at_2d::<Vec2f>(j, i)?;
                        points_a.push((i as f32, j as f32));
                        points_b.push((i as f32 + pt[0] as f32, j as f32 + pt[1] as f32));
                    }
                }

                // Debug: Save frames with DIS centers and displacement vectors if environment variable is set
                if std::env::var("GYROFLOW_DEBUG_DIS").is_ok() {
                    let debug_dir = "debug_frames";
                    if let Err(e) = std::fs::create_dir_all(debug_dir) {
                        log::error!("Failed to create debug directory: {}", e);
                    }
                    
                    // Create frame 1 with centers and displacement vectors
                    let mut frame1_debug = (*self.img).clone();
                    
                    // Draw DIS centers and displacement vectors on frame 1
                    for (i, ((x_a, y_a), (x_b, y_b))) in points_a.iter().zip(points_b.iter()).enumerate() {
                        let center_x = *x_a as u32;
                        let center_y = *y_a as u32;
                        
                        if center_x < frame1_debug.width() && center_y < frame1_debug.height() {
                            // Draw center point (white cross)
                            for dx in -1..=1 {
                                for dy in -1..=1 {
                                    let px = (center_x as i32 + dx).max(0) as u32;
                                    let py = (center_y as i32 + dy).max(0) as u32;
                                    if px < frame1_debug.width() && py < frame1_debug.height() {
                                        let pixel = frame1_debug.get_pixel_mut(px, py);
                                        *pixel = image::Luma([255]); // White cross
                                    }
                                }
                            }
                            
                            // Draw displacement vector (white line)
                            let end_x = *x_b as u32;
                            let end_y = *y_b as u32;
                            
                            if end_x < frame1_debug.width() && end_y < frame1_debug.height() {
                                // Simple line drawing using Bresenham-like algorithm
                                let dx = (end_x as i32 - center_x as i32).abs();
                                let dy = (end_y as i32 - center_y as i32).abs();
                                let steps = dx.max(dy) as usize;
                                
                                if steps > 0 {
                                    for step in 0..=steps {
                                        let t = step as f32 / steps as f32;
                                        let line_x = (center_x as f32 + t * (end_x as f32 - center_x as f32)) as u32;
                                        let line_y = (center_y as f32 + t * (end_y as f32 - center_y as f32)) as u32;
                                        
                                        if line_x < frame1_debug.width() && line_y < frame1_debug.height() {
                                            let pixel = frame1_debug.get_pixel_mut(line_x, line_y);
                                            *pixel = image::Luma([255]); // White line
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                    let frame1_debug_path = format!("{}/dis_frame_{:06}_debug.png", debug_dir, self.timestamp_us);
                    if let Err(e) = frame1_debug.save(&frame1_debug_path) {
                        log::error!("Failed to save DIS debug frame: {} - {}", frame1_debug_path, e);
                    } else {
                        log::debug!("Successfully saved DIS debug frame: {}", frame1_debug_path);
                    }
                }
                Ok((points_a, points_b))
            }();

            self.used.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            next.used.fetch_add(1, std::sync::atomic::Ordering::SeqCst);

            match result {
                Ok(res) => {
                    self.matched_points.write().insert(next.timestamp_us, res.clone());
                    return Some(res);
                },
                Err(e) => {
                    log::error!("OpenCV error: {:?}", e);
                }
            }
        }
        None
    }

    // Can cleanup if the frame has been used twice: as the next and as the current frame
    fn can_cleanup(&self) -> bool {
        self.used.load(std::sync::atomic::Ordering::SeqCst) == 2
    }
    fn cleanup(&mut self) {
        self.img = Arc::new(image::GrayImage::default());
    }
}
