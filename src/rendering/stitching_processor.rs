// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2025

use super::dual_video_processor::DualVideoProcessor;
use super::cpu_stitcher::CpuStitcher;
use super::ffmpeg_processor::FFmpegError;
use super::ffmpeg_video_converter::Converter;
use super::ffmpeg_video::RateControl;
use ffmpeg_next::{ frame, format::Pixel, ffi };
use ffmpeg_next::software; // for pixel format conversion
use std::sync::{ Arc, atomic::{AtomicBool, Ordering} };
use std::cell::RefCell;

/// Processor that mimics FfmpegProcessor interface but feeds stitched dual-stream frames
/// through the existing stabilization pipeline
pub struct StitchingProcessor<'a> {
    dual_proc: DualVideoProcessor<'a>,
    stitcher: CpuStitcher,
    on_frame_callback: RefCell<Option<Box<dyn FnMut(i64, &mut frame::Video, Option<&mut frame::Video>, &mut Converter, &mut RateControl) -> Result<(), FFmpegError> + Send>>>,
    // Mock fields to satisfy existing code expectations
    pub video_codec: Option<String>,
    pub gpu_encoding: bool,
    pub video: VideoTranscoderMock,
}

pub struct VideoTranscoderMock {
    pub gpu_encoding: bool,
    pub encoder_params: EncoderParamsMock,
    pub processing_order: super::ffmpeg_video::ProcessingOrder,
}

pub struct EncoderParamsMock {
    pub pixel_format: Option<Pixel>,
}

impl<'a> StitchingProcessor<'a> {
    pub fn new(dual_proc: DualVideoProcessor<'a>, stitcher: CpuStitcher) -> Self {
        Self {
            dual_proc,
            stitcher,
            on_frame_callback: RefCell::new(None),
            video_codec: Some("libx264".to_string()), // Default codec
            gpu_encoding: false,
            video: VideoTranscoderMock {
                gpu_encoding: false,
                encoder_params: EncoderParamsMock {
                    pixel_format: Some(Pixel::YUV420P),
                },
                processing_order: super::ffmpeg_video::ProcessingOrder::PreConversion,
            },
        }
    }

    pub fn on_frame<F>(&self, callback: F) -> Result<(), FFmpegError>
    where
        F: FnMut(i64, &mut frame::Video, Option<&mut frame::Video>, &mut Converter, &mut RateControl) -> Result<(), FFmpegError> + Send + 'static,
    {
        *self.on_frame_callback.borrow_mut() = Some(Box::new(callback));
        Ok(())
    }

    pub fn start_decoder_only(&mut self, ranges: Vec<(f64, f64)>, cancel_flag: Arc<AtomicBool>) -> Result<(), FFmpegError> {
        ::log::info!("[stitching_processor] Starting dual-stream decode and align");

        // Collect aligned frame pairs
        let aligned_frames = Arc::new(parking_lot::Mutex::new(Vec::<(i64, frame::Video, frame::Video)>::new()));

        {
            let frames = aligned_frames.clone();
            self.dual_proc.decode_and_align(
                ranges,
                cancel_flag.clone(),
                33_000, // 33ms tolerance for 30fps
                move |ts: i64, front_frame: &mut frame::Video, back_frame: &mut frame::Video| {
                    // Clone frames to store them (expensive but necessary for thread safety)
                    let front_clone = unsafe { frame::Video::wrap(ffi::av_frame_clone(front_frame.as_ptr())) };
                    let back_clone = unsafe { frame::Video::wrap(ffi::av_frame_clone(back_frame.as_ptr())) };
                    frames.lock().push((ts, front_clone, back_clone));
                }
            )?;
        }

        // Now process the collected frames (single-threaded)
        let mut callback = self.on_frame_callback.borrow_mut().take()
            .ok_or_else(|| FFmpegError::InternalError(ffmpeg_next::Error::Bug))?;

        // For stitched output, we expect RGBA format
        let mut output_frame = frame::Video::new(Pixel::RGBA, self.stitcher.output_width, self.stitcher.output_height);

        // Lazy-initialized converters for input -> RGBA
        let mut front_converter: Option<software::scaling::Context> = None;
        let mut back_converter: Option<software::scaling::Context> = None;

        for (ts, front_frame, back_frame) in aligned_frames.lock().drain(..) {
            if cancel_flag.load(Ordering::SeqCst) {
                break;
            }

            // Ensure input frames are in RGBA before stitching
            let mut front_rgba: frame::Video;
            if front_frame.format() != Pixel::RGBA {
                if front_converter.is_none() {
                    match software::scaling::Context::get(
                        front_frame.format(),
                        front_frame.width(),
                        front_frame.height(),
                        Pixel::RGBA,
                        front_frame.width(),
                        front_frame.height(),
                        software::scaling::Flags::BILINEAR,
                    ) {
                        Ok(conv) => front_converter = Some(conv),
                        Err(e) => { ::log::error!("[stitching_processor] Failed to create front RGBA converter: {e:?}"); continue; }
                    }
                }
                let mut tmp = frame::Video::new(Pixel::RGBA, front_frame.width(), front_frame.height());
                if let Err(e) = front_converter.as_mut().unwrap().run(&front_frame, &mut tmp) {
                    ::log::error!("[stitching_processor] Front frame conversion to RGBA failed: {e:?}");
                    continue;
                }
                front_rgba = tmp;
            } else {
                // Clone to get an owned, mutable frame
                front_rgba = unsafe { frame::Video::wrap(ffi::av_frame_clone(front_frame.as_ptr())) };
            }

            let mut back_rgba: frame::Video;
            if back_frame.format() != Pixel::RGBA {
                if back_converter.is_none() {
                    match software::scaling::Context::get(
                        back_frame.format(),
                        back_frame.width(),
                        back_frame.height(),
                        Pixel::RGBA,
                        back_frame.width(),
                        back_frame.height(),
                        software::scaling::Flags::BILINEAR,
                    ) {
                        Ok(conv) => back_converter = Some(conv),
                        Err(e) => { ::log::error!("[stitching_processor] Failed to create back RGBA converter: {e:?}"); continue; }
                    }
                }
                let mut tmp = frame::Video::new(Pixel::RGBA, back_frame.width(), back_frame.height());
                if let Err(e) = back_converter.as_mut().unwrap().run(&back_frame, &mut tmp) {
                    ::log::error!("[stitching_processor] Back frame conversion to RGBA failed: {e:?}");
                    continue;
                }
                back_rgba = tmp;
            } else {
                back_rgba = unsafe { frame::Video::wrap(ffi::av_frame_clone(back_frame.as_ptr())) };
            }

            // Stitch the frames
            match self.stitcher.stitch(&front_rgba, &back_rgba) {
                Ok(stitched) => {
                    // For now, create mock converter and rate control
                    // TODO: Properly integrate with existing pipeline
                    let mut mock_converter = Converter::default();
                    let mut mock_rate_control = RateControl::default();

                    // Call the callback with the stitched frame
                    // We need to clone the stitched frame since it needs to be mutable for the callback
                    let mut stitched_clone = unsafe { frame::Video::wrap(ffi::av_frame_clone(stitched.as_ptr())) };
                    let _ = callback(ts, &mut stitched_clone, Some(&mut output_frame), &mut mock_converter, &mut mock_rate_control);
                }
                Err(e) => {
                    ::log::error!("Stitching failed for ts {}: {:?}", ts, e);
                    // Continue processing other frames
                }
            }
        }

        Ok(())
    }
}

// Note: Converter and RateControl use their derived Default implementations
