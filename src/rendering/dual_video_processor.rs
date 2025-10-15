// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2025

use super::ffmpeg_processor::{ FfmpegProcessor, FFmpegError };
use ffmpeg_next::{ frame, Dictionary };
use std::sync::{ Arc, atomic::AtomicBool };
use std::collections::VecDeque;

pub struct DualVideoProcessor<'a> {
    pub front: FfmpegProcessor<'a>,
    pub back:  FfmpegProcessor<'a>,
}

impl<'a> DualVideoProcessor<'a> {
    pub fn from_file_with_streams(
        base: &'a gyroflow_core::filesystem::EngineBase,
        url: &str,
        gpu_decoding: bool,
        gpu_decoder_index: usize,
        decoder_options: Option<Dictionary>,
        front_stream_index: usize,
        back_stream_index: usize,
    ) -> Result<Self, FFmpegError> {
        // Note: callers should ensure indices are valid and distinct
        let front = FfmpegProcessor::from_file_with_video_stream_index(base, url, gpu_decoding, gpu_decoder_index, decoder_options.clone(), Some(front_stream_index))?;
        let back  = FfmpegProcessor::from_file_with_video_stream_index(base, url, gpu_decoding, gpu_decoder_index, decoder_options, Some(back_stream_index))?;
        ::log::info!("[dual_video_processor] Selected stream indices: front={}, back={}", front_stream_index, back_stream_index);
        ::log::info!("[dual_video_processor] Opened processors: front.input_index={}, back.input_index={}", front.video.input_index, back.video.input_index);
        Ok(Self { front, back })
    }

    /// Start decoding only and invoke provided callbacks independently per stream.
    /// Alignment and fusion are not handled here (skeleton only).
    pub fn start_decoder_only<F1, F2>(
        &mut self,
        ranges_ms: Vec<(f64, f64)>,
        cancel_flag: Arc<AtomicBool>,
        mut on_front_frame: F1,
        mut on_back_frame: F2,
    ) -> Result<(), FFmpegError>
    where
        F1: FnMut(i64, &mut frame::Video) + Send + 'static,
        F2: FnMut(i64, &mut frame::Video) + Send + 'static,
    {
        let cf1 = cancel_flag.clone();

        self.front.on_frame(move |ts, input, _out, _conv, _rc| {
            on_front_frame(ts, input);
            if cf1.load(std::sync::atomic::Ordering::SeqCst) { return Ok(()); }
            Ok(())
        });

        self.back.on_frame(move |ts, input, _out, _conv, _rc| {
            on_back_frame(ts, input);
            Ok(())
        });

        // For now, decode full ranges sequentially per processor.
        // Future: interleave pumps and implement timestamp alignment.
        self.front.start_decoder_only(ranges_ms.clone(), cancel_flag.clone())?;
        self.back.start_decoder_only(ranges_ms, cancel_flag)?;
        Ok(())
    }

    /// Decode both streams and emit timestamp-aligned pairs within the given tolerance (microseconds).
    /// This is a basic, memory-backed implementation for correctness validation.
    pub fn decode_and_align<F>(
        &mut self,
        ranges_ms: Vec<(f64, f64)>,
        cancel_flag: Arc<AtomicBool>,
        max_delta_us: i64,
        mut on_pair: F,
    ) -> Result<(), FFmpegError>
    where
        F: FnMut(i64, &mut frame::Video, &mut frame::Video) + Send + 'static,
    {
        let front_frames = Arc::new(parking_lot::Mutex::new(Vec::<(i64, frame::Video)>::new()));
        let back_frames  = Arc::new(parking_lot::Mutex::new(Vec::<(i64, frame::Video)>::new()));

        {
            let cf = cancel_flag.clone();
            let ff = front_frames.clone();
            self.front.on_frame(move |ts, input, _out, _conv, _rc| {
                if cf.load(std::sync::atomic::Ordering::SeqCst) { return Ok(()); }
                ff.lock().push((ts, input.clone()));
                Ok(())
            });
        }
        self.front.start_decoder_only(ranges_ms.clone(), cancel_flag.clone())?;

        {
            let bf = back_frames.clone();
            self.back.on_frame(move |ts, input, _out, _conv, _rc| {
                bf.lock().push((ts, input.clone()));
                Ok(())
            });
        }
        self.back.start_decoder_only(ranges_ms, cancel_flag)?;

        // Align by two-pointer walk
        let mut a: VecDeque<(i64, frame::Video)> = front_frames.lock().drain(..).collect();
        let mut b: VecDeque<(i64, frame::Video)> = back_frames.lock().drain(..).collect();

        let mut _last_ts = 0i64;
        while let (Some((tsa, mut fa)), Some((tsb, mut fb))) = (a.front().cloned(), b.front().cloned()) {
            let d = tsa - tsb;
            if d.abs() <= max_delta_us {
                _last_ts = (tsa + tsb) / 2;
                let _ = a.pop_front();
                let _ = b.pop_front();
                on_pair(_last_ts, &mut fa, &mut fb);
            } else if d < 0 { // A leads B
                let _ = a.pop_front();
            } else { // B leads A
                let _ = b.pop_front();
            }
        }
        Ok(())
    }
}


