## Modified Gyroflow

This repository contains a modified version of [Gyroflow](https://github.com/gyroflow/gyroflow) with the following functionality added:
- motion direction alignment – the virtual camera is pointed in the direction of motion estimated via relative pose estimation,
- GPS synchronization – GPX files can be loaded, synchronized with the motion estimated from IMU data and motion direction alignment,
- GPS data visualization – a simple map and GPS course angle alignment chart option in the timeline.

## Data extraction from omnidirectional videos

The videos have been recorded with head-mounted omnidirectional cameras. They contain gyroscope and accelerometer (IMU) data and unsynchronized GPS data.

Requirements:
- Transform fisheye projection to perspective projection with 127° field of view.
- Correct the camera direction (optical axis) to point along the motion direction.
- Synchronize GPS tracks with the video (IMU).

_Potential additional requirements: equirectangular projection, added distoriton?_

Checklist:
- [ ] Motion direction correction/alignment  
  - [x] Motion direction estimation  
    - Optical flow PyrLK on 480p best, DIS on any resolution also good, PyrLK on higher resolutions bad.  
    - Relative pose estimation: essential matrix via LMedS.  
  - [x] Basic camera direction correction for forward-facing camera  
  - [x] Smoothing  
  - [ ] Secondary (backward-camera) video stream support (for backward facing cameras)  
  - [ ] Handling of non-translating camera
- [ ] GPS synchronization  
  - [x] GPX loading  
  - [x] Synchronization  
    - [x] Manual  
    - [x] Automatic  
      - [x] Using motion direction and smoothing  
      - [ ] With improved interpolation  
  - [x] GPX saving  
  - [ ] Method to determine when GPS synchronization doesn't work (e.g. due to no turns)
- [ ] Batch processing
- [ ] Equirectangular projection?

## How to run

### Motion direction alignment (single video)

GUI:
1. Run Gyroflow.
2. Load the video via the "Open file" button in the "Video information" section or the "Drop video file here" area.
3. In the "Stabilization" section:
	1. Check the "Lock horizon" checkbox and the "Motion direction alignment" checkbox. This starts video analysis (relative pose estimation), which takes less time than the duration of the video on a CPU, a few times less on a GPU.
	2. Select "Plain 3D" smoothing and set the time constant to 1 s.
	3. Set the FOV to e.g. 127° by entering "3.222" into the FOV textbox.
	4. Optionally, check the "Flip backward direction" checkbox if you want to point the virtual camera opposite to the motion direction when the camera is moving backward. 
4. Once the analysis is complete, to render the output video, modify the output path and click the "Export" button.

CLI:

**Step 1: Create a preset file** (e.g., `preset.json`):
```json
{
  "version": 2,
  "stabilization": {
    "fov": 3.222,
    "method": "Plain 3D",
    "smoothing_params": [{"name": "time_constant", "value": 1.0}],
    "horizon_lock_amount": 1.0,
    "motion_direction_enabled": true,
    "motion_direction_params": [{"name": "flip_backward_dir", "value": false}]
  },
  "output": {"width": 960, "height": 540}
}
```

**Step 2: Process video with motion estimation:**
```bash
gyroflow video.mp4 --preset preset.json -p "{'output_path': '/path/to/output.mp4'}"
```
The last argument is optional.


### GPS synchronization (single video)

#### GUI

1. Run Gyroflow.
2. Load the video.
3. In the "Stabilization" section:
	1. Check the the "Motion direction alignment" checkbox.
	2. Select "Plain 3D" smoothing and set the time constant to 1 s.
	3. Uncheck the "Flip backward direction" checkbox to avoid errors when the camera temporarily looking sideways or back.
4. In the "Motion direction" section:
	1. Load the GPX file via the "Load GPX" button.
	2. Optionally, display the GPS alignment chart by right-clicking the timeline and selecting "Chart display mode"→"GPS", and display the GPS map by checking the "GPS map" checkbox.
	3. Select "Auto" under "GPS sync mode".
	4. Check "Use processed motion for sync". This so that the corrected and smoothed motion direction is used, which should be visible in the GPS chart.
	5. Set GPS speed threshold to 1.5 m/s.
	6. Once the analysis is complete, save the aligned GPX file, click on the "Export" link to open a menu and click "Export synchronized GPX".

#### CLI

**1. Create a preset file** (e.g., `preset.json`):
The preset file can be the same as before, but only the following is necessary:
```json
{
  "version": 2,
  "stabilization": {
    "method": "Plain 3D",
    "smoothing_params": [{"name": "time_constant", "value": 1.0}],
    "motion_direction_enabled": true,
  }
}
```

**2. Process video and synchronize the GPS track with the video:**

```bash
gyroflow video.mp4 \
  --preset preset.json \  # should have motion_direction_enabled and "Plain 3D" smoothing
  --input-gpx track.gpx \
  --gps-settings "{ 'sync_mode': 'auto', 'use_processed_motion': true, 'speed_threshold': 1.5, 'max_time_offset_s': 5.0 }" \
  -p "{'output_path': '/outputs/stabilized.mp4'}" \
  --export-gpx /outputs/synchronized.gpx \
  --gps-report
```

This will:
1. Load the GPX track file (`--input-gpx track.gpx`)
2. Synchronize the GPS data using the processed motion direction and provided GPS settings
3. Export the synchronized GPX file (`--export-gpx synchronized.gpx`)
4. Create a GPS synchronization report file (`gps_report.txt`) with offset, similarity, and correlation information (`--gps-report`, requires `--export-gpx`)

GPS synchronization settings:
- `max_time_offset_s`: Maximum allowed time offset

### Batch processing: motion direction alignment and GPS synchronization

**Batch processing with subdirectories (INSV + GPX files):**
```bash
# Structure: input_root/
#   ├── session1/
#   │   ├── video1.insv
#   │   └── track1.gpx
#   ├── session2/
#   │   ├── video2.insv
#   │   └── track2.gpx
#   └── ...

# Define input and output directories
input_root="input"
output_root="output"
output_root_abs=$(realpath $output_root)

# Define preset configurations
preset='{ "version": 2, "stabilization": { "fov": 3.222, "method": "Plain 3D", "smoothing_params": [{"name": "time_constant", "value": 1.0}], "horizon_lock_amount": 1.0, "motion_direction_enabled": true, "motion_direction_params": [{"name": "flip_backward_dir", "value": false}] }, "output": { "width": 960, "height": 540 } }'
preset_back='{ "version": 2, "stabilization": { "fov": 3.222, "method": "Plain 3D", "smoothing_params": [{"name": "time_constant", "value": 1.0}], "horizon_lock_amount": 1.0, "motion_direction_enabled": true, "motion_direction_params": [{"name": "flip_backward_dir", "value": true}] }, "output": { "width": 960, "height": 540 } }'

# Process all subdirectories containing INSV and GPX files
for rec_dir in "$input_root"/*/; do
    if [ -d "$rec_dir" ]; then
        rec_name=$(basename "$rec_dir")
        out_dir="${output_root_abs}/$rec_name"
        mkdir -p $out_dir
        
        vid_file=$(find "$rec_dir" -name "*.insv" | head -1)
        gpx_file=$(find "$rec_dir" -name "*.gpx" | head -1)

        if [ -f "$vid_file" ] && [ -f "$gpx_file" ]; then
            echo "Processing: $rec_name"
            vid_name=$(basename "$vid_file" .insv)
            # Look at the motion direction; synchronize GPS time
            export LD_LIBRARY_PATH="/home/igrubisic/projects/gyroflow/target/release:/home/igrubisic/projects/gyroflow/ext/6.4.3/gcc_64/lib:/home/igrubisic/projects/gyroflow/ext/ffmpeg-8.0-linux-clang-gpl-lite/lib:/home/igrubisic/projects/gyroflow/ext/ffmpeg-8.0-linux-clang-gpl-lite/lib/amd64" && /home/igrubisic/apps/Gyroflow/gyroflow "$vid_file" \
              --preset "$preset" \
              -p "{'output_folder': '${out_dir}/', 'output_filename': '${vid_name}.mp4'}" \
              --input-gpx "$gpx_file" \
              --gps-settings "{ 'sync_mode': 'auto', 'use_processed_motion': true, 'speed_threshold': 1.5, 'max_time_offset_s': 5.0 }" \
              --export-gpx "${out_dir}/${vid_name}.gpx" \
              --gps-report
            # Look back if moving backward
            gyroflow "$vid_file" --preset "$preset_back" -p "{'output_folder': '${output_root_abs}/$rec_name/', 'output_filename': '${vid_name}_back.mp4'}"
        else
            echo "Skipping $rec_name: missing INSV or GPX file"
        fi
    fi
done
```

**Resulting output structure:**
```
output/
├── session1/
│   ├── video1.mp4          # original preset (flip_backward_dir: false)
│   ├── video1_back.mp4     # flipped preset (flip_backward_dir: true)
│   ├── video1.gpx          # synchronized GPX (from first call)
│   └── gps_report.txt      # GPS sync report (from both calls)
├── session2/
│   ├── video2.mp4          # original preset (flip_backward_dir: false)
│   ├── video2_back.mp4     # flipped preset (flip_backward_dir: true)
│   ├── video2.gpx          # synchronized GPX (from first call)
│   └── gps_report.txt      # GPS sync report (from both calls)
└── ...
```

## Linux

The Linux [build](https://github.com/Ivan1248/gyroflow/releases) produces the following artifacts:
- `Gyroflow-linux64.AppImage` - Portable AppImage executable
- `Gyroflow-linux64.tar.gz` - Tar archive with executable and dependencies
- `Gyroflow-1.6.3-x86_64.AppImage.zsync` - Zsync file for incremental updates

### Running the AppImage

1. Make the AppImage executable:
   ```bash
   chmod +x Gyroflow-linux64.AppImage
   ```

2. Run the application:
   ```bash
   ./Gyroflow-linux64.AppImage
   ```

The AppImage is self-contained and includes all necessary dependencies. No installation is required.

### Running from tar.gz archive

1. Extract the archive:
   ```bash
   tar -xzf Gyroflow-linux64.tar.gz
   ```

2. Run the application:
   ```bash
   ./Gyroflow/gyroflow
   ```


## Testing videos

| ID                         | $\Delta t_\rho$ | $ρ_{\Delta t_\rho}$ | $\Delta t_{d_1}$ (max 5) | $d_1$ | $ρ$  | Description / Comments                                                              | Error |
| -------------------------- | --------------- | ------------------- | ------------------------ | ----- | ---- | ----------------------------------------------------------------------------------- | ----- |
| 20231223_unit1_58_59_836   | 0.8             | .811                | 0.7                      | 1.36  | .809 | 2 turns                                                                             | 0     |
| 20241210_unit1_32          | 2.4             | .766                | **1.8**                  | 1.43  | .475 | Many turns, **VQF inaccurate**, L1 distance is better                               | 0     |
| 20241210_unit1_536         | 0.3             | .870                | 0.7                      | 1.35  | .782 | One turn, then stationary                                                           | 0     |
| 20241210_unit1_537         | 4.9             | **.213**            | 5.2                      | 0.70  | .267 | **Straight path – GPS sync fails**, synchronization doesn’t work (delays incorrect) | 1     |
| 20241210_unit4_29          |                 |                     | 5                        | 0.97  | .338 | straight path?                                                                      | 1     |
| 20241210_unit4_30          |                 |                     | 2.9                      | 0.91  | .884 | very slow left turning, metrics good, but possible error                            | ?     |
| 20241210_unit4_31          |                 |                     | 1.4                      | 0.95  | .293 | straight, only bypassing a stationary truck                                         | 0     |
| 20241210_unit4_33_685      |                 |                     | 0.6                      | 2.18  | .780 | circuit with many turns                                                             | 0     |
| 20241210_unit4_539         | 0.9             | .669                | 0.7                      | 3.42  | .661 | Opposite 180° turns, good for testing, clear GPS delay                              | 0     |
| 20241210_unit5_112_627     | 1.4             | .653                | 1.3                      | 1.65  | .602 | **Bad sync due to reverse direction flipping before 180° turn**                     | 0     |
| 20241210_unit5_213_214_215 | 1.6             | .747                | 1.6                      | 3.01  | .746 | Many turns                                                                          | 0     |
| 20241210_unit5_216         | 3.1             | .622                | 1.7                      | 2.69  | .598 | Stationary at start and mid                                                         | 0     |
| 20241210_unit5_238         | 0.0             | .601                | 0.0                      | 3.40  | .438 | **reverse direction flipping**                                                      | 0     |
| 20241210_unit5_239         |                 |                     | 2.7                      | 2.98  | .617 | noisy motion direction due to dense traffic, maybe 0.5s off                         | 0     |
| 20241210_unit5_240_241     |                 |                     | 2.2                      | 4.02  | .124 | STRANGE: motion direction seems to use the back camera stream #todo                 | 0     |
| 20241210_unit5_242_243     |                 |                     | 2                        | 3.07  | .496 |                                                                                     | 0     |
| 20241210_unit5_244_245     |                 |                     | 1.4                      | 2.45  | .428 |                                                                                     | 0     |
| 20241210_unit5_246         |                 |                     | 1.3                      | 3.54  | .256 |                                                                                     | 0     |
| 20241210_unit5_247         |                 |                     | 2                        | 2.69  | .753 | STRANGE: motion direction seems to use the back camera stream #todo                 | 0     |
| 20241210_unit5_251_252     | 1.7             | .623                | 0.6                      | 1.86  | .630 | **Motion direction errors on 90° head turns**, possibly related to backward motion  | 0     |
| 20241210_unit5_257_258     | 1.3             | .619                | 1.5                      | 4.01  | .640 | Small map                                                                           | 0     |
| 20241212_unit3_640_642     | 0.2             | .732                | 0.3                      | 4.068 |      | Backward-facing, many turns, **motion errors @ 2:04**                               | 0     |
| 20241213_unit3_418         | 0.0             | .659                | 0.0                      | 1.731 |      | Backward-facing, sharp corners                                                      | 0     |
| 20241213_unit3_419         | 2.0             | .442                | 2.0                      | 5.053 |      | Backward-facing, sharp corners, motion direction errors, correlation lower          | 0     |
| 20241213_unit3_424         | 1.3             | .416                | 1.4                      | 6.918 |      | Backward-facing, **very noisy GPS start**, delays incorrect (should be ~2.0 s)      | 1     |
| 20241213_unit3_845_859     | 0.2             | —                   | 2.5                      | 2.942 |      | Backward-facing, clear delay, correlation unreliable, corr. incorrect, L1 correct   | 0     |
| 20241214_unit1_566_567     | 6.1             | .160                | 7.8                      | 0.856 |      | **Straight path – GPS sync fails**, delays incorrect                                | 1     |  

Correct offsets (estimate): 20/24.  

Error guesses ($\rho<0.45$):  
- TP=4, TN=14, FP=6, FN=0
- $P=0.4$, $R=1$  

Confidently correct without human checking: 14/24.

## Other notes

### Optical flows

Optical flow algorithms receive original undistorted frames.

DIS:
- DIS has some points in invalid positions (outside the fisheye disc).
- `optical_flow_to` returns points sampled in a grid with width 15 spread over the image. The first point is at $(0, 0)$. Offsetting the points so that their center of mass is at the center of the frame also offsets the motion direction estimate for some reason. #todo figure out.

PyrLK:
- Seems to work better than DIS for motion direction on 480p frames.
- On 720p and higher, it makes big errors very much.