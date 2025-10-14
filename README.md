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

**Step 1: Create a preset file** (e.g., `motion_preset.json`):
```json
{
  "version": 2,
  "stabilization": {
    "fov": 3.222,
    "method": "Plain 3D",
    "smoothing_params": [
      {"name": "time_constant", "value": 1.0}
    ],
    "horizon_lock_amount": 1.0,
    "motion_direction_enabled": true,
    "motion_direction_params": [
      {"name": "flip_backward_dir", "value": false}
    ]
  }
}
```

**Step 2: Process video with motion estimation:**
```bash
gyroflow video.mp4 --preset motion_preset.json -p "{'output_path': '/path/to/output.mp4'}"
```
The last argument is optional.


### GPS synchronization (single video)

**GUI:**
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

**CLI:**

**Step 1: Create a preset file** (e.g., `gps_preset.json`):
```json
{
  "version": 2,
  "stabilization": {
    "method": "Plain3D",
    "smoothing_params": [
      {"name": "time_constant", "value": 1.0}
    ],
    "motion_direction_enabled": true,
  }
}
```

**Step 2: Process video with GPS synchronization:**
**Specifying output paths:**
```bash
gyroflow video.mp4 \
  --preset gps_preset.json \
  --gpx-file track.gpx \
  --gps-settings "{ 'sync_mode': 'auto', 'use_processed_motion': true, 'speed_threshold': 1.5, 'max_time_offset_s': 5.0 }" \
  -p "{'output_path': '/outputs/stabilized.mp4'}" \
  --export-gpx /outputs/synchronized.gpx \
  --gps-report
```

This will:
1. Load the GPX track file (`--gpx-file track.gpx`)
2. Synchronize the GPS data using the processed motion direction and provided GPS settings
3. Export the synchronized GPX file (`--export-gpx synchronized.gpx`)
4. Create a GPS synchronization report file (`gps_report.txt`) with offset, similarity, and correlation information (`--gps-report`, requires `--export-gpx`)

GPS synchronization settings:
- `max_time_offset_s`: Maximum allowed time offset in seconds (default: 10.0, recommended: 5.0 to prevent incorrect synchronization)

The GPS synchronization will output the time offset in milliseconds and the correlation coefficient to help assess synchronization quality.

### Batch processing: motion direction alignment and GPS synchronization

**Batch processing with subdirectories (INSV + GPX files):**
```bash
# Structure: input_dir/
#   ├── session1/
#   │   ├── video1.insv
#   │   └── track1.gpx
#   ├── session2/
#   │   ├── video2.insv
#   │   └── track2.gpx
#   └── ...

# Process all subdirectories containing INSV and GPX files
for session_dir in input_dir/*/; do
    if [ -d "$session_dir" ]; then
        session_name=$(basename "$session_dir")
        mkdir -p "output/$session_name"
        
        insv_file=$(find "$session_dir" -name "*.insv" | head -1)
        gpx_file=$(find "$session_dir" -name "*.gpx" | head -1)
        if [ -f "$insv_file" ] && [ -f "$gpx_file" ]; then
            echo "Processing: $session_name"
            insv_basename=$(basename "$insv_file" .insv)
            gyroflow "$insv_file" \
              --preset "{ 'version': 2, 'stabilization': { 'fov': 3.222, 'method': 'Plain3D', 'smoothing_params': [{'name': 'time_constant', 'value': 1.0}], 'horizon_lock_amount': 1.0, 'motion_direction_enabled': true, 'motion_direction_params': [{'name': 'flip_backward_dir', 'value': false}] } }" \
              --estimate-motion \
              --gpx-file "$gpx_file" \
              --gps-settings "{ 'sync_mode': 'auto', 'use_processed_motion': true, 'speed_threshold': 1.5, 'max_time_offset_s': 5.0 }" \
              -p "{'output_folder': 'output/$session_name/', 'output_filename': '${insv_basename}.mp4'}" \
              --export-gpx "output/$session_name/${insv_basename}.gpx" \
              --gps-report
        else
            echo "Skipping $session_name: missing INSV or GPX file"
        fi
    fi
done
```

**Resulting output structure:**
```
output/
├── session1/
│   ├── video1.mp4
│   ├── video1.gpx  # if --export-gpx is specified
│   └── gps_report.txt   # if --gps-report is specified (requires --export-gpx)
├── session2/
│   ├── video2.mp4
│   ├── video2.gpx  # if --export-gpx is specified
│   └── gps_report.txt   # if --gps-report is specified (requires --export-gpx)
└── ...
```

## Testing videos

| ID                         | $\Delta t_\rho$ | $ρ$      | $\Delta t_{d_1}$ | $d_1$ | Description / Comments                                                              | Error |
| -------------------------- | --------------- | -------- | ---------------- | ----- | ----------------------------------------------------------------------------------- | ----- |
| 20231223_unit1_58_59_836   | 0.8             | .811     | 0.7              | 1.363 | 2 turns                                                                             | -     |
| 20241210_unit1_32          | 2.4             | .766     | **1.8**          | 1.427 | Many turns, **VQF inaccurate**, L1 distance is better                               | -     |
| 20241210_unit1_536         | 0.3             | .870     | 0.7              | 1.346 | One turn, then stationary                                                           | -     |
| 20241210_unit1_537         | 4.9             | **.213** | 5.2              | 0.697 | **Straight path – GPS sync fails**, synchronization doesn’t work (delays incorrect) | 1     |
| 20241210_unit4_539         | 0.9             | .669     | 0.7              | 3.479 | Opposite 180° turns, good for testing, clear GPS delay                              | -     |
| 20241210_unit5_112_627     | 1.4             | .653     | 1.3              | 1.800 | **Bad sync due to reverse direction flipping before 180° turn**                     | -     |
| 20241210_unit5_213_214_215 | 1.5             | .747     | 1.6              | 3.009 | Many turns                                                                          | -     |
| 20241210_unit5_216         | 3.1             | .622     | 1.7              | 2.688 | Stationary at start and mid                                                         | -     |
| 20241210_unit5_238         | 0.0             | .601     | 0.0              | 3.437 | **reverse direction flipping**                                                      | -     |
| 20241210_unit5_251_252     | 1.7             | .623     | 0.6              | 1.856 | **Motion direction errors on 90° head turns**, possibly related to backward motion  | -     |
| 20241210_unit5_257_258     | 1.3             | .619     | 1.5              | 4.005 | Small map                                                                           | -     |
| 20241212_unit3_640_642     | 0.2             | .732     | 0.3              | 4.068 | Backward-facing, many turns, **motion errors @ 2:04**                               | -     |
| 20241213_unit3_418         | 0.0             | .659     | 0.0              | 1.731 | Backward-facing, sharp corners                                                      | -     |
| 20241213_unit3_419         | 2.0             | .442     | 2.0              | 5.053 | Backward-facing, sharp corners, motion direction errors, correlation lower          | -     |
| 20241213_unit3_424         | 1.3             | .416     | 1.4              | 6.918 | Backward-facing, **very noisy GPS start**, delays incorrect (should be ~2.0 s)      | 1     |
| 20241213_unit3_845_859     | 0.2             | —        | 2.5              | 2.942 | Backward-facing, clear delay, correlation unreliable, corr. incorrect, L1 correct   | -     |
| 20241214_unit1_566_567     | 6.1             | .160     | 7.8              | 0.856 | **Straight path – GPS sync fails**, delays incorrect                                | 1     |



## Other notes

### Optical flows

Optical flow algorithms receive original undistorted frames.

DIS:
- DIS has some points in invalid positions (outside the fisheye disc).
- `optical_flow_to` returns points sampled in a grid with width 15 spread over the image. The first point is at $(0, 0)$. Offsetting the points so that their center of mass is at the center of the frame also offsets the motion direction estimate for some reason. #todo figure out.

PyrLK:
- Seems to work better than DIS for motion direction on 480p frames.
- On 720p and higher, it makes big errors very much.