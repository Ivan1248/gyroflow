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
    "method": "Plain3D",
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
  --gps-settings "{ 'sync_mode': 'auto', 'use_processed_motion': true, 'speed_threshold': 1.5 }" \
  -p "{'output_path': '/outputs/stabilized.mp4'}" \
  --export-gpx /outputs/synchronized.gpx
```

This will:
1. Load the GPX track file (`--gpx-file track.gpx`)
2. Synchronize the GPS data using the processed motion direction and provided GPS settings
3. Export the synchronized GPX file (`--export-gpx synchronized.gpx`)

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
              --gps-settings "{ 'sync_mode': 'auto', 'use_processed_motion': true, 'speed_threshold': 1.5, 'sample_rate_hz': 10 }" \
              -p "{'output_folder': 'output/$session_name/', 'output_filename': '${insv_basename}.mp4'}" \
              --export-gpx "output/$session_name/${insv_basename}.gpx"
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
│   └── sync_video1.gpx
├── session2/
│   ├── video2.mp4
│   └── sync_video2.gpx
└── ...
```

## Testing videos

- 20241210_unit1_536 – one turn, then stationary for a few seconds, GPS delay 0.7 s
- 20241210_unit1_537 – **straight path – GPS synchronization doesn't work**, GPS delay -11.9 s (incorrect)
- 20241210_unit1_32 – many turns, **VQF is inaccurate**, GPS delay 1.9s
- 20241210_unit4_539 – GPS and yaw have opposite 180° turns, good for GPS testing, **clear GPS delay** 1s
- 20241210_unit5_112_627 – **bad GPS synchronization with motion direction alignment due to reverse direction flipping before 180° turn**, GPS delay 1.6s
- 20241210_unit5_216 – stationary at beginning and at about 1/3 of the duration, **GPS delay 2.2s=
- 20241210_unit5_236 – **bad GPS synchronization (7 s delay) due to reverse direction flipping**, GPS delay 1.6s
- 20241210_unit5_251_252 – **motion direction errors on 90° head turns** (related to handling backward motion?), GPS delay 0.9s
- 20241210_unit5_257_258 – small map, GPS delay 1.9s
- 20241212_unit3_640_642 – backward-facing, many turns, **strange motion estimation errors at 2:04**, GPS delay 0.5s
- 20241213_unit3_424 – backward-facing, **very noisy GPS at the start**, **GPS delay 3.0s (incorrect, should be 2.0s)**
- 20241213_unit3_845_859 – **backward-facing**, motion direction alignment improves GPS synchronization, GPS delay 2.3s
- 20241214_unit1_566_567 – **straight path** – GPS synchronization doesn't work, GPS delay -9.8s (incorrect)


## Other notes

### Optical flows

Optical flow algorithms receive original undistorted frames.

DIS:
- DIS has some points in invalid positions (outside the fisheye disc).
- `optical_flow_to` returns points sampled in a grid with width 15 spread over the image. The first point is at $(0, 0)$. Offsetting the points so that their center of mass is at the center of the frame also offsets the motion direction estimate for some reason. #todo figure out.

PyrLK:
- Seems to work better than DIS for motion direction on 480p frames.
- On 720p and higher, it makes big errors very much.