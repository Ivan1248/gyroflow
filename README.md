## Gyroflow with motion direction alignment and GPS synchronization 

This repository contains a modified version of [Gyroflow](https://github.com/gyroflow/gyroflow) for the purposes of a road safety assesment project. The following functionality is added:
- Motion direction alignment – the virtual camera is pointed in the direction of motion estimated using relative pose estimation.
- Secondary video stream selection option.
- GPS synchronization – GPX files can be loaded and synchronized with the motion estimated from IMU data and motion direction alignment.
- Frame sampling based on GPS coordinates (pre-defined or based on a distance interval).
- GPS data visualization – a simple map and GPS course angle alignment chart option in the timeline.

Some features are work-in-progress and the implementations can be improved.

## Data extraction from omnidirectional videos

Our videos have been recorded with head-mounted omnidirectional cameras. They contain gyroscope and accelerometer (IMU) data and unsynchronized GPS data.

**Requirements:**
- Transform fisheye projection to perspective projection with 127° field of view
- Correct the camera direction (optical axis) to point along the forward motion direction
- Synchronize GPS tracks with the video (IMU)

**Features:**
- [x] Motion direction correction/alignment
  - [x] Motion direction estimation
    - Optical flow PyrLK on 480p best, DIS on any resolution also good, PyrLK on higher resolutions bad.
    - Relative pose estimation: essential matrix via LMedS.
  - [x] Basic camera direction correction for forward-facing camera
  - [x] Smoothing
- [x] GPS synchronization
  - [x] GPX loading
  - [x] Synchronization
    - [x] Manual
    - [x] Automatic
      - [x] Using motion direction and smoothing with speed threshold
  - [x] Visualization (map and synchronization plots)
  - [x] GPX export
  - [x] GPS-based frame sampling
    - [x] Based on a distance interval with speed-based filtering
    - [x] Based on explicit coordinatex (CSV)
  - [x] CSV waypoints export
  - [x] Method to determine when GPS synchronization doesn't work (e.g. due to no turns) – 
	  - See evaluation results in [EVALUATION.md](EVALUATION.md)
- [x] Secondary (backward-camera) video stream support (for backward facing cameras)
  - [x] CLI support for stream selection (`--stream 0/1`)
  - [ ] Stitching of front and back video streams
  - [ ] Equirectangular projection?
- [ ] Added distortion
- [x] CLI and batch processing
- [ ] No bugs

## Configuration

### Default presets

**Motion direction alignment preset:**
```json
{
  "version": 2,
  "stabilization": {
    "fov": 3.222,
    "method": "Plain 3D",
    "smoothing_params": [{"name": "time_constant", "value": 1.0}],
    "horizon_lock_amount": 100.0,
    "motion_direction_enabled": true,
    "motion_direction_params": [
      {"name": "min_inlier_ratio", "value": 0.5},
      {"name": "flip_backward_dir", "value": false}
    ]
  },
  "output": {"width": 1280, "height": 920}
}
```

**GPS synchronization-only preset:**
```json
{
  "version": 2,
  "stabilization": {
    "method": "Plain 3D",
    "smoothing_params": [{"name": "time_constant", "value": 1.0}],
    "motion_direction_enabled": true
  }
}
```

### GPS settings

**Default GPS settings:**
```json
{
  "sync_mode": "auto",
  "use_processed_motion": true,
  "speed_threshold": 1.5,
  "max_time_offset_s": 5.0
}
```

## How to run

### Builds

Built Linux and Windows executables can be downloaded from [Releases](https://github.com/Ivan1248/gyroflow/releases).

The Linux build produces the following artifacts:
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

**Process video with motion estimation:**
```bash
gyroflow video.mp4 --preset preset.json -p '{ "output_path": "/path/to/output.mp4" }' --report-motion motion_report.txt
```
The `-p` and `--report-motion` arguments are optional.

**Motion report generation:**
- `--report-motion <path>`: creates a motion report file containing the average translation direction and the index of the forward looking video stream (based on relative pose estimation)


### GPS synchronization (single video)

#### GUI

1. Run Gyroflow.
2. Load the video.
3. In the "Stabilization" section:
	1. Check the "Motion direction alignment" checkbox.
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

**Process video and synchronize the GPS track:**

```bash
gyroflow video.mp4 \
  --preset preset.json \
  --input-gpx track.gpx \
  --gps-settings '{ "sync_mode": "auto", "use_processed_motion": true, "speed_threshold": 1.5, "max_time_offset_s": 5.0 }' \
  -p '{ "output_path": "/outputs/stabilized.mp4" }' \
  --overwrite \
  --export-gpx /outputs/synchronized.gpx \
  --report-gps /outputs/gps_report.txt \
  --stream 0
```

This will:
1. Load the GPX track file (`--input-gpx track.gpx`)
2. Synchronize the GPS data using the processed motion direction and provided GPS settings
3. Export the synchronized GPX file (`--export-gpx synchronized.gpx`)
4. Create a GPS synchronization report with offset, error, correlation, and course range information (`--report-gps <path>`, requires `--export-gpx`)
5. Use the first video stream (`--stream 0`)


**Render images correspondin to selected GPS waypoints:**

For rendering of individual frames as images at regular distances, use:
```
  -p '{ "output_path": "frames/%05d.png", "codec": "PNG" }' \
```
and additionally supply:
```
  --frame-sampling gps \
  --distance-interval 10.0 \
  --sampling-min-speed 1.0 \
```
For pre-defined GPS coordinates, supply:
```
  --sampling-coords waypoints.csv \
```
instead of `distance-interval` and `sampling-min-speed`.


### CLI Options Reference

#### Dual-stream video processing

- `--stream <mode>`: Select video stream
  - `0` or `front` (default): Use front/default camera stream
  - `1` or `back`: Use back camera stream
  - `stitched`: use stitched dual-stream mode (not implemented)

#### Motion analysis and reporting

- `--report-motion <path>`: Create motion report file with average translation direction and forward stream information

#### GPS synchronization and reporting

- `--input-gpx <file>`: Load GPX track file for synchronization
- `--export-gpx <path>`: Export synchronized GPX file
- `--gps-settings <json>`: GPS synchronization settings (sync_mode, use_processed_motion, speed_threshold, max_time_offset_s, sample_rate_hz)
- `--report-gps <path>`: Generate GPS synchronization report with offset, error, correlation, and course range information

#### GPS waypoint-based frame sampling

- `--frame-sampling <mode>`: Frame sampling mode for sequence exports
  - `all` (default): Export all frames
  - `gps`: Export frames sampled based on GPS data
- `--distance-interval <meters>`: Sample frames at regular distance intervals along the GPS track (requires `--frame-sampling gps`)
- `--sampling-min-speed <m/s>`: Minimum speed threshold for GPS distance sampling (default: 0.0 m/s). Frames are only sampled when the GPS speed exceeds this threshold (requires `--distance-interval`)
- `--sampling-coords <file>`: CSV file containing GPS waypoint coordinates for frame sampling (lat,lon per line). Overrides distance-based sampling when provided (requires `--frame-sampling gps`)

**CSV format for waypoints:**
```
# Latitude and longitude coordinates, one per line
lat1,lon1
lat2,lon2
```
Additional columns are allowed and ignored in parsing.


## Batch processing scripts

This repository provides automated batch processing capabilities for INSV and GPX files with motion direction alignment and GPS synchronization.

### Directory structure

**Input structure:**
```
input_root/
├── session1/
│   ├── video1.insv
│   └── track1.gpx
├── session2/
│   ├── video2.insv
│   └── track2.gpx
└── ...
```

**Output structure (video):**
```
output/
├── session1/
│   ├── video1.mp4          # Stabilized video with forward motion alignment
│   ├── video1.gpx          # Synchronized GPX file
│   ├── video1_motion.txt   # Motion analysis report
│   └── video1_gps.txt      # GPS synchronization report
├── session2/
│   ├── video2.mp4
│   ├── video2.gpx
│   ├── video2_motion.txt
│   └── video2_gps.txt
└── ...
```

### Python scripts

This repository includes two Python scripts for batch processing and data analysis.

### batch_process.py

A Python script for batch processing INSV and GPX files with motion direction alignment and GPS synchronization.

**Usage:**
```bash
python scripts/batch_process.py input_directory output_directory [options]
```

**Arguments:**
- `input_directory`: Directory containing subdirectories with INSV and GPX files
- `output_directory`: Directory where processed results will be saved

**Options:**
- `--dir-regex <pattern>`: Regex pattern for input subdirectories (default: all directories)
- `--gyroflow-cmd <command>`: Gyroflow command/path (default: 'gyroflow')

**Features:**
- Processes dual-stream videos (front/back camera)
- Automatically selects the forward-facing stream based on motion analysis
- Applies motion direction alignment and GPS synchronization
- Generates motion and GPS reports for analysis

**Example:**
```bash
# Process all subdirectories
python scripts/batch_process.py ./input ./output

# Process only directories starting with 'session' and use custom gyroflow command
python scripts/batch_process.py ./input ./output --dir-regex 'session.*' --gyroflow-cmd './gyroflow-cli'
```

### summarize.py

A Python script that summarizes motion and GPS report files into CSV format for analysis and statistics.

**Usage:**
```bash
python scripts/summarize.py output_directory [options]
```

**Arguments:**
- `output_directory`: Directory containing the report files (motion and GPS .txt files)

**Options:**
- `--unified`: Create unified summary combining motion and GPS data with classification
- `--correctness-expr <expression>`: Expression for correctness classification (default: `correlation > 0.45 and course_range_deg > 45 and offset < 4`)

**Output files:**
- `motion_summary.csv`: Motion analysis data (stream, avg_transl_dir, is_looking_forward)
- `gps_summary.csv`: GPS synchronization data (offset, error, correlation, course_range_deg)
- `unified_summary.csv`: Combined motion and GPS data with correctness classification (when using `--unified`)

**Example:**
```bash
# Create separate motion and GPS summary CSVs
python scripts/summarize.py ./output

# Create unified summary with custom correctness criteria
python scripts/summarize.py ./output --unified --correctness-expr "correlation > 0.5 and offset < 3"
```

**CSV format example:**
```csv
video,offset,error,correlation,course_range_deg,correct
session1/video1,0.8,0.811,0.809,45.2,true
session2/video2,2.4,0.766,0.475,120.1,false
```
