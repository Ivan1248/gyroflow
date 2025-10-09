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
- [ ] Batch processing
- [ ] Equirectangular projection?

## How to run

### Motion direction alignment (single video)

1. Run Gyroflow.
2. Load a video via the "Open file" button in the "Video information" section or the "Drop video file here" area.
3. In the "Stabilization" section:
	1. Check the "Lock horizon" checkbox and the "Motion direction alignment" checkbox. This starts video analysis (relative pose estimation), which takes less time than the duration of the video on a CPU, a few times less on a GPU.
	2. Select "Plain 3D" smoothing and set the time constant to 1.5 s.
	3. Set the FOV to e.g. 127° by entering "3.222" into the FOV textbox.
4. Once the analysis is complete, to render the output video, modify the output path and click the "Export" button.

### GPS synchronization

1. Perform the **motion direction alignment** steps except for the video rendering and optionally FOV setting.
2. In the "Motion direction" section:
	1. Load the GPX file via the "Load GPX" button.
	2. Optionally, display the GPS alignment chart by right-clicking the timeline and selecting "Chart display mode"→"GPS", and display the GPS map by checking the "GPS map" checkbox.
	3. Select "Auto" under "GPS sync mode".
	4. Check "Use processed motion for sync".
	5. Once the analysis is complete, save the aligned GPX file, click on the "Export" link to open a menu and click "Export synchronized GPX".

## Testing videos

- 20241210_unit1_536 – one turn, then stationary for a few seconds
- 20241210_unit1_537 – ==straight path – GPS synchronization doesn't work==
- 20241210_unit1_32 – many turns, ==VQF is inaccurate==, GPS delay ~2s
- 20241210_unit4_539 – GPS and yaw have opposite 180° turns, good for GPS testing, ==clear GPS delay== ~0.9s
- 20241210_unit5_112_627 – ==bad GPS synchronization with motion direction alignment due to reverse direction flipping before 180° turn==, GPS delay 1.8s
- 20241210_unit5_216 – stationary at beginning and at about 1/3 of the duration, ==GPS delay ~2.5s=
- 20241210_unit5_236 – ==bad GPS synchronization (7 s delay) due to reverse direction flipping==, GPS delay ~1.7s
- 20241210_unit5_251_252 – ==motion direction errors on 90° head turns== (related to handling backward motion?)
- 20241210_unit5_257_258 – small map, GPS delay ~2.4s
- 20241213_unit3_845_859 – ==backward-facing==, motion direction alignment improves GPS synchronization, GPS delay ~2s
- 20241212_unit3_640_642 – backward-facing
- 20241214_unit1_566_567 – straight path – GPS synchronization doesn't work

## Other notes

### Optical flows

Optical flow algorithms receive original undistorted frames.

DIS:
- DIS has some points in invalid positions (outside the fisheye disc).
- `optical_flow_to` returns points sampled in a grid with width 15 spread over the image. The first point is at $(0, 0)$. Offsetting the points so that their center of mass is at the center of the frame also offsets the motion direction estimate for some reason. #todo figure out.

PyrLK:
- Seems to work better than DIS for motion direction on 480p frames.
- On 720p and higher, it makes big errors very much.