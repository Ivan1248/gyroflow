## Modified Gyroflow

This repository contains a modified version of [Gyroflow](https://github.com/gyroflow/gyroflow) with the following functionality added:
- motion direction alignment – the virtual camera is pointed in the direction of motion estimated vie relative pose estimation,
- and GPS synchronization – GPX files can be loaded, synchronized with the motion estimated from IMU data and motion direction alignment,
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
	- [v] Motion direction estimation
		- Optical flow PyrLK on 480p best, DIS on any resolution also good, PyrLK on higher resolutions bad.
		- Relative pose estimation: essential matrix via LMedS.
	- [v] Basic camera direction correction for forward-facing camera
	- [v] Smoothing
	- [ ] Secondary (backward-camera) video stream support (for backward facing cameras)
	- [ ] Handling of non-translating camera
- [ ] GPS synchronization
	- [v] GPX loading
	- [v] Synchronization
		- [v] Manual
		- [v] Automatic
			- [v] Using motion direction and smoothing
			- [ ] With improved interpolation
	- [v] GPX saving
- [ ] Batch processing
- [ ] Equirectangular projection?