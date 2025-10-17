# Testing videos

| ID                         | $\Delta t_\rho$ | $ρ_{\Delta t_\rho}$ | $\Delta t_{d_1}$ (max 5) | $d_1$ | $ρ$  | Error | Notes                                                                               |
| -------------------------- | --------------- | ------------------- | ------------------------ | ----- | ---- | ----- | ----------------------------------------------------------------------------------- |
| 20231223_unit1_58_59_836   | 0.8             | .811                | 0.7                      | 1.36  | .809 | 0     | 2 turns                                                                             |
| 20241210_unit1_32          | 2.4             | .766                | **1.8**                  | 1.43  | .475 | 0     | Many turns, **VQF inaccurate**, L1 distance is better                               |
| 20241210_unit1_536         | 0.3             | .870                | 0.7                      | 1.35  | .782 | 0     | One turn, then stationary                                                           |
| 20241210_unit1_537         | 4.9             | **.213**            | 5.2                      | 0.70  | .267 | 1     | **Straight path – GPS sync fails**, synchronization doesn't work (delays incorrect) |
| 20241210_unit4_29          |                 |                     | 5                        | 0.97  | .338 | 1     | straight path?                                                                      |
| 20241210_unit4_30          |                 |                     | 2.9                      | 0.91  | .884 | ?     | very slow left turning, metrics good, but possible error                            |
| 20241210_unit4_31          |                 |                     | 1.4                      | 0.95  | .293 | 0     | straight, only bypassing a stationary truck                                         |
| 20241210_unit4_33_685      |                 |                     | 0.6                      | 2.18  | .780 | 0     | circuit with many turns                                                             |
| 20241210_unit4_539         | 0.9             | .669                | 0.7                      | 3.42  | .661 | 0     | Opposite 180° turns, clear GPS delay                                                |
| 20241210_unit5_112_627     | 1.4             | .653                | 1.3                      | 1.65  | .602 | 0     | reverse direction flipping before 180° turn                                         |
| 20241210_unit5_213_214_215 | 1.6             | .747                | 1.6                      | 3.01  | .746 | 0     | Many turns                                                                          |
| 20241210_unit5_216         | 3.1             | .622                | 1.7                      | 2.69  | .598 | 0     | Stationary at start and mid                                                         |
| 20241210_unit5_238         | 0.0             | .601                | 0.0                      | 3.40  | .438 | 0     | reverse direction flipping                                                          |
| 20241210_unit5_239         |                 |                     | 2.7                      | 2.98  | .617 | 0     | noisy motion direction due to dense traffic, maybe 0.5s off                         |
| 20241210_unit5_240_241     |                 |                     | 2.2                      | 4.02  | .124 | 0     | FFmpeg "best" stream is  the back camera stream                                     |
| 20241210_unit5_242_243     |                 |                     | 2                        | 3.07  | .496 | 0     |                                                                                     |
| 20241210_unit5_244_245     |                 |                     | 1.4                      | 2.45  | .428 | 0     |                                                                                     |
| 20241210_unit5_246         |                 |                     | 1.3                      | 3.54  | .256 | 0     |                                                                                     |
| 20241210_unit5_247         |                 |                     | 2                        | 2.69  | .753 | 0     | FFmpeg "best" stream is  the back camera stream                                     |
| 20241210_unit5_251_252     | 1.7             | .623                | 0.6                      | 1.86  | .630 | 0     | **Motion direction errors on 90° head turns**, possibly related to backward motion  |
| 20241210_unit5_257_258     | 1.3             | .619                | 1.5                      | 4.01  | .640 | 0     | Small map                                                                           |
| 20241212_unit3_640_642     | 0.2             | .732                | 0.3                      | 4.068 |      | 0     | Backward-facing, many turns, **motion errors @ 2:04**                               |
| 20241213_unit3_418         | 0.0             | .659                | 0.0                      | 1.731 |      | 0     | Backward-facing, sharp corners                                                      |
| 20241213_unit3_419         | 2.0             | .442                | 2.0                      | 5.053 |      | 0     | Backward-facing, sharp corners, motion direction errors, correlation lower          |
| 20241213_unit3_424         | 1.3             | .416                | 1.4                      | 6.918 |      | 1     | Backward-facing, **very noisy GPS start**, delays incorrect (should be ~2.0 s)      |
| 20241213_unit3_845_859     | 0.2             | —                   | 2.5                      | 2.942 |      | 0     | Backward-facing, clear delay, correlation unreliable, corr. incorrect, L1 correct   |
| 20241214_unit1_566_567     | 6.1             | .160                | 7.8                      | 0.856 |      | 1     | **Straight path – GPS sync fails**, delays incorrect                                |

Correct offsets (estimate): 20/24.
Error guesses ($\rho<0.45$):
- TP=4, TN=14, FP=6, FN=0
- $P=0.4$, $R=1$
Confidently correct without human checking: 14/24


## Other notes

### Optical flows

Optical flow algorithms receive original undistorted frames.

DIS:
- DIS has some points in invalid positions (outside the fisheye disc).
- `optical_flow_to` returns points sampled in a grid with width 15 spread over the image. The first point is at $(0, 0)$. Offsetting the points so that their center of mass is at the center of the frame also offsets the motion direction estimate for some reason. #todo figure out.

PyrLK:
- Seems to work better than DIS for motion direction on 480p frames.
- On 720p and higher, it makes big errors very much.
