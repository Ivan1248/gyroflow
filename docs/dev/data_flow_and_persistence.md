# Gyroflow Data Flow and Persistence

## Overview

Gyroflow follows a layered architecture with clear separation between the GUI (QML), controller (Rust QObject), and core processing logic (Rust). This document outlines how data flows between these layers and what gets persisted when.

## Architecture & Data Flow

### Core Layer (`src/core/`)
The foundation containing all data structures and processing logic:

- **StabilizationManager**: Single source of truth for all project data (thread-safe with RwLock)
- **Key Components**:
  - `gyro`: GyroSource (IMU data, orientation, transforms)
  - `lens`: LensProfile (calibration data, distortion models)
  - `smoothing`: Smoothing settings and algorithms
  - `stabilization`: Stabilization processing parameters
  - `keyframes`: KeyframeManager for time-based parameter animation
  - `params`: StabilizationParams (FOV, readout time, etc.)
  - `input_file`: InputFile (video URL, project file URL, etc.)
  - `sync_data`: SyncData (motion estimation results)
  - `gps`: GPS data and synchronization

### Controller Layer (`src/controller.rs`)
QObject interface bridging QML GUI and Rust core:

- **Role**: Translates QML property changes into core method calls
- **Qt Properties**: Direct bindings to GUI elements
- **Methods**: Exposes core functions (`load_video()`, `set_fov()`, `export_gyroflow_file()`)
- **Data Synchronization**: Controller properties mirror core data with automatic propagation

### GUI Layer (`src/ui/`)
QML-based user interface with automatic persistence:

- **Settings System**: Automatic save/load via `Settings` QObject with 1-second debouncing
- **Controller Binding**: GUI elements bind to controller properties and call methods
- **Auto-sync**: Changes propagate bidirectionally via Qt bindings and signals

### Data Flow Patterns

**GUI → Controller → Core:**
1. **Direct Property Binding**: `Slider { value: controller.fov; onValueChanged: controller.fov = value }`
2. **Method Calls**: `controller.set_smoothing_method(smoothingMethod.currentIndex)`
3. **Settings Auto-Save**: Components initialize with `settings.init(sett)` and call `settings.propChanged(sett)`

**Core → Controller → GUI:**
1. **Property Updates**: Core changes trigger controller property updates
2. **Signals**: Controller emits signals that GUI listens to
3. **Direct Reading**: GUI reads controller properties directly

## Persistence Mechanisms

### Application Settings (`settings.json`)
User data directory storage for UI preferences, device selections, and user-defined defaults. Automatically saved with 1-second debouncing when UI properties change.

### Project Files (`.gyroflow`)
User-specified files with complete project state. Export levels vary:

- **Simple**: Stabilization parameters, smoothing settings, keyframes, sync data
- **With Gyro Data**: Simple + raw IMU data, processed quaternions, file metadata
- **With Processed Data**: With Gyro Data + per-frame matrices, processed motion data

### Lens Profiles
Calibration data stored in `lens_profiles/` subdirectory with distortion models and user ratings.

## GUI Settings Reference

The following tables list all settings visible in Gyroflow's GUI. Each includes persistence method and default value source.

### Default Source Legend

**System-Derived:**
- **Hardcoded in QML**: Fixed defaults defined in GUI code
- **Hardcoded in Rust**: Fixed defaults defined in Rust code
- **Operating system**: Values from OS (window positions, locale)
- **Auto-calculated**: Computed from UI layout or capabilities
- **Auto-detected**: Detected from hardware/software configuration
- **Platform-specific default**: OS-specific default values

**User-Derived:**
- **User-defined**: Set by user actions or preferences
- **User choice**: Explicit user selections/decisions
- **User input**: Directly entered by user
- **Last used value/path/file**: Remembered from previous interactions
- **User-approved URLs**: Security-approved URLs

**Application-Specific:**
- **System locale**: OS language/locale settings
- **Lens profile default**: From loaded lens calibration
- **Algorithm-specific defaults**: Based on selected smoothing algorithm

### Table Column Reference

- **Setting Key**: The unique identifier used in Gyroflow's settings system and code
- **Component**: Which UI component or menu contains this setting
- **Description**: What the setting controls or affects
- **Type**: Data type - `number` (integer/float), `boolean`, `string`, or `JSON` (complex object)
- **Default**: The default value when the setting is not configured
- **Persistence**: Where and how the setting value is stored
- **Source for default value**: Where the default value is defined in the codebase
- **SoT copies**: All sources of truth that define the default value (GUI, core, CLI)
- **Priority Order**: The order in which sources are checked (highest to lowest priority)

### Window & UI Layout

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `windowX` | Main Window | Window X position | number | System default | Application settings | Operating system | GUI | User settings → GUI defaults → OS defaults |
| `windowY` | Main Window | Window Y position | number | System default | Application settings | Operating system | GUI | User settings → GUI defaults → OS defaults |
| `windowWidth` | Main Window | Window width | number | 1650 | Application settings | Hardcoded in GUI | GUI:main_window.qml:13 | User settings → GUI defaults |
| `windowHeight` | Main Window | Window height | number | 950 | Application settings | Hardcoded in GUI | GUI:main_window.qml:14 | User settings → GUI defaults |
| `visibility` | Main Window | Window visibility state | number | 0 | Application settings | Hardcoded in GUI | GUI:main_window.qml:43 | User settings → GUI defaults |
| `leftPanelSize` | App Layout | Left panel width | number | Auto | Application settings | Auto-calculated | GUI | User settings → GUI auto-calculation |
| `rightPanelSize` | App Layout | Right panel width | number | Auto | Application settings | Auto-calculated | GUI | User settings → GUI auto-calculation |
| `bottomPanelSize` | Video Area | Bottom panel height (windowed) | number | Auto | Application settings | Auto-calculated | GUI | User settings → GUI auto-calculation |
| `bottomPanelSize-full` | Video Area | Bottom panel height (fullscreen) | number | Auto | Application settings | Auto-calculated | GUI | User settings → GUI auto-calculation |
| `calibPanelSize` | Calibrator | Calibration panel width | number | Auto | Application settings | Auto-calculated | GUI | User settings → GUI auto-calculation |
| `stabOverviewSplit` | Video Area | Stabilization overview split view | boolean | false | Application settings | Hardcoded in GUI | GUI:VideoArea.qml:791 | User settings → GUI defaults |
| `uiScaling` | Advanced Menu | UI scaling factor | number | 0 | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:18 | User settings → GUI defaults |

### Stabilization Settings

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `smoothingMethod` | Stabilization | Smoothing algorithm selection | number | 1 | Application settings | Hardcoded in GUI | GUI:Stabilization.qml:266, core:smoothing/mod.rs:42 | CLI args → User settings → GUI defaults → core defaults |
| `croppingMode` | Stabilization | Adaptive zoom mode (No cropping/Dynamic/Static) | number | 1 | Application settings | Hardcoded in GUI | GUI:Stabilization.qml:577, core:stabilization_params.rs:140, CLI | CLI args → User settings → GUI defaults → core defaults → CLI logic |
| `adaptiveZoom` | Stabilization | Adaptive zoom window size | number | 4.0 | Application settings | Hardcoded in GUI | GUI:Stabilization.qml:619, core:stabilization_params.rs:140, CLI | CLI args → User settings → GUI defaults → core defaults |
| `correctionAmount` | Stabilization | Lens correction amount | number | 1.0 | Application settings | Hardcoded in GUI | GUI:Stabilization.qml:636, core:stabilization_params.rs:155, CLI | CLI args → User settings → GUI defaults → core defaults |
| `useGravityVectors` | Stabilization | Use gravity vectors for horizon lock | boolean | false | Project file | Hardcoded in core (GyroSource::new) | GUI:Stabilization.qml:431, core:gyro_source/mod.rs:104 | User settings → GUI defaults → core defaults |
| `hlIntegrationMethod` | Stabilization | Horizon lock integration method | number | 1 | Project file | Hardcoded in core (GyroSource::new) | GUI:Stabilization.qml:446, core:gyro_source/mod.rs:105 | User settings → GUI defaults → core defaults |
| `videoSpeedAffectsSmoothing` | Stabilization | Video speed affects smoothing | boolean | true | Project file | Hardcoded in GUI | GUI:Stabilization.qml:768, core:stabilization_params.rs:179 | User settings → GUI defaults → core defaults |
| `videoSpeedAffectsZooming` | Stabilization | Video speed affects zooming | boolean | true | Project file | Hardcoded in GUI | GUI:Stabilization.qml:781, core:stabilization_params.rs:180 | User settings → GUI defaults → core defaults |
| `videoSpeedAffectsZoomingLimit` | Stabilization | Video speed affects zoom limits | boolean | true | Project file | Hardcoded in GUI | GUI:Stabilization.qml:794, core:stabilization_params.rs:181 | User settings → GUI defaults → core defaults |
| `zoomingMethod` | Stabilization | Zooming method | number | 1 | Project file | Hardcoded in GUI | GUI:Stabilization.qml:817, core:stabilization_params.rs:142 | User settings → GUI defaults → core defaults |
| `maxZoom` | Stabilization | Maximum zoom limit | number | 130.0 | Project file | Hardcoded in GUI | GUI:Stabilization.qml:601, core:stabilization_params.rs:152 | User settings → GUI defaults → core defaults |
| `maxZoomIterations` | Stabilization | Maximum zoom iterations | number | 5 | Project file | Hardcoded in GUI | GUI:Stabilization.qml:923, core:stabilization_params.rs:153 | User settings → GUI defaults → core defaults |
| `smoothing-{method}-{param}` | Stabilization | Smoothing algorithm parameters | number | Varies | Project file | Algorithm-specific defaults | GUI, core | User settings → GUI defaults → Algorithm defaults |
| `motion_direction_enabled` | Stabilization | Motion direction alignment enabled | boolean | false | Project file | Hardcoded in GUI | GUI:Stabilization.qml:476, core:smoothing/motion_direction.rs:17 | User settings → GUI defaults → core defaults |
| `motion_direction_params` | Stabilization | Motion direction parameters | JSON | [] | Project file | Hardcoded in GUI | GUI:Stabilization.qml:476, core:smoothing/motion_direction.rs:18 | User settings → GUI defaults → core defaults |

#### Motion Direction Alignment (Data Flow)

- GUI initializes from project JSON:
  - If `motion_direction_enabled` exists, GUI calls `controller.set_motion_direction_enabled(enabled)` and updates the checkbox.
  - If `motion_direction_params` exists, GUI iterates and calls `controller.set_motion_direction_param(name, value)` for each entry; sliders/checkboxes are updated to match.
- No bulk-load controller method is used; parameters and enabled state are applied via individual setters (consistent with smoothing params).
- When the checkbox is turned on and no motion directions are present, the GUI triggers motion estimation automatically to populate data.
- Status messages for motion direction are displayed in the Motion Direction section and refreshed based on compute progress.

### Motion Estimation Settings

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `processingResolution` | Motion Data | Motion estimation resolution | number | 3 | Application settings | Hardcoded in GUI | GUI:MotionData.qml:1294 | User settings → GUI defaults |
| `ofMethod` | Motion Data | Optical flow method | number | 2 | Lens sync_settings | Lens profile default | GUI:MotionData.qml:1316 | User settings → Lens profile → GUI defaults |
| `poseMethod` | Motion Data | Pose estimation method | number | 0 | Lens sync_settings | Lens profile default | GUI:MotionData.qml:1337 | User settings → Lens profile → GUI defaults |
| `everyNthFrame` | Motion Data | Process every Nth frame | number | 1 | Lens sync_settings | Lens profile default | GUI:MotionData.qml:1347 | User settings → Lens profile → GUI defaults |
| `showFeatures` | Motion Data | Show detected features | boolean | false | Application settings | Hardcoded in GUI | GUI:MotionData.qml:1361, core:lib.rs:104 | User settings → GUI defaults → core defaults |
| `showOF` | Motion Data | Show optical flow | boolean | false | Application settings | Hardcoded in GUI | GUI:MotionData.qml:1371, core:stabilization_params.rs:135 | User settings → GUI defaults → core defaults |
| `showMotionDirection` | Motion Data | Show motion direction | boolean | true | Application settings | Hardcoded in GUI | GUI:MotionData.qml:1381, core:lib.rs:106 | User settings → GUI defaults → core defaults |

### GPS Synchronization Settings

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `gps_sync_mode` | GPS | GPS sync mode (Off/Auto/Manual) | number | 0 | Project file | Hardcoded in GUI | GUI:GPS.qml:770, core:gps/mod.rs:17 | User settings → GUI defaults → core defaults |
| `gps_use_processed_motion` | GPS | Use processed motion for GPS sync | boolean | false | Project file | Hardcoded in GUI | GUI:GPS.qml:782 | User settings → GUI defaults |
| `gps_sync_settings` | GPS | GPS sync method and parameters | JSON | {} | Project file | Hardcoded in GUI | GUI:GPS.qml:751-759 | User settings → GUI defaults |
| `gps_offset_ms` | GPS | Manual GPS offset in milliseconds | number | 0 | Project file | User input | GUI:GPS.qml:917, core:gps/mod.rs:22 | User settings → GUI defaults → core defaults |

### Synchronization Settings

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `initialOffset` | Synchronization | Initial sync offset | number | 0 | Application settings | Hardcoded in GUI | GUI:Synchronization.qml:224 | User settings → GUI defaults |
| `syncSearchSize` | Synchronization | Sync search window size | number | 5.0 | Application settings | Hardcoded in GUI | GUI:Synchronization.qml:248, CLI:cli.rs:826 | CLI args → User settings → GUI defaults → CLI hardcoded (5) |
| `maxSyncPoints` | Synchronization | Maximum sync points | number | 3 | Application settings | Hardcoded in GUI | GUI:Synchronization.qml:272, CLI:cli.rs:828 | CLI args → User settings → GUI defaults → CLI hardcoded (5) |
| `timePerSyncpoint` | Synchronization | Time per sync point | number | 1.5 | Application settings | Hardcoded in GUI | GUI:Synchronization.qml:301, CLI:cli.rs:830 | CLI args → User settings → GUI defaults → CLI hardcoded (1) |
| `sync_lpf` | Synchronization | Sync low-pass filter | number | 0 | Application settings | Hardcoded in GUI | GUI:Synchronization.qml:335 | User settings → GUI defaults |
| `checkNegativeInitialOffset` | Synchronization | Allow negative initial offset | boolean | false | Application settings | Hardcoded in GUI | GUI:Synchronization.qml:228 | User settings → GUI defaults |
| `experimentalAutoSyncPoints` | Synchronization | Use experimental auto sync points | boolean | false | Application settings | Hardcoded in GUI | GUI:Synchronization.qml:199, CLI:cli.rs:833 | CLI args → User settings → GUI defaults → CLI hardcoded (true) |

### Export Settings

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `defaultCodec` | Export | Default video codec | number | 1 | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:258, CLI | CLI args → User settings → GUI defaults → CLI mapping |
| `exportAudio` | Export | Export audio track | boolean | true | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:507, CLI:cli.rs:810 | CLI args → User settings → GUI defaults → CLI hardcoded (true) |
| `keyframeDistance` | Export | Keyframe distance | number | 1 | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:564, CLI:cli.rs:811 | CLI args → User settings → GUI defaults → CLI hardcoded (1) |
| `preserveOtherTracks` | Export | Preserve other video tracks | boolean | false | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:573, CLI:cli.rs:812 | CLI args → User settings → GUI defaults → CLI hardcoded (false) |
| `padWithBlack` | Export | Pad with black frames | boolean | false | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:580, CLI:cli.rs:813 | CLI args → User settings → GUI defaults → CLI hardcoded (false) |
| `exportTrimsSeparately` | Export | Export trims as separate files | boolean | false | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:587, CLI:cli.rs:814 | CLI args → User settings → GUI defaults → CLI hardcoded (false) |
| `useVulkanEncoder` | Export | Use Vulkan encoder | boolean | false | Application settings | Platform-specific default | GUI:menu/Export.qml:696 | User settings → Platform defaults → GUI defaults |
| `useD3D12Encoder` | Export | Use D3D12 encoder | boolean | false | Application settings | Platform-specific default | GUI:menu/Export.qml:705 | User settings → Platform defaults → GUI defaults |
| `metadataComment` | Export | Metadata comment | string | "" | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:552, CLI:cli.rs:815 | CLI args → User settings → GUI defaults → CLI empty string |
| `audioCodec` | Export | Audio codec | number | 0 | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:600, CLI:cli.rs:820 | CLI args → User settings → GUI defaults → CLI mapping |
| `interpolationMethod` | Export | Frame interpolation method | number | 2 | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:611, CLI:cli.rs:821 | CLI args → User settings → GUI defaults → CLI mapping |
| `preserveOutputSettings` | Export | Preserve output settings | boolean | false | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:670 | User settings → GUI defaults |
| `preserveOutputPath` | Export | Preserve output path | boolean | false | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:683 | User settings → GUI defaults |
| `preservedWidth` | Export | Last used output width | number | 0 | Application settings | Last used value | GUI | User settings → GUI auto-save |
| `preservedHeight` | Export | Last used output height | number | 0 | Application settings | Last used value | GUI | User settings → GUI auto-save |
| `preservedBitrate` | Export | Last used bitrate | number | 0 | Application settings | Last used value | GUI | User settings → GUI auto-save |
| `preservedOutputPath` | Export | Last used output folder | string | "" | Application settings | Last used value | GUI | User settings → GUI auto-save |
| `outputSizePresets` | Export | Custom output size presets | JSON | {} | Application settings | User-defined | GUI | User settings → GUI defaults |
| `exportGpu-{codec}` | Export | GPU encoding for specific codec | boolean | false | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:493, CLI | CLI args → User settings → GUI defaults → CLI codec lookup |
| `encoderOptions-{codec}` | Export | Encoder options for specific codec | string | "" | Application settings | Hardcoded in GUI | GUI:menu/Export.qml:527, CLI | CLI args → User settings → GUI defaults → CLI codec lookup |

### Advanced Settings

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `previewPipeline` | Advanced | Preview rendering pipeline | number | 0 | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:321 | User settings → GUI defaults |
| `renderBackground` | Advanced | Render background color | string | "#111111" | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:139 | User settings → GUI defaults |
| `safeAreaGuide` | Advanced | Show safe area guide | boolean | false | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:239 | User settings → GUI defaults |
| `gpudecode` | Advanced | GPU video decoding | boolean | true | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:246, core, CLI | CLI args → User settings → GUI defaults → core defaults |
| `backgroundMode` | Advanced | Background mode | number | 0 | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:92, core:stabilization_params.rs:157 | User settings → GUI defaults → core defaults |
| `marginPixels` | Advanced | Background margin pixels | number | 0 | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:103 | User settings → GUI defaults |
| `featherPixels` | Advanced | Background feather pixels | number | 0 | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:119 | User settings → GUI defaults |
| `defaultSuffix` | Advanced | Default filename suffix | string | "_stabilized" | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:399, CLI | CLI args → User settings → GUI defaults → CLI hardcoded |
| `playSounds` | Advanced | Play UI sounds | boolean | true | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:407 | User settings → GUI defaults |
| `sphericalGrid` | Advanced | Show spherical grid | boolean | false | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:231, core:stabilization_params.rs:137 | User settings → GUI defaults → core defaults |
| `theme` | Advanced | UI theme | number | 1 | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:154 | User settings → GUI defaults |
| `previewResolution` | Advanced | Preview resolution override | number | 0 | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:63 | User settings → GUI defaults |
| `uiScaling` | Advanced | UI scaling factor | number | 2 | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:179 | User settings → GUI defaults |
| `processingDevice` | Advanced | Processing device name | string | "" | Application settings | Auto-detected | GUI | User settings → Auto-detection → GUI defaults |
| `processingDeviceIndex` | Advanced | Processing device index | number | 0 | Application settings | Auto-detected | GUI | User settings → Auto-detection → GUI defaults |
| `renderingDevice` | Advanced | Rendering device | string | "" | Application settings | Auto-detected | GUI | User settings → Auto-detection → GUI defaults |
| `lang` | Advanced | UI language | string | Auto-detected | Application settings | System locale | GUI | User settings → System locale → GUI defaults |

### Lens Calibration Settings

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `calib_maxPoints` | Lens Calibrate | Maximum calibration points | number | 15 | Application settings | Hardcoded in GUI | GUI:menu/LensCalibrate.qml:203 | User settings → GUI defaults |
| `calib_everyNthFrame` | Lens Calibrate | Process every Nth frame for calibration | number | 10 | Application settings | Hardcoded in GUI | GUI:menu/LensCalibrate.qml:345 | User settings → GUI defaults |
| `calib_iterations` | Lens Calibrate | Calibration iterations | number | 500 | Application settings | Hardcoded in GUI | GUI:menu/LensCalibrate.qml:492 | User settings → GUI defaults |
| `calib_maxSharpness` | Lens Calibrate | Maximum sharpness threshold | number | 8 | Application settings | Hardcoded in GUI | GUI:menu/LensCalibrate.qml:358 | User settings → GUI defaults |
| `calibratedBy` | Lens Calibrate | Calibrator name | string | "" | Application settings | User input | GUI | User settings → GUI defaults |

### Timeline Settings

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `timelineChart` | Timeline | Timeline chart view mode | number | 0 | Application settings | Hardcoded in GUI | GUI:components/Timeline.qml:291 | User settings → GUI defaults |
| `restrictTrimRange` | Timeline | Restrict trim range | boolean | true | Application settings | Hardcoded in GUI | GUI:components/Timeline.qml:15 | User settings → GUI defaults |

### Video & Playback Settings

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `volume` | Video Area | Video playback volume | number | 100 | Application settings | Hardcoded in GUI | GUI:VideoArea.qml:1050 | User settings → GUI defaults |
| `imageSequenceFps` | Video Area | Image sequence FPS | number | 30 | Application settings | Hardcoded in GUI | GUI:VideoArea.qml:397 | User settings → GUI defaults |
| `dualStreamMode` | Video Info | Dual stream mode | string | "default" | Application settings | Hardcoded in GUI | GUI:menu/VideoInformation.qml:226, CLI | CLI args → User settings → GUI defaults |

### UI State & Preferences

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `{menuName}-opened` | Menu Items | Menu expanded/collapsed state | boolean | false | Application settings | Hardcoded in GUI | GUI (dynamic pattern) | User settings → GUI defaults |
| `gridLines` | Grid Guide | Number of grid lines | number | 0 | Application settings | Hardcoded in GUI | GUI:components/GridGuide.qml:64 | User settings → GUI defaults |
| `lensProfileFavorites` | Lens Profile | Favorite lens profiles | string | "" | Application settings | User-defined | GUI:menu/LensProfile.qml:187 | User settings → GUI defaults |
| `rated-profile-{checksum}` | Lens Profile | Profile rating submitted | boolean | false | Application settings | User action | GUI:menu/LensProfile.qml:128 | User settings → GUI defaults |
| `folder-{type}` | File Dialogs | Last used folder for file type | string | "" | Application settings | Last used path | GUI | User settings → GUI auto-save |
| `CSVExportSelection` | Motion Data | CSV export column selection | JSON | {} | Application settings | User-defined | GUI | User settings → GUI defaults |
| `lastProject` | Controller | Last opened project path | string | "" | Application settings | Last opened file | GUI | User settings → GUI auto-save |
| `allowedUrls` | Controller | Allowed external URLs | JSON array | [] | Application settings | User-approved URLs | GUI | User settings → GUI defaults |

### Render Queue Settings

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `renderQueue` | Render Queue | Queued render jobs | JSON | {} | Application settings | User-defined | GUI | User settings → GUI defaults |
| `parallelRenders` | Render Queue | Number of parallel renders | number | Auto | Application settings | Auto-detected | GUI | User settings → Auto-detection → GUI defaults |
| `defaultOverwriteAction` | Render Queue | Default overwrite action | number | 0 | Application settings | Hardcoded in GUI | GUI:RenderQueue.qml:843 | User settings → GUI defaults |
| `exportMode` | Render Queue | Export mode | number | 0 | Application settings | Hardcoded in GUI | GUI:RenderQueue.qml:853 | User settings → GUI defaults |
| `showQueueWhenAdding` | Render Queue | Show queue when adding items | boolean | true | Application settings | Hardcoded in GUI | GUI:RenderQueue.qml:856 | User settings → GUI defaults |

### RED Camera Settings

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `r3dConvertFormat` | Advanced | RED convert format | number | 0 | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:255 | User settings → GUI defaults |
| `r3dColorMode` | Advanced | RED color mode | number | 0 | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:273 | User settings → GUI defaults |
| `r3dGammaCurve` | Advanced | RED gamma curve | number | 7 | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:286 | User settings → GUI defaults |
| `r3dColorSpace` | Advanced | RED color space | number | 0 | Application settings | Hardcoded in GUI | GUI:menu/Advanced.qml:298 | User settings → GUI defaults |
| `r3dRedlineParams` | Advanced | RED additional parameters | string | "" | Application settings | User-defined | GUI | User settings → GUI defaults |

### Dialog States

| Setting Key | Component | Description | Type | Default | Persistence | Source for default value | SoT copies | Priority Order |
|-------------|-----------|-------------|------|---------|-------------|----------------|----------|----------------|
| `dontShowAgain-{identifier}` | Dialogs | Don't show again state | number | 0 | Application settings | User choice | GUI | User settings → GUI defaults |

## CLI Defaults

The Command Line Interface (CLI) uses the same settings system as the GUI but applies defaults through a dedicated `setup_defaults()` function in `src/cli.rs`. This ensures consistent behavior between GUI and CLI operations.

### CLI Default Application Process

1. **StabilizationManager Initialization**: CLI creates `StabilizationManager::default()` with built-in Rust defaults
2. **Settings Override**: `setup_defaults()` reads user settings from `settings.json` and applies them
3. **JSON Configuration**: Returns additional defaults for output and synchronization parameters

### CLI-Specific Default Mappings

The CLI `setup_defaults()` function maps settings to stabilization parameters with specific logic:

| Setting Key | CLI Behavior | Default Value | Notes |
|-------------|--------------|---------------|--------|
| `croppingMode` | Conditional zoom setting | 1 | 0=no zoom, 1=adaptive zoom, 2=static zoom |
| `adaptiveZoom` | Window size for dynamic zoom | 4.0 | Only applied when croppingMode=1 |
| `correctionAmount` | Lens correction multiplier | 1.0 | Applied to StabilizationManager |
| `smoothingMethod` | Algorithm selection | 1 | Followed by method-specific parameters |
| `smoothing-{method}-{param}` | Algorithm parameters | Varies | Dynamically loaded per method |
| `gpudecode` | GPU decoding enabled | true | Applied to StabilizationManager |
| `defaultSuffix` | Output filename suffix | "_stabilized" | Applied to RenderQueue |

### CLI Output Configuration Defaults

The CLI provides comprehensive default JSON configuration for rendering:

```json
{
  "output": {
    "codec": "H.264/AVC",  // Based on defaultCodec setting (0=H.264, 1=H.265, etc.)
    "use_gpu": true,       // Based on exportGpu-{codec} setting
    "audio": true,         // Based on exportAudio setting
    "keyframe_distance": 1,
    "encoder_options": "", // Based on encoderOptions-{codec}
    "metadata": {"comment": ""},
    "preserve_other_tracks": false,
    "pad_with_black": false,
    "export_trims_separately": false,
    "audio_codec": "AAC",  // Based on audioCodec setting
    "interpolation": "Lanczos4"
  },
  "synchronization": {
    "initial_offset": 0,
    "search_size": 5,
    "max_sync_points": 5,
    "every_nth_frame": 1,
    "time_per_syncpoint": 1,
    "of_method": 2,
    "offset_method": 2,
    "auto_sync_points": true
  }
}
```

### CLI vs GUI Default Consistency

The CLI ensures consistency with GUI defaults by:

- **Same Settings Source**: Both use `gyroflow_core::settings` with identical fallback values
- **Shared Defaults**: Stabilization parameters use the same defaults as GUI components
- **Lazy Loading**: Settings are loaded on-demand from the same `settings.json` file
- **Fallback Chain**: CLI → User settings → Hardcoded defaults → StabilizationManager defaults

### CLI Processing Order

```rust
// 1. Create StabilizationManager with Rust defaults
let stab = Arc::new(StabilizationManager::default());

// 2. Load lens profiles
stab.lens_profile_db.write().load_all();

// 3. Apply CLI/user settings with fallbacks
let additional_data = setup_defaults(&stab, &mut queue);

// 4. Override with CLI arguments (preset, out_params, sync_params)
```

This layered approach ensures CLI operations respect user preferences while providing sensible defaults for automation scenarios.
