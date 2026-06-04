# Architecture

Pandora SLAM is organized around small C++ modules that can be built as shared libraries and composed by runner targets.

## Data Flow

1. `run/visual_semantic_localization/main.cc` loads a YAML configuration into `VisualSemanticLocalizationOptions`.
2. `VisualSemanticLocalizationApp` owns the data-loading flow and reads raw images, semantic masks, IMU samples, ground-truth poses and semantic contours.
3. `DataGroup` packages synchronized data for one semantic-contour timestamp.
4. `VisualSemanticLocalizationCore` initializes the semantic map and ESKF, then processes each `DataGroup`.
5. Utility modules provide camera projection, YAML parsing, time interpolation, SO(3) helpers and timing/logging support.

## Module Boundaries

- `common_utils`: low-level helpers that should not depend on application modules.
- `camera_model`: camera projection and intrinsic model support.
- `data_loader`: file and timestamp parsing for experiment data.
- `ESKF`: inertial state estimation primitives.
- `lidar_visual_alignment`: LiDAR-camera alignment logic and calibration experiments.
- `visual_semantic_localization`: application-level semantic localization pipeline.

## Data Boundary

Private datasets, generated point clouds, debug images and calibration outputs are not part of the public repository. Config files should use placeholders or relative paths so users can point the code at their own data.
