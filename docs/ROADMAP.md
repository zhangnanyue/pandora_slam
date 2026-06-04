# Roadmap

Pandora SLAM is a newly public research prototype. The immediate goal is to make the codebase easier to build, inspect and adapt without exposing private datasets.

## Near Term

- Add a public sample configuration that can run against a small synthetic or public dataset.
- Document coordinate frames for camera, IMU, back LiDAR, front LiDAR and world/map references.
- Add unit tests for YAML parsing, transform composition and time-series interpolation.
- Integrate the semantic measurement module into the main visual-semantic localization target once the API is stable.
- Add a Docker or devcontainer build environment for reproducible dependency setup.

## Medium Term

- Publish a minimal example dataset or instructions for adapting KITTI/nuScenes-style folders.
- Add regression checks for ESKF initialization and prediction using deterministic fixtures.
- Expand LiDAR-camera alignment documentation with input/output diagrams and tuning notes.
- Add benchmark notes for CPU build time and runtime hot spots.

## Maintenance Principles

- Keep private datasets, generated debug outputs and machine-specific paths out of git.
- Prefer small examples over large binary assets.
- Keep CMake targets buildable before merging public changes.
- Document experimental assumptions instead of presenting prototype code as production-ready robotics software.
