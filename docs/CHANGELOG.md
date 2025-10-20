# Changelog

All notable changes to this project are documented here. This log focuses on the recent work to preserve color in point clouds, standardize outputs, and introduce dominant-color analysis.

## 2025-10-20 / 0.9.0-alpha.2

### Added
- `pcg_bbox` standalone CLI: compute the centers of two UOBB PLYs, the vector from the first to the second, and the Euclidean distance between the centers.
- `gen` subcommand for `pcg_bbox`: `pcg_bbox gen <out.ply> cx cy cz lx ly lz yaw_deg` for quickly generating a test UOBB PLY (Z-up; yaw in degrees).
- `point` subcommand for `pcg_bbox`: `pcg_bbox point x y z <bbox_uobb.ply>` to compute the vector and distance from a user-provided point to the bbox center.

### Changed
- Merged the former test generator `pcg_bbox_gen` into `pcg_bbox` as the `gen` subcommand to reduce maintenance and simplify usage.
- Docs: README updated with `pcg_bbox` usage and examples.

### Removed
- Deprecated and removed the standalone `pcg_bbox_gen` executable and source.

### Notes
- `pcg_bbox` compute mode remains simple: provide two `*_uobb.ply` files exported by `pcg_room`; the center is computed by averaging the box vertices (equals the geometric center). Output format is four lines: `center1`, `center2`, `vector_1_to_2`, `distance`.

## 2025-10-18 / 0.9.0-alpha.1

### Added
- Color-preserving PLY IO (XYZRGB): Load and export point clouds with RGB attributes; backward compatible with XYZ-only PLYs.
- Standardized results layout under `results/` for each room, with unified filenames.
- CSV schema extended: `object_code`, `class`, `object_id`, `room_id`, `floor_id` added.
- `pcg_color` standalone CLI for dominant color analysis of clusters/PLYs.
- Color GMM core library (diagonal, weighted EM; k-means++ init; log-sum-exp; predict API).

### Changed
- `pcg_room` clustering/export: per-cluster PLY exports now keep vertex colors and follow the naming pattern `<object_code>_<class>_{cluster|uobb|mesh}.ply`.
- All outputs consolidated beneath `<room>/results/` (previous ad-hoc directories removed).
- Default configuration updated to a single perceptual decision rule in Lab: keep components if ΔE*76 ≥ `color_deltaE_keep` (20 by default).
- `pcg_reconstruct` mesh naming aligned with the new pattern; one mesh per cluster (accepted Poisson or AF fallback).

### Removed
- Deprecated experimental apps and artifacts: `pcg_api`, synthetic generators, and temporary test data.
- Obsolete parameter `color_deltaE_merge` (replaced by a single `color_deltaE_keep` threshold).

### Fixed
- Loss of color during IO and exports due to XYZ-only handling. Introduced RGB-aware readers/writers and switched internal point type to `PointXYZRGB` where relevant.
- Inconsistent directory structure and filenames across modules. Unified under `results/` and standardized naming/CSV.

### Notes
- Dominant-color method: force K=3 in RGB, filter components by minimum weight and per-channel stddev (≤ `color_max_stddev`), convert remaining centroids to CIELab (D65), merge visually similar colors using ΔE*76 < `color_deltaE_keep`. If all components are filtered, the app reports `M=0` (no dominant color).
- `pcg_color` prints a concise summary: final component count `M` and for each component its weight, mean RGB, and variance.

