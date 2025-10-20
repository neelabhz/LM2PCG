# Changelog

All notable changes to this project are documented here. This log mirrors the style of `docs/CHANGELOG.md` and focuses on the latest integrations for AI orchestration and structured outputs.

## 2025-10-20 / 0.9.0-alpha.3

### Added
- Python AI API layer `scripts/ai_api.py` for chatbot/automation workflows:
  - Path resolution by filename, `object_code` (e.g., `0-7-12`), and floor-room CSV.
  - Head-code dispatcher: `RCN` (reconstruct), `VOL` (mesh volume/closedness), `CLR` (dominant color), `BBD` (bbox distance between two objects).
  - `check-env` command to verify executables; `--json` option for CLI responses.
  - `--filename` accepts absolute/relative paths in addition to names under `output/`.
- Global structured JSON output mode:
  - New config toggle `json_output: true` in `data/configs/default.yaml`.
  - C++ apps emit JSON to stdout when enabled: `pcg_reconstruct`, `pcg_volume`, `pcg_color`, `pcg_bbox`.
- Documentation updates:
  - `docs/AI_API.md` explains head codes, usage examples, and JSON schemas.
  - README now includes AI API quick-start and JSON output instructions.

### Changed
- Standardized console outputs to English-only for machine readability.
- `pcg_bbox` exposes three modes (compute/gen/point) and supports JSON output when `json_output` is true.

### Fixed
- Addressed a duplicate variable declaration and lambda regression in `src/apps/pcg_bbox.cpp`.

### Notes
- Multi-item operations (e.g., reconstruction across many clusters) stream one JSON object per item to stdout.
- `ai_api.py` prefers parsing tool JSON and falls back to legacy text where necessary.
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

