# Changelog

All notable changes to this project are documented here. This log mirrors the style of `docs/CHANGELOG.md` and focuses on the latest integrations for AI orchestration and structured outputs.

## 1.1.0 / 2025-10-23

### Added
- Web Viewer (deck.gl): New UI toggle "No color (gray)" for the shell layer. When enabled, the viewer renders the shell with a constant gray and does not upload per-vertex color to the GPU (reduces memory/bandwidth).
- Loader: `loadPly(url, { dropColor: boolean })` option to omit COLOR attributes during load.
- Data prep script: `--shellNoColor` flag to write the shell PLY without RGB columns. This reduces file size and speeds up parsing; `label` and `point_id` are still preserved.

### Changed
- Viewer: The shell layer can be rendered colorless to improve interactivity on large rooms (e.g., room_007). The clusters remain colored.
- Docs: `web/pointcloud-viewer/README.md` updated with performance tips and new flags.

### Fixed
- Data prep: Ensured `label` (and `point_id` when present) are preserved in downsampled ASCII PLYs, enabling the "Hide labels 1,3" toggle to work on shell clouds.

### Notes
- For best performance, combine `--shellNoColor` at data generation time with the viewer’s "No color (gray)" toggle. You can also increase shell downsampling (e.g., `--ratioShell 0.05` or add `--voxelShell`) for further gains.

## 1.0.1 / 2025-10-23

### Added
- AI API: Room-level path resolution by room code (e.g., `0-7`). In a single query it returns:
  - The room CSV path
  - The room shell copy (e.g., `0-7-0_shell.ply`)
  - The room shell UOBB (e.g., `0-7-0_shell_uobb.ply`)
- CLI: New `resolve-room <floor-room>` subcommand (e.g., `resolve-room 0-7 --json`). The existing `resolve-room-csv <floor> <room>` now also returns `shell` and `shell_uobb` paths.

### Changed
- pcg_room: For files detected as shell clouds (filename contains "shell"), copy the original input `shell*.ply` to the output directory next to the corresponding `_shell_uobb.ply`. Naming and location example:
  - Input: `data/rooms/Full House/floor_0/room_007/shell_007.ply`
  - Output: `output/Full House/floor_0/room_007/results/shell/shell_007/0-7-0_shell.ply` and `0-7-0_shell_uobb.ply`

### Notes
- Copying the raw shell cloud alongside its computed UOBB improves traceability.
- Documentation: `docs/AI_API.md` updated to describe room-level resolution and the new/extended CLI commands.

## 1.0.0 / 2025-10-22

### Fixed
- AI API: `CLR` head code now auto-builds `pcg_color` on first use (parity with `RCN`/`VOL`/`ARE`/`BBD`). Previously, `CLR` failed with “Missing executable” instead of triggering the first-run build.

### Clarified
- AI API: `VOL` supports auto-reconstruction by default when a mesh is missing. If invoked with an `object_code` that only has a cluster, `VOL` calls `RCN` first and then computes volume. Use `--no-auto-recon` to disable.

### Notes
- Release readiness: verified head codes end-to-end with JSON outputs enabled and CMake Release build. No changes required in `VOL` implementation; behavior verified via `python3 scripts/ai_api.py VOL --object <object_code> --json`.

## 2025-10-20 / 0.9.1

### Fixed / Improved
- AI API: Added automatic CMake build on first use. When a head code (e.g., ARE/VOL/RCN/BBD/CLR) needs a missing executable, the dispatcher configures and builds targets into `build/` automatically.
- New CLI: `python3 scripts/ai_api.py BUILD [--reconfigure]` to build explicitly or force a reconfigure.
- New CLI: `python3 scripts/ai_api.py check-env --json` shows availability of executables and key paths.

### Docs
- Updated `docs/AI_API.md` with installation, first-run (auto-build), manual build options, and troubleshooting.
- Updated README with quick examples for `check-env`, `BUILD`, and first-run behavior.

## 2025-10-20 / 0.9.0

### Added
- `pcg_area` CLI (CGAL): computes mesh surface area and reports closedness. Works for open and closed meshes; emits JSON when `json_output: true`.
- AI API head code `ARE`: resolves mesh by `object_code` or filename, auto-reconstructs if needed, and returns `{ mesh, closed, area }`.

### Changed
- README and AI_API docs updated to include `ARE` and `pcg_area` usage and JSON schemas.

### Notes
- This release finalizes feature additions for the 0.9 series; subsequent 0.9.x will focus on fixes and polish only.

## 2025-10-20 / 0.9.0-alpha.4

### Changed
- Reconstruction output filenames now encode the method:
  - Poisson: `<object_code>_<class>_mesh_possion.ply`
  - AF:      `<object_code>_<class>_mesh_af.ply`
  - Legacy `<object_code>_<class>_mesh.ply` remains recognized by the AI API.
- Volume computation is now performed only for closed meshes. For open meshes, `pcg_volume` skips volume (JSON `volume: null`; text shows a skip note). The AI API maps `null` to `0.0` in its VOL JSON for downstream simplicity.

### Fixed
- AI API `RCN` JSON now includes the `method` field inferred from the mesh filename suffix and returns mesh paths under the correct `results/recon/<stem>/` directory.

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

