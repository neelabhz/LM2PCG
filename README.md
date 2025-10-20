# Indoor Point Cloud Pipeline / 0.9.0-alpha.4

A compact C++17 pipeline for indoor point-cloud processing with PCL (and optional CGAL for reconstruction). It clusters object point clouds, computes upright OBBs, preserves vertex colors end-to-end, and exports standardized results. Utilities include per-cluster reconstruction and a dominant-color CLI.


## Key features
- Color-preserving PLY IO: loads XYZ and XYZRGB; exports colored clusters/UOBBs/meshes
- FEC-style clustering (radius-based) with optional small-cluster filtering
- Upright (Z-up) oriented bounding box per cluster
- Standardized outputs under results with unified filenames
- CSV schema extended with IDs and semantics (object_code/class/etc.)
- Reconstruction per cluster (Poisson with acceptance checks, AF fallback)
 - Standalone tools: pcg_room, pcg_reconstruct, pcg_volume, pcg_color, pcg_bbox
 - AI API for orchestration (scripts/ai_api.py) with structured JSON outputs

## AI API (scripts/ai_api.py) and JSON output

An optional Python-based orchestration layer provides two capabilities:
- Path resolution by object_code, filename, or floor-room
- Operation dispatch via 3-letter head codes: RCN (reconstruct), VOL (volume), CLR (dominant color), BBD (bbox distance)

JSON output mode
- Controlled by `json_output` in `data/configs/default.yaml`. When `true`, C++ apps print structured JSON to stdout; `ai_api` consumes this directly.
- Current default: `true`.

Examples
```bash
# Check environment
python3 scripts/ai_api.py check-env --json

# Reconstruction on a cluster file (absolute or relative path)
python3 scripts/ai_api.py RCN --filename output/full_house/floor_1/room_007/results/filtered_clusters/window_007/1-7-1_window_cluster.ply --json

# Volume on a mesh file
python3 scripts/ai_api.py VOL --filename output/full_house/floor_0/room_001/results/recon/window_001/0-1-3_window_mesh.ply --json

# Dominant color by object_code
python3 scripts/ai_api.py CLR --object 0-7-12 --json

# BBox distance between two objects
python3 scripts/ai_api.py BBD 1-7-2 1-7-3 --json
```

More details and schemas: see `docs/AI_API.md`.


## Quick start
Prerequisites
- CMake ≥ 3.16
- C++17 compiler (Clang, GCC, MSVC)
- PCL ≥ 1.10 (common, io, kdtree)
- CGAL (optional; needed for reconstruction/volume)

macOS with Homebrew
```
brew install pcl cmake
```
Linux with apt (example)
```
sudo apt-get update
sudo apt-get install -y cmake build-essential libpcl-dev
```

Build
```
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
```
Executables are in build/: pcg_room, pcg_reconstruct (if CGAL), pcg_volume (if CGAL), pcg_color, pcg_bbox.


## How to run

### 1) Clustering and per-room processing (pcg_room)
Process a whole site (auto floors → rooms):
```
./build/pcg_room "data/rooms/Full House" "output/Full House"
```
Process a single room directory (the directory contains .ply files):
```
./build/pcg_room "data/rooms/Full House/floor_0/room_001" \
                 "output/Full House/floor_0/room_001"
```
Optional overrides (radius in meters, min_cluster_size in points):
```
./build/pcg_room <input_dir> <output_dir> [radius] [min_cluster_size]
```
Outputs (per room):
- `<room>.csv` with geometry + semantic columns
- `results/filtered_clusters/<stem>/` with colored per-cluster PLYs:
  - `<object_code>_<class>_cluster.ply`
  - `<object_code>_<class>_uobb.ply`
- If reconstruction will be run later, these clusters are the inputs for `pcg_reconstruct`.


## Outputs and naming
Per room under `output/<site>/<floor>/<room>/`:
- CSV summary: `<room>.csv`
  - Columns include: `file, cluster_id, object_code, class, object_id, room_id, floor_id, center_x, center_y, center_z, size_x, size_y, size_z, yaw_rad`
- Results folder: `results/`
  - Colored per-cluster exports under `results/filtered_clusters/<stem>/`
  - Standardized filenames: `<object_code>_<class>_{cluster|uobb|mesh}.ply`
  - Reconstruction (if enabled): `results/recon/<object_stem>/` with one mesh per cluster (accepted Poisson or AF fallback)
    - Mesh filenames include method suffix when applicable: `<object_code>_<class>_mesh_possion.ply` (Poisson) or `<object_code>_<class>_mesh_af.ply` (AF). Legacy `<object_code>_<class>_mesh.ply` remains recognized by ai_api.

Special case: any filename containing "shell" is treated as one cloud (no clustering/recon; UOBB only).


### 2) Reconstruction with CGAL — pcg_reconstruct
```
./build/pcg_reconstruct <input_root_or_room_dir> <output_root_dir>
./build/pcg_reconstruct <single_cluster_ply> <room_output_root>
./build/pcg_reconstruct <room_dir> <room_output_root> <only_substring>
```
Policy: Try Poisson first with acceptance checks (normals fraction, closedness, volume ≤ ratio × convex hull). If it fails, fall back to AF. Exactly one mesh is kept per cluster; filenames encode the chosen method.
Inputs:
- A site/floor/room path under `output/...` containing `results/filtered_clusters`, or
- A single cluster PLY (colored) produced by `pcg_room`.
Outputs:
- `results/recon/<object_stem>/` per cluster with exactly one mesh:
  - `<object_code>_<class>_mesh.ply` (accepted Poisson), or
  - `<object_code>_<class>_mesh.ply` (AF fallback) — same naming, only one kept.
Examples:
```
# Reconstruct everything under a site output root
./build/pcg_reconstruct "output/Full House" "output/Full House"

# Reconstruct a single cluster file
./build/pcg_reconstruct \
  "output/Full House/floor_0/room_001/results/filtered_clusters/door_001/door_001_cluster.ply" \
  "output/Full House/floor_0/room_001"

# Reconstruct only clusters whose filename contains "chair"
./build/pcg_reconstruct "output/Full House/floor_0/room_001" \
                        "output/Full House/floor_0/room_001" chair
```


### 3) Dominant color analysis — pcg_color
```
./build/pcg_color <input_cluster_or_cloud.ply>
```
Method: sample RGB, fit force-K=3 diagonal GMM, filter components by weight and per-channel stddev, convert surviving means to CIELab (D65), merge visually similar colors using ΔE*76 < `color_deltaE_keep` (default 20). If all components are filtered, reports M=0. Output prints M and, for each kept component, weight, mean RGB, and variance.
Examples:
```
# Analyze a cluster produced by pcg_room
./build/pcg_color "output/Full House/floor_0/room_001/results/filtered_clusters/sofa_001/sofa_001_cluster.ply"

# Analyze an arbitrary colored PLY (XYZRGB)
./build/pcg_color "data/rooms/Full House/floor_0/room_001/sofa_001.ply"
```

### 4) Mesh volume and closedness — pcg_volume
If CGAL is available:
```
./build/pcg_volume <mesh_file_1> [mesh_file_2 ...]
```
Outputs a line per file with: path, closed (true/false), and volume. Volume is computed only for closed meshes (e.g., Poisson). For open meshes (e.g., AF), the tool skips volume; JSON shows `"volume": null` and ai_api maps it to `0.0` by default.
Examples:
```
# Check a reconstructed mesh
./build/pcg_volume "output/Full House/floor_0/room_001/results/recon/door_001/door_001_mesh.ply"

# Batch check multiple meshes
./build/pcg_volume output/Full\ House/**/results/recon/**/**_mesh.ply
```

### 5) BBox center, vector, distance — pcg_bbox
`pcg_bbox` offers three modes:

- Compute mode (input two UOBB PLYs exported by `pcg_room`):
```
./build/pcg_bbox <bbox1_uobb.ply> <bbox2_uobb.ply>
```
Output (four lines):
```
center1: cx1, cy1, cz1
center2: cx2, cy2, cz2
vector_1_to_2: dx, dy, dz
distance: d
```

- Generate mode (quickly create a test UOBB):
```
./build/pcg_bbox gen <out.ply> cx cy cz lx ly lz yaw_deg
```
Notes: Z-axis is up; `yaw_deg` is rotation around Z in degrees. Example:
```
./build/pcg_bbox gen output/test/box1.ply 0 0 0 2 1 1 0
./build/pcg_bbox gen output/test/box2.ply 3 4 0 1 1 1 0
./build/pcg_bbox output/test/box1.ply output/test/box2.ply
```

- Point-to-bbox mode (vector from a given point to a bbox center):
```
./build/pcg_bbox point x y z <bbox_uobb.ply>
```
Output (four lines):
```
point: x, y, z
bbox_center: cx, cy, cz
vector_point_to_center: dx, dy, dz
distance: d
```
Example:
```
# Generate a box centered at the origin, then measure from point (1, 2, 0)
./build/pcg_bbox gen output/test/box.ply 0 0 0 2 1 1 0
./build/pcg_bbox point 1 2 0 output/test/box.ply
```

## AI API (chatbot/automation)
The Python layer `scripts/ai_api.py` provides:
- Path resolution by filename/object_code/room
- Head-code dispatcher for core operations

Head codes:
- RCN: reconstruct a cluster to a mesh
- VOL: compute mesh volume and closedness
- CLR: analyze dominant color(s) for a cluster/PLY
- BBD: bbox center vector and distance between two objects

Examples (JSON output recommended):
```
# Environment readiness
python3 scripts/ai_api.py check-env --json

# Reconstruct a specific cluster by object_code
python3 scripts/ai_api.py RCN --object 0-7-12 --json

# Or by direct file path to a cluster
python3 scripts/ai_api.py RCN --filename output/full_house/floor_1/room_007/results/filtered_clusters/window_007/1-7-1_window_cluster.ply --json

# Volume on a mesh
python3 scripts/ai_api.py VOL --filename output/full_house/floor_0/room_001/results/recon/window_001/0-1-3_window_mesh.ply --json

# Dominant color by object_code
python3 scripts/ai_api.py CLR --object 0-7-12 --json

# BBox distance between two objects
python3 scripts/ai_api.py BBD 1-7-2 1-7-3 --json
```

## Structured JSON output
Enable/disable globally via `data/configs/default.yaml`:
```
json_output: true  # when true, tools print JSON to stdout
```
When enabled, C++ apps emit machine-readable JSON. Multi-item operations stream one JSON object per item. The AI API consumes these JSONs and returns consolidated results for the CLI.

## Configuration
Default file: `data/configs/default.yaml`. Key parameters with typical defaults:

### Clustering
- radius (default 0.05): Neighbor radius for clustering in meters.
- min_cluster_size (50): Discard clusters smaller than this many points.
- max_neighbors (150): KD-tree neighbor cap during clustering/featurization.
- filter_factor (0.70): Keep clusters whose average size ≥ filter_factor × global average.
- no_filter_ratio (2.0): If max_cluster_size / min_cluster_size ≤ this ratio, skip size-based filtering entirely.

### Reconstruction — Poisson
- poisson_spacing_neighbors (6): Neighbors for average spacing estimation.
- poisson_normal_neighbors (18): Neighbors for jet normals and MST orientation.
- poisson_min_oriented_fraction (0.3): If oriented normals fraction < threshold, skip Poisson.
- poisson_require_closed (true): Require Poisson output to be a closed mesh.
- poisson_invalid_ratio_vs_hull (1.6): Reject Poisson if mesh volume > ratio × convex hull volume.

### Reconstruction — AF (Advancing Front)
- af_min_points (3): Minimum number of points to attempt AF.
- af_require_closed (false): Require AF output to be closed.

### Color analysis — pcg_color
- color_sample_n (300): Number of RGB points sampled per cluster for analysis.
- color_bic_k_penalty (0): Unused in force-K=3 mode; kept for reference.
- color_min_weight (0.10): Discard GMM components with weight below this threshold.
- color_max_stddev (30.0): Discard components with any channel stddev above this (per channel).
- color_deltaE_keep (20.0): ΔE*76 threshold; ΔE < threshold → merge (drop lower-weight), else keep both.