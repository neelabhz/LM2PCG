# Indoor Point Cloud Pipeline

A compact C++17 pipeline for indoor point-cloud processing with PCL (and optional CGAL for reconstruction). It clusters object point clouds, computes upright OBBs, preserves vertex colors end-to-end, and exports standardized results. Utilities include per-cluster reconstruction and a dominant-color CLI.


## Key features
- Color-preserving PLY IO: loads XYZ and XYZRGB; exports colored clusters/UOBBs/meshes
- FEC-style clustering (radius-based) with optional small-cluster filtering
- Upright (Z-up) oriented bounding box per cluster
- Standardized outputs under results with unified filenames
- CSV schema extended with IDs and semantics (object_code/class/etc.)
- Reconstruction per cluster (Poisson with acceptance checks, AF fallback)
- Standalone tools: pcg_room, pcg_reconstruct, pcg_volume, pcg_color


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
Executables are in build/: pcg_room, pcg_reconstruct (if CGAL), pcg_volume (if CGAL), pcg_color.


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

Special case: any filename containing "shell" is treated as one cloud (no clustering/recon; UOBB only).


### 2) Reconstruction with CGAL — pcg_reconstruct
```
./build/pcg_reconstruct <input_root_or_room_dir> <output_root_dir>
./build/pcg_reconstruct <single_cluster_ply> <room_output_root>
./build/pcg_reconstruct <room_dir> <room_output_root> <only_substring>
```
Policy: Try Poisson first with acceptance checks (normals fraction, closedness, volume ≤ ratio × convex hull). If it fails, fall back to AF. Exactly one mesh is kept per cluster.
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
Outputs a line per file with: path, closed (true/false), and volume. Useful for validating reconstructed meshes and for downstream QC.
Examples:
```
# Check a reconstructed mesh
./build/pcg_volume "output/Full House/floor_0/room_001/results/recon/door_001/door_001_mesh.ply"

# Batch check multiple meshes
./build/pcg_volume output/Full\ House/**/results/recon/**/**_mesh.ply
```

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