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

macOS (Homebrew)
```
brew install pcl cmake
```
Linux (apt example)
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

Run (site or single room)
```
# Process a whole site (auto floors → rooms)
./build/pcg_room data/rooms/client_room output/client_room

# Or process a single room directory containing .ply files
./build/pcg_room data/rooms/client_room/floor_0/room_001 \
                 output/client_room/floor_0/room_001
```
Optional overrides
```
./build/pcg_room <input_dir> <output_dir> [radius] [min_cluster_size]
```


## Outputs and naming
Per room under `output/<site>/<floor>/<room>/`:
- CSV summary: `<room>.csv`
  - Columns include: `file, cluster_id, object_code, class, object_id, room_id, floor_id, center_x, center_y, center_z, size_x, size_y, size_z, yaw_rad`
- Results folder: `results/`
  - Colored per-cluster exports under `results/filtered_clusters/<stem>/`
  - Standardized filenames: `<object_code>_<class>_{cluster|uobb|mesh}.ply`
  - Reconstruction (if enabled): `results/recon/<object_stem>/` with one mesh per cluster (accepted Poisson or AF fallback)

Special case: any filename containing "shell" is treated as one cloud (no clustering/recon; UOBB only).


## Reconstruction (optional, CGAL)
```
./build/pcg_reconstruct <input_root_or_room_dir> <output_root_dir>
./build/pcg_reconstruct <single_cluster_ply> <room_output_root>
./build/pcg_reconstruct <room_dir> <room_output_root> <only_substring>
```
Policy: Try Poisson first with acceptance checks (normals fraction, closedness, volume ≤ ratio × convex hull). If it fails, fall back to AF. Exactly one mesh is kept per cluster.


## Dominant color CLI
```
./build/pcg_color <input_cluster_or_cloud.ply>
```
Method: sample RGB, fit force-K=3 diagonal GMM, filter components by weight and per-channel stddev, convert surviving means to CIELab (D65), merge visually similar colors using ΔE*76 < `color_deltaE_keep` (default 20). If all components are filtered, reports M=0. Output prints M and, for each kept component, weight, mean RGB, and variance.


## Configuration (essentials)
Default file: `data/configs/default.yaml`.
- Clustering: `radius`, `min_cluster_size`, `max_neighbors`, `filter_factor`, `no_filter_ratio`
- Reconstruction (Poisson): `poisson_*` thresholds; AF: `af_*`
- Color analysis: `color_sample_n`, `color_min_weight`, `color_max_stddev`, `color_deltaE_keep`


## Troubleshooting (quick)
- PCL not found: install via Homebrew/apt and ensure PCLConfig.cmake on CMAKE_PREFIX_PATH
- Large .ply: process room-by-room; tune `radius`/`min_cluster_size`/`max_neighbors`
- Poisson rejected often: tweak `poisson_normal_neighbors`/`poisson_spacing_neighbors`, thresholds, or rely on AF


## Changelog
See docs/CHANGELOG.md for recent changes.


## License
Project code depends on PCL/Eigen/CGAL. See their licenses for those components.
