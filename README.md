# Indoor Point Cloud Pipeline

A C++17 pipeline for indoor point-cloud processing based on PCL. It clusters object point clouds, filters small clusters, computes an upright oriented bounding box (UOBB) per cluster, and writes diagnostics and a per-room CSV.


## Features
- Robust PLY (binary little-endian) loading (xyz required; other vertex properties ignored)
- Clustering via FEC (radius-based) with configurable parameters
- Optional filtering by average cluster size
- Upright (Z-aligned) oriented bounding box (UOBB)
- CSV aggregation per room and UOBB mesh export for diagnostics
- Hierarchical traversal: site → floors → rooms
- Special-case skip: files containing "shell" are not clustered (treated as a single cluster)


## Repository layout (key paths)
- `src/apps/pcg_room.cpp` — main app to process folders
- `include/pcg/*.hpp` — headers (clustering, bbox, io, params, csv)
- `src/geometry|io/*` — module implementations
- `data/rooms/<site>/floor_x/room_y/` — input room folders (PLY files)
- `data/configs/default.yaml` — default parameters
- `output/` — generated results

Example input tree:
```
data/
  rooms/
    client_room/
      floor_0/
        room_001/  # contains *.ply such as shell_001.ply, door_001.ply, ...
        room_002/
      floor_1/
        room_003/
```


## Prerequisites
- CMake ≥ 3.16
- C++17 compiler (Clang, GCC, MSVC)
- PCL ≥ 1.10 with components: common, io, kdtree
- Eigen (typically brought in via PCL)

macOS (Homebrew):
```
brew install pcl cmake
```
Linux (apt-based, example; versions may vary):
```
sudo apt-get update
sudo apt-get install -y cmake build-essential libpcl-dev
```
Windows: Install PCL and CMake, ensure PCL is visible to CMake (PCLConfig.cmake on CMAKE_PREFIX_PATH).


## Build
```
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
```
This produces the executable `pcg_room` in the `build/` directory.


## Run
Two primary modes are supported.

1) Process an entire site (auto traverse floors → rooms):
```
./build/pcg_room data/rooms/client_room output/client_room
```
This will generate outputs under:
```
output/client_room/<floor>/<room>/
  diagnostics/
  <room>.csv
```

2) Process a single room folder (the input directory itself contains .ply files):
```
./build/pcg_room data/rooms/client_room/floor_0/room_001 output/client_room/floor_0/room_001
```

Optional CLI overrides:
```
./build/pcg_room <input_dir> <output_dir> [radius] [min_cluster_size]
```
- `radius` (double): neighbor radius for clustering
- `min_cluster_size` (int): minimum cluster size


## Configuration
Default parameters are read from `data/configs/default.yaml` if present. Keys:
- `radius` (double)
- `min_cluster_size` (int)
- `max_neighbors` (int)
- `filter_factor` (double, e.g., 0.70 means keep clusters >= 70% of average cluster size)
- `no_filter_ratio` (double, e.g., 2.0 means if max/min cluster size ≤ 2.0, skip filtering)

The app tries several relative locations to find `data/configs/default.yaml`. If not found, built-in defaults are used.


## Special handling: "shell" files
Any PLY filename containing the substring "shell" (case-insensitive) is treated as a single cluster:
- No clustering or filtering is performed
- A UOBB is computed directly on the entire cloud

Examples: `shell_001.ply`, `Shell_Room.ply`.


## Outputs
For each processed room, under `output/<site>/<floor>/<room>/` you will find:

- CSV summary: `<room>.csv`
  - Columns: `file, cluster_id, center_x, center_y, center_z, size_x, size_y, size_z, yaw_rad`
  - Units follow the input coordinates (e.g., meters → m, yaw in radians)

- Diagnostics folder: `diagnostics/`
  - Per-cluster UOBB meshes: `diagnostics/filtered_clusters/<stem>/<stem>_cluster_<k>_uobb.ply`
  - Log file: `<room>.log` with clustering statistics and thresholds

Notes on UOBB:
- UOBB is upright (Z-up); yaw is the rotation around Z from +X to the box's first horizontal axis
- Base area/volume are not computed/output by design in the current configuration


## Parameter tuning tips
- Increase `radius` or decrease `min_cluster_size` to merge/simplify clusters
- Decrease `radius` or increase `min_cluster_size` to split clusters / reduce noise
- `filter_factor` controls removal of small clusters relative to the average size
- If cluster sizes are fairly uniform, filtering is skipped when `max/min ≤ no_filter_ratio`
- `max_neighbors` caps neighbor search results for performance


## Troubleshooting
- CMake cannot find PCL:
  - On macOS, run `brew install pcl` and retry
  - Ensure `PCLConfig.cmake` is discoverable by CMake (e.g., set `CMAKE_PREFIX_PATH`)
- IDE shows red squiggles for PCL headers but build succeeds:
  - This usually indicates IntelliSense includePath issues; the CMake build system still compiles fine
- Very large `.ply` files consume lots of memory/time:
  - Run room-by-room; consider downsampling externally if needed
  - Adjust `radius`, `min_cluster_size`, and `max_neighbors` for performance


## License
This repository contains project-specific code that depends on PCL and Eigen. Refer to those projects for their respective licenses.
