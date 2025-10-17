# Indoor Point Cloud Pipeline

A C++17 pipeline for indoor point-cloud processing based on PCL (and CGAL for reconstruction). It clusters object point clouds, optionally filters small clusters, computes upright oriented bounding boxes (UOBB), and can reconstruct meshes per cluster with quality checks. It also provides standalone mesh volume computation utilities.


## Features
- Robust PLY (binary little-endian) loading (xyz required; other vertex properties ignored)
- Clustering via FEC (radius-based) with configurable parameters
- Optional filtering by average cluster size (configurable skip when sizes uniform)
- Upright (Z-aligned) oriented bounding box (UOBB) per cluster
- CSV aggregation per room and UOBB mesh export for diagnostics
- Hierarchical traversal: site → floors → rooms
- Special-case skip: files containing "shell" are not clustered or reconstructed (treated as a single cloud for UOBB only)
- Reconstruction per cluster (requires CGAL):
  - Poisson reconstruction with robust acceptance checks
    - Requires sufficient oriented normals fraction
    - Requires closed output (configurable)
    - Volume sanity: Poisson mesh volume must be ≤ ratio × convex hull volume of input points
  - AF (Advancing Front) fallback only when Poisson is invalid or fails
  - Exactly one output mesh per cluster: prefer accepted Poisson, otherwise AF
- Volume utilities:
  - Standalone CLI to report mesh volume and closedness
  - Library functions to compute mesh volume and point-cloud convex hull volume


## Repository layout (key paths)
- `src/apps/pcg_room.cpp` — main app to process folders
- `src/apps/pcg_reconstruct.cpp` — per-cluster mesh reconstruction (Poisson + AF, with validation)
- `src/apps/pcg_volume.cpp` — compute mesh volume (and closedness) from a mesh file
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
- CGAL (optional but required for reconstruction and volume CLI)

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
This produces executables in `build/`:
- `pcg_room` (always)
- `pcg_reconstruct` (if CGAL is found)
- `pcg_volume` (if CGAL is found)


## Run
Two primary modes are supported for preprocessing and clustering.

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


### Reconstruction (Poisson + AF)
If CGAL is available, you can reconstruct meshes per cluster using the filtered clusters produced by `pcg_room`.

Usage:
```
./build/pcg_reconstruct <input_root_or_room_dir> <output_root_dir>
```

Notes:
- Input can be a site root (e.g., `output/client_room`), a floor, or a single room directory that contains `diagnostics/filtered_clusters`.
- For每个 cluster，程序会先尝试 Poisson：
  - 满足定向法线占比阈值
  - （可选）要求输出网格闭合
  - 网格体积需 ≤ 配置阈值 × 点云凸包体积
  - 若通过，输出 `<stem>_poisson.ply`，不会再跑 AF
- 若 Poisson 失败/无效，则回退 AF，输出 `<stem>_af.ply`
- 输出位于：`<room>/diagnostics/recon/<object_stem>/`


### Mesh volume computation
如果 CGAL 可用，使用体积计算 CLI：
```
./build/pcg_volume <mesh_file_1> [mesh_file_2 ...]
```
输出包含：
- 路径
- closed: true/false（网格是否闭合）
- volume: 数值（单位与坐标一致）

开发者接口（库函数）：
- 头文件：`include/pcg/geometry/volume.hpp`
  - `double mesh_signed_volume(const pcg::geom::Mesh& mesh);`
    - 计算网格体积（对闭合网格有效；异常/不适用返回 0.0）
  - `double convex_hull_volume(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);`
    - 计算点云三维凸包体积（点数不足或退化返回 0.0）


## Configuration
Default parameters are read from `data/configs/default.yaml` if present.

Pipeline parameters:
- `radius` (double): clustering neighborhood radius (meters)
- `min_cluster_size` (int): discard clusters smaller than this many points
- `max_neighbors` (int): kd-tree neighbor cap for clustering/featurization
- `filter_factor` (double): keep clusters whose average size ≥ `filter_factor` × global average
- `no_filter_ratio` (double): if `max_cluster_size / min_cluster_size` ≤ this ratio, skip filtering entirely

Reconstruction parameters (Poisson):
- `poisson_spacing_neighbors` (int): neighbors for average spacing
- `poisson_normal_neighbors` (int): neighbors for jet normals & MST orientation
- `poisson_min_oriented_fraction` (double): oriented normals fraction threshold；不足则跳过 Poisson
- `poisson_require_closed` (bool): 是否要求 Poisson 输出闭合
- `poisson_invalid_ratio_vs_hull` (double): 体积上限系数（Poisson 体积 ≤ ratio × 凸包体积）

Reconstruction parameters (AF):
- `af_min_points` (int): 尝试 AF 的最小点数
- `af_require_closed` (bool): 是否要求 AF 输出闭合

The apps try several relative locations to find `data/configs/default.yaml`. If not found, built-in defaults are used.


## Special handling: "shell" files
Any PLY filename containing the substring "shell" (case-insensitive) is treated as a single cluster:
- No clustering or filtering is performed
- A UOBB is computed directly on the entire cloud
- No reconstruction is performed for shell clouds

Examples: `shell_001.ply`, `Shell_Room.ply`.


## Outputs
For each processed room, under `output/<site>/<floor>/<room>/` you will find:

- CSV summary: `<room>.csv`
  - Columns: `file, cluster_id, center_x, center_y, center_z, size_x, size_y, size_z, yaw_rad`
  - Units follow the input coordinates (e.g., meters → m, yaw in radians)

- Diagnostics folder: `diagnostics/`
  - Per-cluster UOBB meshes: `diagnostics/filtered_clusters/<stem>/<stem>_cluster_<k>_uobb.ply`
  - Log file: `<room>.log` with clustering statistics and thresholds
  - If reconstruction was run: per-cluster mesh under `diagnostics/recon/<stem>/`
    - Accepted Poisson: `<stem>_cluster_<k>_poisson.ply`
    - AF fallback: `<stem>_cluster_<k>_af.ply`

Reconstruction policy summary:
- Prefer Poisson when it passes all checks (normals fraction、闭合性、体积对比凸包)
- Only run AF when Poisson is invalid or throws

Notes on UOBB:
- UOBB is upright (Z-up); yaw is the rotation around Z from +X to the box's first horizontal axis
- Base area/volume are not computed/output by design in the current configuration


## Parameter tuning tips
- Increase `radius` or decrease `min_cluster_size` to merge/simplify clusters
- Decrease `radius` or increase `min_cluster_size` to split clusters / reduce noise
- `filter_factor` controls removal of small clusters relative to the average size
- If cluster sizes are fairly uniform, filtering is skipped when `max/min ≤ no_filter_ratio`
- `max_neighbors` caps neighbor search results for performance

Reconstruction tips:
- 若 Poisson 通过率偏低，可尝试：
  - 调高/调低 `poisson_normal_neighbors` 或 `poisson_spacing_neighbors`
  - 略微降低 `poisson_min_oriented_fraction`
  - 慎重提高 `poisson_invalid_ratio_vs_hull`（过大可能引入劣质网格）
- AF 默认不强制闭合（`af_require_closed: false`），以提高回退可用性；如需闭合结果可设为 true（可能导致无结果）。


## Troubleshooting
- CMake cannot find PCL:
  - On macOS, run `brew install pcl` and retry
  - Ensure `PCLConfig.cmake` is discoverable by CMake (e.g., set `CMAKE_PREFIX_PATH`)
- IDE shows red squiggles for PCL headers but build succeeds:
  - This usually indicates IntelliSense includePath issues; the CMake build system still compiles fine
- Very large `.ply` files consume lots of memory/time:
  - Run room-by-room; consider downsampling externally if needed
  - Adjust `radius`, `min_cluster_size`, and `max_neighbors` for performance
- PLY 解析器提示 camera 属性未处理：
  - 这些与点坐标无关，已在重建程序中降低 PCL 日志级别，不影响结果
- CGAL Poisson 报错或体积超阈值：
  - 程序会自动回退到 AF；可按“Reconstruction tips”调整参数


## License
This repository contains project-specific code that depends on PCL and Eigen. Refer to those projects for their respective licenses.
