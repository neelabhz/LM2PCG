# Indoor Point Cloud Pipeline   1.3.0-alpha.3

[![Release](https://img.shields.io/github/v/release/Jackson513ye/LM2PCG?sort=semver)](https://github.com/Jackson513ye/LM2PCG/releases)

A compact C++17 pipeline for indoor point-cloud processing with PCL and optional CGAL. Clusters object point clouds, computes upright OBBs, preserves vertex colors end-to-end, and exports standardized results.

## Key Features

- **Color-preserving PLY I/O**: Full XYZRGB support for clusters, UOBBs, and meshes
- **FEC-style clustering**: Radius-based with smart filtering
- **Upright bounding boxes**: Optimal OBBs using convex hull + rotating calipers
- **Reconstruction**: Poisson with acceptance checks + AF fallback
- **Analysis tools**: Volume, surface area, dominant color, bbox distance
- **AI API**: Python orchestration with auto-detection ([docs/AI_API.md](docs/AI_API.md))
- **Web viewer**: Interactive 3D visualization with object selection and download ([docs/POINTCLOUD_VIEWER.md](docs/POINTCLOUD_VIEWER.md))

## Quick Start

### Prerequisites

```bash
# macOS
brew install cmake cgal boost eigen pcl

# Linux (Debian/Ubuntu)
sudo apt-get install cmake build-essential libpcl-dev libcgal-dev libeigen3-dev libboost-all-dev
```

### Build

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
```

### Quick Start with Wrapper Script (Recommended)

Use `pcg.sh` for automated building and execution:

```bash
# Auto-builds if needed, then processes the dataset
./pcg.sh "./data/rooms/Full House"
```

**Features**:
- Auto-build on first run or when executable is missing
- Fixed output to `./output/` (auto-cleared before each run)
- Automatic RMS (Room Manifest Summary) generation after processing
- Copies `rooms_manifest.csv` to output directories

### Manual Usage

```bash
# Process a room (clustering + UOBB + CSV)
./build/pcg_room "./data/rooms/Full House"

# Note: Output is now fixed to ./output/ and is auto-cleared before each run
# Room processing no longer accepts single room directories

# Reconstruct clusters to meshes
./build/pcg_reconstruct "output/Full House" "output/Full House"

# Analyze mesh properties
./build/pcg_volume output/Full\ House/**/results/recon/**/*_mesh.ply
./build/pcg_area output/Full\ House/**/results/recon/**/*_mesh.ply
```

## Core Tools

### 1. pcg_room - Clustering and Processing
```bash
./build/pcg_room <input_dir> [radius] [min_cluster_size]
```
Process entire sites with floor/room structure. Output is fixed to `./output/` (auto-cleared). Outputs colored cluster PLYs, UOBBs, and CSV summaries.

### 2. pcg_reconstruct - Mesh Generation
```bash
./build/pcg_reconstruct <input_root_or_room_dir> <output_root_dir>
```
Converts clusters to meshes using Poisson (with validation) or AF fallback.

### 3. pcg_volume - Volume Analysis
```bash
./build/pcg_volume <mesh_file> [mesh_file_2 ...]
```
Computes volume for closed meshes.

### 4. pcg_area - Surface Area Analysis
```bash
./build/pcg_area <mesh_file> [mesh_file_2 ...]
```
Computes surface area for both open and closed meshes.

### 5. pcg_color - Dominant Color
```bash
./build/pcg_color <cluster.ply>
```
Analyzes dominant colors using GMM and perceptual color distance (ΔE*76).

### 6. pcg_bbox - Bounding Box Tools
```bash
# Compute distance between two objects
./build/pcg_bbox <bbox1_uobb.ply> <bbox2_uobb.ply>

# Generate test UOBB
./build/pcg_bbox gen <out.ply> cx cy cz lx ly lz yaw_deg

# Point to bbox distance
./build/pcg_bbox point x y z <bbox_uobb.ply>
```

## AI API

Python orchestration layer with intelligent auto-detection. **[Complete guide →](docs/AI_API.md)**

```bash
# Direct operations with auto-detection
python3 scripts/ai_api.py VOL 0-7-12
python3 scripts/ai_api.py ARE 0-7-12
python3 scripts/ai_api.py CLR 0-7-12
python3 scripts/ai_api.py BBD 1-7-2 1-7-3
python3 scripts/ai_api.py RMS

# Visualization (auto-clean-all, auto-serve)
python3 scripts/ai_api.py VIS 0-7        # Visualize room
python3 scripts/ai_api.py VIS 0-7-12     # Visualize single object
python3 scripts/ai_api.py VIS 0-1,0-5,0-7 # Multi-rooms
```

**Operations**: `RCN` (reconstruct), `VOL` (volume), `ARE` (area), `CLR` (color), `BBD` (distance), `RMS` (room summary), `VIS` (visualization)

**Key Features (v1.4.0)**:
- Auto-detection of rooms and objects
- Relative path handling for portability
- VIS auto-clean-all and auto-serve by default
- Multi-rooms visualization support

## Web Visualization

Interactive 3D viewer with object selection, confirmation, and download. **[Complete guide →](docs/POINTCLOUD_VIEWER.md)**

```bash
cd web/pointcloud-viewer
npm install

# Quick start (via AI API - recommended)
python3 ../../scripts/ai_api.py VIS 0-7    # Auto-clean, auto-serve

# Or manual npm commands
npm run visualize -- --mode room --room 0-7 --name room_007
```

**4 Modes**: `room`, `clusters`, `multi-rooms`, `room-with-objects`  

**Key Features (v1.4.0)**: 
- **Interactive Object Selection**: Click to highlight, confirm selections
- **One-Click Download**: Download source PLY files via integrated API
- **Non-pickable UOBB**: Bounding boxes don't block object selection
- **Multi-rooms Support**: Unified visualization with 1% shell downsampling
- **Auto-serve**: Servers start automatically with VIS command
- Per-object visibility toggles with semantic naming
- Automatic UOBB generation using convex hull + rotating calipers
- 10M+ points @ 60 FPS performance

**Server Management:**
```bash
# From web/pointcloud-viewer
./start_dev.sh    # Start frontend (5173) + API server (8090)
./stop_dev.sh     # Stop both servers
```

## Configuration

Default settings in `data/configs/default.yaml`:

**Clustering**:
- `radius` (0.05): Neighbor radius in meters
- `min_cluster_size` (50): Minimum points per cluster
- `filter_factor` (0.70): Size-based filter threshold

**Reconstruction**:
- `poisson_min_oriented_fraction` (0.3): Normal orientation threshold
- `poisson_require_closed` (true): Require closed meshes
- `af_require_closed` (false): AF fallback policy

**Color Analysis**:
- `color_sample_n` (300): RGB sample size
- `color_deltaE_keep` (20.0): Perceptual color merge threshold

**Viewer (v1.4.0)**:
- `viewer_downsample_ratio` (0.2): Cluster downsampling rate (20%)
- `viewer_downsample_ratio_shell` (0.05): Shell downsampling rate (5%)
  - Note: RMS multi-rooms mode uses 1% (0.01) for performance
- `viewer_point_size` (3): Default point size for rendering
- `viewer_uobb_opacity` (0.3): UOBB transparency (0.0-1.0)
- `viewer_uobb_color` ([30, 144, 255]): UOBB color (RGB)

**JSON Output**:
```yaml
json_output: true  # Enable structured JSON output
```

## Output Structure

```
output/<site>/<floor>/<room>/
├── <room>.csv                          # Geometry summary
└── results/
    ├── filtered_clusters/<stem>/
    │   ├── <object_code>_<class>_cluster.ply
    │   └── <object_code>_<class>_uobb.ply
    └── recon/<object_stem>/
        └── <object_code>_<class>_mesh.ply
```

**Object codes**: `floor-room-object` (e.g., `0-7-12` = floor 0, room 7, object 12)

## Documentation

- **[AI API](docs/AI_API.md)** - Python orchestration and automation
- **[Point Cloud Viewer](docs/POINTCLOUD_VIEWER.md)** - Web visualization system
- **[Changelog](docs/CHANGELOG.md)** - Version history
