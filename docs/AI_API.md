# AI API: path resolution and head-code dispatcher

An AI-friendly control surface for the LM2PCG pipeline. It resolves paths (by object/room/filename) and dispatches short “head codes” to the compiled C++ tools with the right arguments.

## Table of Contents

- [Overview](#overview)
- [Capabilities](#capabilities)
- [Quick Start](#quick-start)
- [Prerequisites](#prerequisites)
- [Building the Project](#building-the-project)
- [Conventions](#conventions)
  - [Object/Room codes](#objectroom-codes)
  - [Directory layout](#directory-layout)
  - [File naming](#file-naming)
- [Path Resolution](#path-resolution)
  - [Resolve by filename](#resolve-by-filename)
  - [Resolve by room](#resolve-by-room)
  - [Resolve by object](#resolve-by-object)
- [Head Codes (operations)](#head-codes-operations)
  - [RCN — Reconstruct](#rcn--reconstruct)
  - [VOL — Volume](#vol--volume)
  - [ARE — Surface Area](#are--surface-area)
  - [CLR — Dominant Color](#clr--dominant-color)
  - [BBD — BBox Distance (pair)](#bbd--bbox-distance-pair)
- [Python API](#python-api)
- [JSON output](#json-output)
- [Related docs](#related-docs)

---

## Overview

The AI API is a lightweight Python layer at `scripts/ai_api.py` that:

- Scans `output/` once to build indices (by filename, room, object_code)
- Resolves inputs like `0-7-12_couch_cluster.ply`, `0-7`, or `0-7-12`
- Dispatches short, memorable operations (RCN/VOL/ARE/CLR/BBD/RMS) to the C++ apps
- Provides helpful error messages when executables are missing

It's designed for automation agents and local scripting. All results are printed to stdout and can be emitted as JSON.

## Capabilities

- Filename-only lookup anywhere under `output/`
- Room-level resolution: CSV + shell copies + shell UOBB
- Object-level resolution: clusters, UOBBs, recon meshes, and inferred `room_dir`
- Head code dispatcher that wraps `pcg_*` binaries with correct arguments
- Room manifest summary (RMS) operation for floor/room statistics

## Quick Start

```bash
# Check environment and tool availability
python3 scripts/ai_api.py check-env --json

# Resolve assets for one object
python3 scripts/ai_api.py resolve-object 0-7-12 --json

# Room manifest summary
python3 scripts/ai_api.py RMS --json

# Reconstruct (RCN) and compute area (ARE)
python3 scripts/ai_api.py RCN --object 0-7-12 --json
python3 scripts/ai_api.py ARE --object 0-7-12 --json

# BBox distance between two objects (pair op)
python3 scripts/ai_api.py BBD 0-7-12 0-7-14 --json
```

**Note**: The API no longer auto-builds. If executables are missing, you'll receive a helpful error message with build instructions. Use `./pcg.sh` or manually build the project first.

## Prerequisites

- Python 3.7+
- C++ toolchain for the C++ apps: C++17 compiler + CMake
- Libraries: CGAL, Boost, Eigen3, PCL (and VTK/libpng/libjpeg/libtiff/zlib)

macOS hint (Homebrew):
```bash
brew install cmake cgal boost eigen pcl
```

```

Linux: use your distro packages for the equivalents.

## Building the Project

The AI API requires C++ executables to be built beforehand. The easiest way is to use the wrapper script:

```bash
# Auto-builds if needed, then processes your data
./pcg.sh "./data/rooms/Full House"
```

Alternatively, build manually:

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
```

Check executable availability:

```bash
python3 scripts/ai_api.py check-env --json
```

If executables are missing, the API will provide helpful error messages with build instructions.

## Conventions

## Install & Build

The API auto-builds binaries into `build/` on first use, or you can build explicitly.

### Auto-build (first run)

When a head code needs a missing executable, the API configures and builds Release targets automatically.

```bash
# Example: compute surface area; triggers configure+build on first run
python3 scripts/ai_api.py ARE --object 0-7-12 --json
```

Check availability anytime:

```bash
python3 scripts/ai_api.py check-env --json
```

### Manual build

- Using the API itself:

```bash
python3 scripts/ai_api.py BUILD                 # configure (if needed) + build
python3 scripts/ai_api.py BUILD --reconfigure   # force reconfigure
```

- Using VS Code tasks (Terminal → Run Task…):
  - “Configure and build (Release)”
  - “Build CMake project (Release)”

Troubleshooting build:
- If libraries are missing, install them and re-run.
- If you switch compilers or upgrade libs, use `BUILD --reconfigure` or delete `build/`.

## Conventions

### Object/Room codes

- Room code: `<floor>-<room>` → e.g., `0-7`
- Object code: `<floor>-<room>-<object>` → e.g., `0-7-12`

### Directory layout

```
output/<site>/
└── floor_<f>/
    └── room_<rrr>/
        ├── <room_dirname>.csv
        └── results/
            ├── shell/
            │   └── shell_<rrr>/
            │       ├── 0-7-0_shell.ply
            │       └── 0-7-0_shell_uobb.ply
            ├── filtered_clusters/<stem>/
            │   ├── 0-7-12_<class>_cluster.ply
            │   └── 0-7-12_<class>_uobb.ply
            └── recon/<stem>/
                ├── 0-7-12_<class>_mesh.ply        (legacy)
                ├── 0-7-12_<class>_mesh_possion.ply (Poisson)
                └── 0-7-12_<class>_mesh_af.ply      (AF)
```

### File naming

- Cluster: `<object_code>_<class>_cluster.ply`
- UOBB: `<object_code>_<class>_uobb.ply`
- Mesh: `<object_code>_<class>_mesh[_possion|_af].ply`
- Room shell copy: `0-7-0_shell.ply` and `0-7-0_shell_uobb.ply`

## Path Resolution

The API scans `output/` once at initialization to build three types of indices for fast lookups:

1. **By filename**: Direct filename → absolute path(s) mapping
2. **By room code**: `(floor_id, room_id)` → CSV files, shell copies, shell UOBBs
3. **By object code**: `<floor>-<room>-<object>` → clusters, UOBBs, meshes, inferred room directory

### How Path Indexing Works

The `PathIndex` class recursively walks the `output/` directory tree and categorizes files based on naming conventions:

#### File Naming Patterns

1. **Object-level assets** (3+ parts):
   - Format: `<object_code>_<class>_<kind>.ply`
   - Examples:
     - `0-7-12_couch_cluster.ply` → cluster
     - `0-7-12_couch_uobb.ply` → UOBB
     - `0-7-12_couch_mesh.ply` → mesh (legacy)
     - `0-7-12_couch_mesh_possion.ply` → mesh (Poisson method)
     - `0-7-12_couch_mesh_af.ply` → mesh (Advancing Front method)

2. **Room-level shell** (2 parts):
   - Format: `<object_code>_shell.ply`
   - Example: `0-7-0_shell.ply` (object_id=0 indicates room-level)
   - Shell UOBB: `0-7-0_shell_uobb.ply`

3. **CSV metadata**:
   - Located in `room_XXX/` directories
   - Named as `<room_dirname>.csv`

#### Parsing Logic

- **Object code extraction**: First underscore-separated part (e.g., `0-7-12` from `0-7-12_couch_cluster.ply`)
- **Kind detection**:
  - Last part is `cluster`, `uobb`, or `mesh` → use as-is
  - Second-to-last is `mesh` (for method suffix) → kind = `mesh`
- **Room inference**: Walks up directory tree to find parent `room_XXX/` directory
- **Shell detection**:
  - 2-part name ending in `_shell` → room shell copy
  - 3-part name ending in `_shell_uobb` → room shell UOBB

### Resolve by filename

Returns all absolute paths matching a given filename (useful when the same file exists in multiple output runs).

```bash
python3 scripts/ai_api.py resolve-filename 0-7-12_couch_cluster.ply --json
```

Output:
```json
{
  "matches": [
    "/abs/path/output/Full House/floor_0/room_007/results/filtered_clusters/couch_007/0-7-12_couch_cluster.ply",
    "/abs/path/output/Full House (copy)/floor_0/room_007/results/filtered_clusters/couch_007/0-7-12_couch_cluster.ply"
  ]
}
```

**Ambiguity handling**: When multiple matches exist (e.g., mirrored runs, backups), all candidates are returned. Some head codes support `--only-substr` to narrow results by path substring.

### Resolve by room

Returns room-level assets: CSV files, shell point clouds, and shell UOBBs.

```bash
python3 scripts/ai_api.py resolve-room 0-7 --json
```

Output:
```json
{
  "floor": 0,
  "room": 7,
  "csv": ["/abs/path/.../room_007/room_007.csv"],
  "shell": ["/abs/path/.../shell_007/0-7-0_shell.ply"],
  "shell_uobb": ["/abs/path/.../shell_007/0-7-0_shell_uobb.ply"]
}
```

Alternative syntax using separate integers:
```bash
python3 scripts/ai_api.py resolve-room-csv 0 7 --json
```

**Note**: The object_id in shell filenames is always `0` (e.g., `0-7-0_shell.ply` for room `0-7`), indicating room-level data rather than a specific object.

### Resolve by object

Returns all assets associated with an object code: clusters, UOBBs, meshes, and the inferred parent room directory.

```bash
python3 scripts/ai_api.py resolve-object 0-7-12 --json
```

Output:
```json
{
  "object_code": "0-7-12",
  "clusters": ["/abs/path/.../0-7-12_couch_cluster.ply"],
  "uobbs": ["/abs/path/.../0-7-12_couch_uobb.ply"],
  "meshes": [
    "/abs/path/.../0-7-12_couch_mesh_possion.ply",
    "/abs/path/.../0-7-12_couch_mesh_af.ply"
  ],
  "room_dir": "/abs/path/.../room_007"
}
```

**Room directory inference**: The API walks up to 8 parent directories from each asset file to locate the nearest `room_XXX/` directory. This is essential for operations requiring room context (e.g., reconstruction needs room-level configuration).

**Asset grouping**: All files sharing the same object code are grouped together, regardless of their physical location in subdirectories like `filtered_clusters/<stem>/` or `recon/<stem>/`.

## Head Codes (operations)

All head codes accept `--json` for structured output where applicable. Missing executables trigger an auto-build.

### RCN — Reconstruct

Reconstruct a mesh from a cluster PLY.

Usage:

```bash
python3 scripts/ai_api.py RCN --object 0-7-12 [--only-substr <hint>] [--json]
python3 scripts/ai_api.py RCN --filename 0-7-12_couch_cluster.ply [--only-substr <hint>] [--json]
```

Behavior:
- Resolves the cluster and its `room_dir`, runs `pcg_reconstruct <cluster> <room_dir>`
- Returns the generated mesh path; method suffix differentiates Poisson/AF

Output with `--json` (from the API):

```json
{ "mesh": "/abs/path/..._mesh_possion.ply", "method": "poisson" }
```

Errors:
- Missing cluster for the object/filename
- Unable to infer `room_dir`

### VOL — Volume

Compute mesh volume and closedness.

Usage:

```bash
python3 scripts/ai_api.py VOL --object 0-7-12 [--no-auto-recon] [--json]
python3 scripts/ai_api.py VOL --filename 0-7-12_couch_mesh.ply [--no-auto-recon] [--json]
```

Behavior:
- If a mesh doesn’t exist and the input is a cluster, auto-reconstruct unless `--no-auto-recon`
- Calls `pcg_volume <mesh>` and parses JSON or legacy text

Output:

```json
{ "mesh": "/abs/path/..._mesh_af.ply", "closed": true, "volume": 0.0123 }
```

Errors:
- Mesh not found and `--no-auto-recon` set
- Volume not parsable from tool output

### ARE — Surface Area

Compute mesh surface area and closedness.

Usage:

```bash
python3 scripts/ai_api.py ARE --object 0-7-12 [--no-auto-recon] [--json]
python3 scripts/ai_api.py ARE --filename 0-7-12_couch_mesh.ply [--no-auto-recon] [--json]
```

Output:

```json
{ "mesh": "/abs/path/..._mesh_possion.ply", "closed": false, "area": 2.345 }
```

### CLR — Dominant Color

Run color GMM analysis on a PLY (prefers cluster when given an object code).

Usage:

```bash
python3 scripts/ai_api.py CLR --object 0-7-12 --json
python3 scripts/ai_api.py CLR --filename some_cluster_or_mesh.ply --json
```

Output (from tool JSON when available):

```json
{
  "file": ".../0-7-12_couch_cluster.ply",
  "M": 3,
  "components": [
    { "weight": 0.52, "mean": [r,g,b], "var": [vr,vg,vb] },
    { "weight": 0.34, "mean": [r,g,b], "var": [vr,vg,vb] },
    { "weight": 0.14, "mean": [r,g,b], "var": [vr,vg,vb] }
  ]
}
```

When JSON is not enabled in C++ tools, the API returns a light parse or raw text.

### BBD — BBox Distance (pair)

Distance and vector between two UOBB centers.

Usage:

```bash
python3 scripts/ai_api.py BBD 0-7-12 0-7-14 --json
```

Output:

```json
{ "distance": 1.234, "vector_1_to_2": { "x": 0.1, "y": -0.2, "z": 0.05 } }
```

Errors:
- Either object lacks a UOBB PLY
- Tool output not parsable

## Python API

```python
from scripts.ai_api import Dispatcher

d = Dispatcher()

# Path resolution
print(d.index.find_by_filename("0-7-12_couch_cluster.ply"))
print(d.index.find_csv(0, 7))
print(d.index.find_assets("0-7-12"))

# Operations
mesh_path = d.op_RCN(object_code="0-7-12")
mesh, vol, closed = d.op_VOL(object_code="0-7-12")
mesh, area, closed = d.op_ARE(object_code="0-7-12")
color = d.op_CLR(object_code="0-7-12")
dist, vec = d.op_BBD("0-7-12", "0-7-14")
```

## JSON output

Global control: `data/configs/default.yaml` → `json_output: true` (default). When enabled, C++ tools emit structured JSON to stdout (one object per processed input).

API `--json` flag: forces structured output for the Python CLI regardless of C++ mode (the API prefers consuming tool JSON and falls back to text parsing when needed).

Per-tool JSON shapes (tool output):

- `pcg_volume` → `{ "file": "<path>", "closed": true|false, "volume": <float> }`
- `pcg_area`   → `{ "file": "<path>", "closed": true|false, "area": <float> }`
- `pcg_bbox`   →
  - gen: `{ "mode": "gen", "status": "ok|failed", "file": "<out.ply>" }`
  - pair: `{ "mode": "pair", "center1": {x,y,z}, "center2": {x,y,z}, "vector_1_to_2": {x,y,z}, "distance": <float> }`
    - errors: `{ "mode": "pair", "status": "not_found|read_failed|empty", ... }`
- `pcg_color`  → `{ "file": "<ply>", "M": <int>, "components": [ { "weight": <float>, "mean": [r,g,b], "var": [vr,vg,vb] }, ... ] }`
  - empty: `{ "file": "<ply>", "M": 0 }`
- `pcg_reconstruct` → one JSON object per input cluster:
  - success (Poisson): `{ "file": "<cluster.ply>", "method": "poisson", "mesh": "<mesh.ply>", "status": "ok" }`
  - success (AF):      `{ "file": "<cluster.ply>", "method": "af",      "mesh": "<mesh.ply>", "status": "ok" }`
  - read failure:      `{ "file": "<cluster.ply>", "status": "read_failed" }`
  - failed:            `{ "file": "<cluster.ply>", "status": "failed" }`

Notes:
- Tools that process multiple inputs emit newline-delimited objects (streaming), not an array.

## Related docs

- [Point Cloud Viewer](./POINTCLOUD_VIEWER.md)
- [Project Changelog](./CHANGELOG.md)
- [Root README](../README.md)

