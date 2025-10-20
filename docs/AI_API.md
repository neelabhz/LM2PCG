# AI API layer: path resolution + head-code dispatcher

This document sketches a unified, AI-friendly control surface for the pipeline. It provides:

  - By `object_code` (e.g., `0-7-12`) → find cluster/uobb/mesh files and the room directory
  - By filename only (e.g., `0-7-12_couch_cluster.ply`) → resolve absolute path under `output/`
  - By floor-room (e.g., `0-7`) → find the room `.csv`
  - Short 3-letter head codes that call into existing binaries with the correct inputs

The implementation is a lightweight Python script at `scripts/ai_api.py` that scans the `output/` tree once to build indices, then resolves and dispatches operations.

## Installation & build

The Python API orchestrates a set of C++ apps (pcg_reconstruct, pcg_volume, pcg_area, pcg_bbox, pcg_color, pcg_room). These must be compiled locally. You have two options:

- Easiest: let the API auto-build on first use (recommended)
- Manual: trigger a build explicitly

Prerequisites (system):
- A C++17 compiler (clang++/AppleClang on macOS, gcc/clang on Linux)
- CMake (Release builds are used)
- CGAL, Boost, Eigen3
- PCL (and dependencies such as VTK, libpng, libjpeg, libtiff, zlib, etc.)

Tip (macOS): install dependencies with Homebrew, e.g., `brew install cmake cgal boost eigen pcl`. On Linux, use your distribution’s package manager for equivalent packages.

### First run (auto-build)

When a head code needs a missing executable, the dispatcher automatically configures and builds targets into `build/`. Nothing else is required beyond installed prerequisites.

Example:

```bash
# Compute surface area; on first run this configures + builds, then runs
python3 scripts/ai_api.py ARE --object 0-7-12 --json
```

Check availability anytime:

```bash
python3 scripts/ai_api.py check-env --json
```

### Manual build options

If you prefer to build explicitly:

- Use the API’s build subcommand (same result as auto-build):

```bash
python3 scripts/ai_api.py BUILD                 # configure (if needed) + build
python3 scripts/ai_api.py BUILD --reconfigure   # force reconfigure then build
```

- Or use the provided VS Code Tasks (Terminal → Run Task…):
  - “Configure and build (Release)”
  - “Build CMake project (Release)”

- Or run CMake by hand (optional):
  - Create `build/` and run a Release configure, then build the targets.

Troubleshooting build:
- If the build reports missing libraries (CGAL/PCL/Boost/Eigen/VTK), install them and rebuild.
- If you switch compilers or upgrade dependencies, use `BUILD --reconfigure` or delete `build/` and re-run a head code; the API will regenerate.

## Design overview

Identifiers and naming conventions (already used by C++ tools):

The API builds three indices:

Ambiguity is handled with heuristics and optional substring hints:

## Head code mapping

Short, 3-letter head codes are dispatched by `scripts/ai_api.py`:

- `RCN` (Reconstruct): Given an `object_code` or a cluster filename, call `pcg_reconstruct`.
  - Input: `--object <object_code>` or `--filename <cluster_name.ply>`; optional `--only-substr`.
  - Internals: `pcg_reconstruct <cluster_ply> <room_dir>`.

- `VOL` (Mesh volume): Given an `object_code` or mesh/cluster filename, call `pcg_volume`.
  - Input: `--object <object_code>` or `--filename <mesh_or_cluster_name.ply>`.
  - If a mesh is missing and the input is a cluster, auto-reconstruct by default (disable with `--no-auto-recon`).
  - Output: prints `mesh_path`, `closed: true|false`, `volume: <number|null>`; for open meshes (e.g., AF), volume is skipped/`null`.

- `ARE` (Mesh surface area): Given an `object_code` or mesh/cluster filename, call `pcg_area`.
  - Input: `--object <object_code>` or `--filename <mesh_or_cluster_name.ply>`.
  - If a mesh is missing and the input is a cluster, auto-reconstruct by default (disable with `--no-auto-recon`).
  - Output: prints `mesh_path`, `closed: true|false`, `area: <number>`; area is computed for both open and closed meshes.

- `CLR` (Dominant color analysis): Given an `object_code` (prefers cluster) or a PLY filename, call `pcg_color` to analyze colors.
  - Input: `--object <object_code>` or `--filename <ply>`
  - Output: Raw stdout from `pcg_color` and a light JSON parse when `--json` is used.

Example two-object op included:
- `BBD` (BBox Distance): distance and vector between two UOBB centers using `pcg_bbox`.
  - Input: Two object codes, e.g., `BBD 0-7-12 0-7-14`.
  - What it acts on: UOBB PLY files (`<object_code>_<class>_uobb.ply`) located under `results/filtered_clusters/<stem>/`.
  - Behavior: Resolves each object's UOBB path, then calls `pcg_bbox <uobb1> <uobb2>`.
  - Output:
    - With `--json`: `{ "distance": <float>, "vector_1_to_2": { "x": dx, "y": dy, "z": dz } }`.
    - Without `--json`: two lines
      - `vector_1_to_2: dx, dy, dz`
      - `distance: d`
  - Error modes: If either UOBB file cannot be found from the given object code(s), the command fails with a clear error message (non-zero exit).


## Quick usage

Python module:
```python
from scripts.ai_api import Dispatcher
d = Dispatcher()
# Feature 1 examples
print(d.index.find_by_filename("0-7-12_couch_cluster.ply"))
print(d.index.find_csv(0, 7))
print(d.index.find_assets("0-7-12"))

# Feature 2 examples
mesh_path = d.op_RCN(object_code="0-7-12")
```

CLI for agents:
```bash
python3 scripts/ai_api.py resolve-filename 0-7-12_couch_cluster.ply
python3 scripts/ai_api.py VOL --object 0-7-12
python3 scripts/ai_api.py ARE --object 0-7-12 --json
python3 scripts/ai_api.py BBD 0-7-12 0-7-14
python3 scripts/ai_api.py CLR --object 0-7-12 --json
```
CLI for agents:
```bash
python3 scripts/ai_api.py resolve-filename 0-7-12_couch_cluster.ply
python3 scripts/ai_api.py VOL --object 0-7-12
python3 scripts/ai_api.py ARE --object 0-7-12 --json
python3 scripts/ai_api.py BBD 0-7-12 0-7-14
python3 scripts/ai_api.py CLR --object 0-7-12 --json

## Notes and edge cases
- Filename-only resolution can yield multiple paths (e.g., mirrored outputs); the API chooses the most likely one but also supports narrowing via a substring filter.
- Auto-build: if required executables are missing, the dispatcher will configure and build them automatically. You can also call `python3 scripts/ai_api.py BUILD` to build on demand.
- Manual CMake builds are fine too—ensure CGAL/PCL and friends are available and build a Release configuration in `build/`.
- Object codes and filenames are trusted to conform to the pipeline’s naming; if external files deviate, results may be unpredictable.
- Future two-object ops can mirror the `BBD` pattern and accept two `object_code`s.

## JSON output (global)
- The global flag `json_output` in `data/configs/default.yaml` controls whether C++ tools emit structured JSON instead of human-readable text. Default is `true` in this repository.
- When `json_output: true`, each tool writes JSON to stdout (one JSON object per processed input). All messages are in English.
- ai_api.py also supports `--json` for its own CLI to return structured results regardless of the C++ tools' mode; internally it prefers consuming JSON from tools and falls back to text parsing if needed.
### Per-tool JSON shapes

- pcg_volume:
  - { "file": "<path>", "closed": true|false, "volume": <float> }

- pcg_area:
  - { "file": "<path>", "closed": true|false, "area": <float> }

- pcg_bbox:
  - gen:  { "mode": "gen",  "status": "ok|failed", "file": "<out.ply>" }
  - pair (BBD): { "mode": "pair", "center1": {x,y,z}, "center2": {x,y,z}, "vector_1_to_2": {x,y,z}, "distance": <float> }
    - errors: { "mode": "pair", "status": "not_found|read_failed|empty", ... }

- pcg_color (CLR):
  - { "file": "<ply>", "M": <int>, "components": [ { "weight": <float>, "mean": [r,g,b], "var": [vr,vg,vb] }, ... ] }
  - when no component survives: { "file": "<ply>", "M": 0 }

- pcg_reconstruct (RCN):
  - For each input cluster, prints one JSON object:
    - success (poisson): { "file": "<cluster.ply>", "method": "poisson", "mesh": "<mesh.ply>", "status": "ok" }
    - success (af):      { "file": "<cluster.ply>", "method": "af",      "mesh": "<mesh.ply>", "status": "ok" }
    - read failure:      { "file": "<cluster.ply>", "status": "read_failed" }
    - reconstruction failed: { "file": "<cluster.ply>", "status": "failed" }

Notes:
- Tools that process multiple inputs (e.g., pcg_reconstruct walking a room) emit one JSON object per processed cluster (line-separated), not a single JSON array, to preserve streaming behavior.
- If you prefer array outputs, we can add a batch wrapper mode, but it would buffer results in memory and lose streaming.
