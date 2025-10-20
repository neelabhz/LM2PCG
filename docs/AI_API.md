# AI API layer: path resolution + head-code dispatcher

This document sketches a unified, AI-friendly control surface for the pipeline. It provides:

- Path resolution APIs (Feature 1):
  - By `object_code` (e.g., `0-7-12`) → find cluster/uobb/mesh files and the room directory
  - By filename only (e.g., `0-7-12_couch_cluster.ply`) → resolve absolute path under `output/`
  - By floor-room (e.g., `0-7`) → find the room `.csv`
- Head-code dispatcher (Feature 2):
  - Short 3-letter head codes that call into existing binaries with the correct inputs

The implementation is a lightweight Python script at `scripts/ai_api.py` that scans the `output/` tree once to build indices, then resolves and dispatches operations.

## Design overview

Identifiers and naming conventions (already used by C++ tools):
- `object_code` = `<floor_id>-<room_id>-<object_id>` (e.g., `0-7-12`)
- File names: `<object_code>_<class>_{cluster|uobb|mesh}.ply`
- Room layout (example): `output/<site>/floor_0/room_007/`
- Room CSV location: `output/.../floor_0/room_007/room_007.csv`
- Cluster PLY location: `.../results/filtered_clusters/<stem>/<object_code>_<class>_cluster.ply`
- UOBB PLY location:    `.../results/filtered_clusters/<stem>/<object_code>_<class>_uobb.ply`
- Mesh PLY location:    `.../results/recon/<stem>/<object_code>_<class>_mesh.ply`

The API builds three indices:
- by filename → [paths]
- by room code `(floor_id, room_id)` → [csv paths]
- by object_code → `ObjectAssets {clusters[], uobbs[], meshes[], room_dir}`

Ambiguity is handled with heuristics and optional substring hints:
- If multiple matches exist, prefer `filtered_clusters` for clusters and `recon` for meshes; otherwise the first sorted match.
- A user can pass `--only-substr` when multiple cluster PLYs exist for the same object.

## Head code mapping

- `RCN` (Reconstruct): Given an `object_code` or a cluster filename, call `pcg_reconstruct` correctly and return the mesh path.
  - Input: `--object <object_code>` or `--filename <cluster_name.ply>`; optional `--only-substr`.
  - Output: absolute mesh PLY path.
  - Internals: `pcg_reconstruct <cluster_ply> <room_dir>`.

- `VOL` (Volume): Given an `object_code` or a mesh filename, compute closedness + volume via `pcg_volume`.
  - Input: `--object <object_code>` or `--filename <mesh_or_cluster_name.ply>`.
  - If a mesh is missing and the input is a cluster, auto-reconstruct by default (disable with `--no-auto-recon`).
  - Output: prints `mesh_path`, `closed: true|false`, `volume: <number>`.

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
mesh_path, volume, is_closed = d.op_VOL(object_code="0-7-12")
dist, vec = d.op_BBD("0-7-12", "0-7-13")
```

CLI for agents:
```bash
python3 scripts/ai_api.py resolve-filename 0-7-12_couch_cluster.ply
python3 scripts/ai_api.py resolve-room-csv 0 7
python3 scripts/ai_api.py resolve-object 0-7-12
python3 scripts/ai_api.py RCN --object 0-7-12
python3 scripts/ai_api.py VOL --object 0-7-12
python3 scripts/ai_api.py BBD 0-7-12 0-7-14
python3 scripts/ai_api.py CLR --object 0-7-12 --json
```

## Notes and edge cases

- Filename-only resolution can yield multiple paths (e.g., mirrored outputs); the API chooses the most likely one but also supports narrowing via a substring filter.
- If `pcg_reconstruct` or `pcg_volume` are not built, ensure CGAL is enabled and rebuild (`cmake -DCMAKE_BUILD_TYPE=Release .. && cmake --build . -j`).
- Object codes and filenames are trusted to conform to the pipeline’s naming; if external files deviate, results may be unpredictable.
- Future two-object ops can mirror the `BBD` pattern and accept two `object_code`s.

## JSON output (global)

- The global flag `json_output` in `data/configs/default.yaml` controls whether C++ tools emit structured JSON instead of human-readable text. Default is `false`.
- When `json_output: true`, each tool writes JSON to stdout (one JSON object per processed input). All messages are in English.
- ai_api.py also supports `--json` for its own CLI to return structured results regardless of the C++ tools' mode; internally it prefers consuming JSON from tools and falls back to text parsing if needed.

### Per-tool JSON shapes

- pcg_volume:
  - { "file": "<path>", "closed": true|false, "volume": <float> }

- pcg_bbox:
  - gen:  { "mode": "gen",  "status": "ok|failed", "file": "<out.ply>" }
  - point: { "mode": "point", "point": {x,y,z}, "bbox_center": {x,y,z}, "vector_point_to_center": {x,y,z}, "distance": <float> }
    - errors: { "mode": "point", "status": "not_found|read_failed", ... }
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
