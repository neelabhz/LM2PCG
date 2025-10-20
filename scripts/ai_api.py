#!/usr/bin/env python3
"""
AI-facing API layer for local point-cloud pipeline control.

Provides two main capabilities:
1) Path resolution (by object_code, filename, or room code)
2) Operation dispatch via 3-letter head codes (e.g., RCN, VOL)

Conventions assumed from the C++ pipeline:
- Object code format: <floor_id>-<room_id>-<object_id> (e.g., 0-7-12)
- Filenames: <object_code>_<class>_{cluster|uobb|mesh}.ply
- Room outputs: output/<site>/floor_<f>/room_<rrr>/
- CSV path: output/.../floor_<f>/room_<rrr>/<room_dirname>.csv
- Cluster PLYs: output/.../results/filtered_clusters/<stem>/<object_code>_<class>_cluster.ply
- UOBB PLYs:    output/.../results/filtered_clusters/<stem>/<object_code>_<class>_uobb.ply
- Recon meshes: output/.../results/recon/<stem>/<object_code>_<class>_mesh.ply

This script can be imported as a module or used as a small CLI.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# ------------------------------
# Helpers and parsing
# ------------------------------

OBJ_RE = re.compile(r"^(?P<floor>\d+)-(?:\s*)?(?P<room>\d+)-(?:\s*)?(?P<object>\d+)$")


def parse_object_code(s: str) -> Tuple[int, int, int]:
    m = OBJ_RE.match(s.strip())
    if not m:
        raise ValueError(f"Invalid object_code '{s}'. Expected '<floor>-<room>-<object>' like '0-7-12'.")
    return int(m.group("floor")), int(m.group("room")), int(m.group("object"))


def repo_root(start: Optional[Path] = None) -> Path:
    """Find repository root by looking for CMakeLists.txt upwards."""
    p = Path(start or __file__).resolve()
    for parent in [p] + list(p.parents):
        if (parent / "CMakeLists.txt").exists():
            return parent
    # Fallback: current working directory
    return Path.cwd()


def build_dir(root: Path) -> Path:
    return root / "build"


def output_root(root: Path) -> Path:
    return root / "output"


def is_room_dir(p: Path) -> bool:
    # room directory named like room_007; contains .csv and results usually
    return p.is_dir() and p.name.startswith("room_")


def room_identifiers(room_dir: Path) -> Tuple[Optional[int], Optional[int]]:
    """Extract floor_id, room_id from parent names like floor_0 and room_007."""
    try:
        floor_name = room_dir.parent.name
        room_name = room_dir.name
        def extract_int(s: str) -> Optional[int]:
            digits = ''.join(ch for ch in s if ch.isdigit())
            return int(digits) if digits else None
        return extract_int(floor_name), extract_int(room_name)
    except Exception:
        return None, None


# ------------------------------
# Path index and resolvers
# ------------------------------

@dataclass
class ObjectAssets:
    object_code: str
    clusters: List[Path]
    uobbs: List[Path]
    meshes: List[Path]
    room_dir: Optional[Path]


class PathIndex:
    """Scan output tree once to build lookups by filename, object_code, and room code."""
    def __init__(self, out_root: Path):
        self.out_root = out_root
        self.by_filename: Dict[str, List[Path]] = {}
        self.csv_by_room: Dict[Tuple[int, int], List[Path]] = {}
        self.assets_by_object: Dict[str, ObjectAssets] = {}

    def build(self) -> None:
        if not self.out_root.exists():
            return
        for p in self.out_root.rglob("*"):
            if p.is_file():
                # filename lookup
                self.by_filename.setdefault(p.name, []).append(p)

                # CSV mapping by room
                if p.suffix == ".csv" and is_room_dir(p.parent):
                    f_id, r_id = room_identifiers(p.parent)
                    if f_id is not None and r_id is not None:
                        self.csv_by_room.setdefault((f_id, r_id), []).append(p)

                # PLY asset mapping by object_code
                if p.suffix.lower() == ".ply":
                    stem = p.stem  # <object_code>_<class>_<kind>
                    parts = stem.split("_")
                    if len(parts) >= 3:
                        object_code = parts[0]
                        kind = parts[-1].lower()  # cluster|uobb|mesh
                        assets = self.assets_by_object.get(object_code)
                        if not assets:
                            assets = ObjectAssets(object_code, [], [], [], None)
                            self.assets_by_object[object_code] = assets
                        if kind == "cluster":
                            assets.clusters.append(p)
                        elif kind == "uobb":
                            assets.uobbs.append(p)
                        elif kind == "mesh":
                            assets.meshes.append(p)
                        # infer room_dir for this asset
                        rd = self._infer_room_dir_from_asset(p)
                        if rd is not None:
                            assets.room_dir = rd

    @staticmethod
    def _infer_room_dir_from_asset(p: Path) -> Optional[Path]:
        """Walk up to find a 'room_XXX' directory."""
        cur = p.parent
        for _ in range(8):
            if cur is None:
                break
            if is_room_dir(cur):
                return cur
            cur = cur.parent if cur != cur.parent else None
        return None

    # Public resolvers
    def find_by_filename(self, name: str) -> List[Path]:
        return self.by_filename.get(name, [])

    def find_csv(self, floor_id: int, room_id: int) -> List[Path]:
        return self.csv_by_room.get((floor_id, room_id), [])

    def find_assets(self, object_code: str) -> Optional[ObjectAssets]:
        return self.assets_by_object.get(object_code)


# ------------------------------
# Dispatcher for head codes
# ------------------------------

class Dispatcher:
    def __init__(self, root: Optional[Path] = None):
        self.root = repo_root(root)
        self.out_root = output_root(self.root)
        self.bin_dir = build_dir(self.root)
        self.index = PathIndex(self.out_root)
        self.index.build()

    def env_status(self) -> Dict[str, object]:
        """Report availability of required executables and basic paths."""
        bins = {
            "pcg_reconstruct": (self.bin_dir / "pcg_reconstruct").exists(),
            "pcg_volume": (self.bin_dir / "pcg_volume").exists(),
            "pcg_bbox": (self.bin_dir / "pcg_bbox").exists(),
            "pcg_room": (self.bin_dir / "pcg_room").exists(),
            "pcg_color": (self.bin_dir / "pcg_color").exists(),
        }
        return {
            "repo_root": str(self.root),
            "build_dir": str(self.bin_dir),
            "output_root": str(self.out_root),
            "executables": bins,
        }

    # --- Head code: RCN (reconstruction) ---
    def op_RCN(self, object_code: Optional[str] = None, filename: Optional[str] = None,
               only_substring: Optional[str] = None) -> Path:
        """Reconstruct mesh for a cluster. Input: object_code or filename.
        Returns the expected mesh path (created or updated).
        """
        if (object_code is None) == (filename is None):
            raise ValueError("RCN requires exactly one of object_code or filename.")

        if object_code:
            assets = self.index.find_assets(object_code)
            if not assets or not assets.clusters:
                raise FileNotFoundError(f"No cluster PLY found for object_code '{object_code}'.")
            # If multiple clusters found (rare), try narrowing by substring or pick the first
            cluster_path = self._choose_one(assets.clusters, only_substring)
            room_dir = assets.room_dir or self.index._infer_room_dir_from_asset(cluster_path)
        else:
            # accept direct path
            fp = Path(filename)  # type: ignore[arg-type]
            if fp.exists():
                cluster_path = fp
            else:
                matches = self.index.find_by_filename(filename)  # type: ignore[arg-type]
                matches = [p for p in matches if p.suffix == ".ply"]
                if not matches:
                    raise FileNotFoundError(f"No file named '{filename}' under '{self.out_root}'.")
                cluster_path = self._choose_one(matches, only_substring)
            # Validate looks like a cluster file
            if not cluster_path.stem.endswith("_cluster"):
                raise ValueError(f"RCN expects a cluster PLY, got '{cluster_path.name}'.")
            room_dir = self.index._infer_room_dir_from_asset(cluster_path)

        if room_dir is None:
            raise RuntimeError(f"Cannot infer room directory for '{cluster_path}'.")

        # Call pcg_reconstruct <cluster_ply> <room_dir>
        exe = self.bin_dir / "pcg_reconstruct"
        if not exe.exists():
            raise FileNotFoundError(f"Missing executable: {exe}")
        self._run([str(exe), str(cluster_path), str(room_dir)])

        # Expected mesh path after reconstruction
        oc, klass, kind = self._parse_asset_name(cluster_path)
        mesh_name = f"{oc}_{klass}_mesh.ply"
        stem_dir = cluster_path.parent.parent.name  # <stem> under filtered_clusters/<stem>
        mesh_dir = room_dir / "results" / "recon" / stem_dir
        return mesh_dir / mesh_name

    # --- Head code: VOL (mesh volume) ---
    def op_VOL(self, object_code: Optional[str] = None, filename: Optional[str] = None,
               auto_reconstruct: bool = True) -> Tuple[Path, float, bool]:
        """Compute volume (and closedness) for a reconstructed mesh.
        Input: object_code or mesh filename. If mesh missing and auto_reconstruct, try RCN.
        Returns: (mesh_path, volume, is_closed)
        """
        if (object_code is None) == (filename is None):
            raise ValueError("VOL requires exactly one of object_code or filename.")

        mesh_path: Optional[Path] = None
        if object_code:
            assets = self.index.find_assets(object_code)
            if assets and assets.meshes:
                mesh_path = self._choose_one(assets.meshes)
            elif auto_reconstruct:
                # Try reconstruct from cluster first
                mesh_path = self.op_RCN(object_code=object_code)
            else:
                raise FileNotFoundError(f"No mesh found for object_code '{object_code}'.")
        else:
            fp = Path(filename)  # type: ignore[arg-type]
            if fp.exists():
                candidate = fp
            else:
                matches = self.index.find_by_filename(filename)  # type: ignore[arg-type]
                matches = [p for p in matches if p.suffix == ".ply"]
                if not matches:
                    raise FileNotFoundError(f"No file named '{filename}' under '{self.out_root}'.")
                candidate = self._choose_one(matches)
            if candidate.stem.endswith("_mesh"):
                mesh_path = candidate
            elif auto_reconstruct and candidate.stem.endswith("_cluster"):
                # it's a cluster, reconstruct first
                mesh_path = self.op_RCN(filename=candidate.name)
            else:
                raise ValueError("VOL expects a mesh PLY, or a cluster with auto_reconstruct=True.")

        if mesh_path is None:
            raise RuntimeError("Internal error: mesh_path is None after resolution.")

        exe = self.bin_dir / "pcg_volume"
        if not exe.exists():
            raise FileNotFoundError(f"Missing executable: {exe}")
        # Run and parse output
        out = self._run([str(exe), str(mesh_path)])
        # Try JSON first
        j = self._try_parse_json(out)
        if j and "volume" in j and "closed" in j:
            return mesh_path, float(j["volume"]), bool(j["closed"])
        # Fallback to legacy text parsing
        is_closed = "closed: true" in out
        vol = None
        for line in out.splitlines():
            line = line.strip()
            if line.startswith("volume:"):
                try:
                    vol = float(line.split(":", 1)[1].strip())
                except Exception:
                    pass
        if vol is None:
            raise RuntimeError("Failed to parse volume from pcg_volume output.")
        return mesh_path, vol, is_closed

    # --- Head code: CLR (dominant color analysis) ---
    def op_CLR(self, object_code: Optional[str] = None, filename: Optional[str] = None,
               auto_pick_cluster: bool = True) -> Dict[str, object]:
        """Run color analysis using pcg_color.
        Input: object_code (prefers cluster), or explicit filename (any PLY with RGB).
        Returns a dict with parsed output if possible; otherwise raw text.
        """
        if (object_code is None) == (filename is None):
            raise ValueError("CLR requires exactly one of object_code or filename.")

        target: Optional[Path] = None
        if object_code:
            assets = self.index.find_assets(object_code)
            if not assets:
                raise FileNotFoundError(f"No assets found for object_code '{object_code}'.")
            # Prefer cluster if available, else fall back to any mesh/ply
            if auto_pick_cluster and assets.clusters:
                target = self._choose_one(assets.clusters)
            elif assets.meshes:
                target = self._choose_one(assets.meshes)
            elif assets.uobbs:
                target = self._choose_one(assets.uobbs)
            else:
                raise FileNotFoundError(f"No usable PLY for color analysis under '{object_code}'.")
        else:
            # allow direct file path or lookup by name
            fp = Path(filename)  # type: ignore[arg-type]
            if fp.exists():
                target = fp
            else:
                matches = self.index.find_by_filename(filename)  # type: ignore[arg-type]
                matches = [p for p in matches if p.suffix == ".ply"]
                if not matches:
                    raise FileNotFoundError(f"No file named '{filename}' under '{self.out_root}'.")
                target = self._choose_one(matches)

        exe = self.bin_dir / "pcg_color"
        if not exe.exists():
            raise FileNotFoundError(f"Missing executable: {exe}")
        out = self._run([str(exe), str(target)])

        # Prefer JSON
        j = self._try_parse_json(out)
        if j:
            return j

        # Fallback: light parse
        result: Dict[str, object] = {"file": str(target), "raw": out}
        try:
            lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
            m_val: Optional[int] = None
            comps: List[Dict[str, object]] = []
            for ln in lines:
                if ln.lower().startswith("final m="):
                    try:
                        m_val = int(ln.split("=", 1)[1].strip())
                    except Exception:
                        pass
                if ln.lower().startswith("component"):
                    comps.append({"line": ln})
            if m_val is not None:
                result["M"] = m_val
            if comps:
                result["components"] = comps
        except Exception:
            pass
        return result

    # --- Example two-object op: BBD (bbox distance) ---
    def op_BBD(self, object_code_1: str, object_code_2: str) -> Tuple[float, Tuple[float, float, float]]:
        """Compute distance and vector between two UOBB centers (example two-object op)."""
        assets1 = self.index.find_assets(object_code_1)
        assets2 = self.index.find_assets(object_code_2)
        if not assets1 or not assets1.uobbs:
            raise FileNotFoundError(f"No UOBB for object_code '{object_code_1}'.")
        if not assets2 or not assets2.uobbs:
            raise FileNotFoundError(f"No UOBB for object_code '{object_code_2}'.")
        u1 = self._choose_one(assets1.uobbs)
        u2 = self._choose_one(assets2.uobbs)
        exe = self.bin_dir / "pcg_bbox"
        if not exe.exists():
            raise FileNotFoundError(f"Missing executable: {exe}")
        out = self._run([str(exe), str(u1), str(u2)])
        # Try JSON
        j = self._try_parse_json(out)
        if j and "distance" in j and "vector_1_to_2" in j:
            v = j["vector_1_to_2"]
            vec = (float(v.get("x", 0.0)), float(v.get("y", 0.0)), float(v.get("z", 0.0)))
            return float(j["distance"]), vec
        # Fallback to legacy text
        vec = (0.0, 0.0, 0.0)
        dist = None
        for line in out.splitlines():
            s = line.strip()
            if s.startswith("vector_1_to_2:"):
                nums = s.split(":", 1)[1].split(",")
                if len(nums) == 3:
                    vec = tuple(float(x) for x in map(str.strip, nums))  # type: ignore[assignment]
            if s.startswith("distance:"):
                try:
                    dist = float(s.split(":", 1)[1].strip())
                except Exception:
                    pass
        if dist is None:
            raise RuntimeError("Failed to parse output from pcg_bbox.")
        return dist, vec  # type: ignore[return-value]

    # ------------------------------
    # Utilities
    # ------------------------------
    @staticmethod
    def _choose_one(paths: List[Path], only_substring: Optional[str] = None) -> Path:
        if not paths:
            raise FileNotFoundError("No candidates to choose from.")
        if only_substring:
            sub = [p for p in paths if only_substring in p.name]
            if sub:
                paths = sub
        if len(paths) > 1:
            # Prefer paths under filtered_clusters for clusters, recon for meshes
            def score(p: Path) -> int:
                s = 0
                parts = [str(x) for x in p.parts]
                if "filtered_clusters" in parts:
                    s += 2
                if "recon" in parts:
                    s += 1
                return -s
            paths = sorted(paths, key=score)
        return paths[0]

    @staticmethod
    def _parse_asset_name(p: Path) -> Tuple[str, str, str]:
        # returns (object_code, class, kind)
        parts = p.stem.split("_")
        if len(parts) < 3:
            raise ValueError(f"Unexpected asset name format: {p.name}")
        return parts[0], parts[-2], parts[-1]

    @staticmethod
    def _run(cmd: List[str]) -> str:
        try:
            proc = subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            return proc.stdout
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Command failed ({e.returncode}): {' '.join(cmd)}\nOutput:\n{e.stdout}")

    @staticmethod
    def _try_parse_json(text: str) -> Optional[Dict[str, object]]:
        s = text.strip()
        if not s:
            return None
        # Fast path
        if s.startswith("{") and s.endswith("}"):
            try:
                return json.loads(s)
            except Exception:
                pass
        # Try to find the last JSON object in the text
        last_open = s.rfind("{")
        last_close = s.rfind("}")
        if last_open != -1 and last_close != -1 and last_close > last_open:
            frag = s[last_open:last_close+1]
            try:
                return json.loads(frag)
            except Exception:
                return None
        return None


# ------------------------------
# Minimal CLI for agents
# ------------------------------

def _cli() -> int:
    parser = argparse.ArgumentParser(description="AI API for local PCG pipeline")
    sub = parser.add_subparsers(dest="cmd", required=True)

    # Resolve by filename
    p_res_fn = sub.add_parser("resolve-filename", help="Find file by name under output/")
    p_res_fn.add_argument("name")
    p_res_fn.add_argument("--json", action="store_true", help="Emit JSON output")

    # Resolve by room
    p_res_room = sub.add_parser("resolve-room-csv", help="Find CSV for floor-room")
    p_res_room.add_argument("floor", type=int)
    p_res_room.add_argument("room", type=int)
    p_res_room.add_argument("--json", action="store_true")

    # Resolve by object_code
    p_res_obj = sub.add_parser("resolve-object", help="Find assets for object_code")
    p_res_obj.add_argument("object_code")
    p_res_obj.add_argument("--json", action="store_true")

    # RCN
    p_rcn = sub.add_parser("RCN", help="Reconstruct from object_code or filename")
    g = p_rcn.add_mutually_exclusive_group(required=True)
    g.add_argument("--object", dest="object_code")
    g.add_argument("--filename")
    p_rcn.add_argument("--only-substr")
    p_rcn.add_argument("--json", action="store_true")

    # VOL
    p_vol = sub.add_parser("VOL", help="Compute mesh volume from object_code or filename")
    g2 = p_vol.add_mutually_exclusive_group(required=True)
    g2.add_argument("--object", dest="object_code")
    g2.add_argument("--filename")
    p_vol.add_argument("--no-auto-recon", action="store_true")
    p_vol.add_argument("--json", action="store_true")

    # CLR
    p_clr = sub.add_parser("CLR", help="Dominant color analysis on a cluster or file")
    g3 = p_clr.add_mutually_exclusive_group(required=True)
    g3.add_argument("--object", dest="object_code")
    g3.add_argument("--filename")
    p_clr.add_argument("--json", action="store_true")

    # BBD (example two-object op)
    p_bbd = sub.add_parser("BBD", help="BBox distance between two object_codes")
    p_bbd.add_argument("object_code_1")
    p_bbd.add_argument("object_code_2")
    p_bbd.add_argument("--json", action="store_true")

    # check environment
    p_chk = sub.add_parser("check-env", help="Show availability of required executables")
    p_chk.add_argument("--json", action="store_true")

    args = parser.parse_args()
    d = Dispatcher()

    if args.cmd == "resolve-filename":
        paths = [str(p) for p in d.index.find_by_filename(args.name)]
        if args.json:
            print(json.dumps({"matches": paths}, ensure_ascii=False))
        else:
            for p in paths:
                print(p)
        return 0 if paths else 1
    if args.cmd == "resolve-room-csv":
        paths = [str(p) for p in d.index.find_csv(args.floor, args.room)]
        if args.json:
            print(json.dumps({"floor": args.floor, "room": args.room, "csv": paths}, ensure_ascii=False))
        else:
            for p in paths:
                print(p)
        return 0 if paths else 1
    if args.cmd == "resolve-object":
        assets = d.index.find_assets(args.object_code)
        if not assets:
            if args.json:
                print(json.dumps({"object_code": args.object_code, "found": False}, ensure_ascii=False))
            else:
                print("<none>")
            return 1
        if args.json:
            print(json.dumps({
                "object_code": assets.object_code,
                "clusters": [str(p) for p in assets.clusters],
                "uobbs": [str(p) for p in assets.uobbs],
                "meshes": [str(p) for p in assets.meshes],
                "room_dir": str(assets.room_dir) if assets.room_dir else None
            }, ensure_ascii=False))
        else:
            print("# clusters:")
            for p in assets.clusters:
                print(p)
            print("# uobbs:")
            for p in assets.uobbs:
                print(p)
            print("# meshes:")
            for p in assets.meshes:
                print(p)
            if assets.room_dir:
                print(f"# room_dir: {assets.room_dir}")
        return 0
    if args.cmd == "RCN":
        mesh = d.op_RCN(object_code=args.object_code, filename=args.filename, only_substring=args.only_substr)
        if args.json:
            print(json.dumps({"mesh": str(mesh)}, ensure_ascii=False))
        else:
            print(mesh)
        return 0
    if args.cmd == "VOL":
        mesh, vol, closed = d.op_VOL(object_code=args.object_code, filename=args.filename, auto_reconstruct=not args.no_auto_recon)
        if args.json:
            print(json.dumps({"mesh": str(mesh), "closed": closed, "volume": vol}, ensure_ascii=False))
        else:
            print(mesh)
            print(f"closed: {str(closed).lower()}")
            print(f"volume: {vol}")
        return 0
    if args.cmd == "CLR":
        res = d.op_CLR(object_code=args.object_code, filename=args.filename)
        if args.json:
            print(json.dumps(res, ensure_ascii=False))
        else:
            # print raw if available
            if "raw" in res:
                print(res["raw"])  # type: ignore[index]
            else:
                print(res)
        return 0
    if args.cmd == "BBD":
        dist, vec = d.op_BBD(args.object_code_1, args.object_code_2)
        if args.json:
            print(json.dumps({
                "distance": dist,
                "vector_1_to_2": {"x": vec[0], "y": vec[1], "z": vec[2]}
            }, ensure_ascii=False))
        else:
            print(f"vector_1_to_2: {vec[0]}, {vec[1]}, {vec[2]}")
            print(f"distance: {dist}")
        return 0
    if args.cmd == "check-env":
        info = d.env_status()
        if args.json:
            print(json.dumps(info, ensure_ascii=False))
        else:
            print(f"repo_root: {info['repo_root']}")
            print(f"build_dir: {info['build_dir']}")
            print(f"output_root: {info['output_root']}")
            print("executables:")
            for k, v in info["executables"].items():
                print(f"  {k}: {'yes' if v else 'no'}")
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(_cli())
