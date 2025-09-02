"""Merge all URDF link visual/collision meshes into global OBJ files.

Usage (inside conda env `discoverse`):
	python align_stp/merge_urdf.py examples/agilex-piper/piper.urdf --outdir /tmp/out

Outputs (unless filtered by flags):
	visual.obj, collision.obj, mapping.json (always) in the outdir (default: <URDF_DIR>/merged_model).

Design notes:
	- Root link = link that never appears as any joint child.
	- Joint <origin xyz rpy> produces transform T_parent_child = Trans(xyz) @ R(rpy)
	- Geometry <origin> similarly handled, final T = T_link_global @ T_geom_local
	- RPY convention (URDF standard): roll about X, pitch about Y, yaw about Z.
	  Rotation matrix: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
	- Scale (possibly 1 or 3 values) applied directly to mesh vertices before transform.
	- Each geometry exported as an OBJ object ("o name"). Multiple geometries per link
	  are suffixed:  link__v0 / link__c0 etc.
	- Keeps geometry separation (no vertex welding) for editor inspection.

Limitations (first version):
	- No primitive (box/cylinder/sphere) expansion (only <mesh> supported)
	- No package:// resolution (could be integrated from existing project resolvers)
	- No MTL material export
	- Non-uniform scale does not recompute corrected normals (trimesh may still provide)

Exit codes:
	0 success, non-zero on error.
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import json
from datetime import datetime, timezone
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Iterable, Any
import multiprocessing as mp
try:  # optional progress bar
	from tqdm import tqdm  # type: ignore
except Exception:  # pragma: no cover
	tqdm = None  # type: ignore
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

try:
	import trimesh  # type: ignore
except ImportError as e:  # pragma: no cover
	print("trimesh is required. Please install dependencies.", file=sys.stderr)
	raise

logger = logging.getLogger(__name__)


# ============================= Data Classes ============================= #

@dataclass
class JointInfo:
	name: str
	parent: str
	child: str
	T_parent_child: np.ndarray  # 4x4
	type: str


@dataclass
class GeometrySpec:
	link: str
	kind: str  # 'visual' | 'collision'
	path: Path
	scale: Tuple[float, float, float]
	T_local: np.ndarray  # 4x4
	index: int  # per-link order for naming


@dataclass
class GeometryInstance:
	mesh: "trimesh.Trimesh"
	part_name: str
	kind: str  # 'visual' | 'collision'
	link: str
	source_path: Path
	scale: Tuple[float, float, float]
	T_final: np.ndarray  # 4x4 global transform (before applied to mesh)


# ============================= Math Utils ============================= #

def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
	"""Return 3x3 rotation matrix from URDF RPY (r,p,y)."""
	# R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
	cr, sr = math.cos(roll), math.sin(roll)
	cp, sp = math.cos(pitch), math.sin(pitch)
	cy, sy = math.cos(yaw), math.sin(yaw)
	Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
	Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
	Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
	return Rz @ Ry @ Rx




def make_transform(xyz: Tuple[float, float, float], rpy: Tuple[float, float, float]) -> np.ndarray:
	R = rpy_to_matrix(*rpy)
	T = np.eye(4)
	T[:3, :3] = R
	T[:3, 3] = np.asarray(xyz)
	return T


def parse_xyz_rpy(elem: ET.Element | None) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
	if elem is None:
		return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
	xyz_str = elem.attrib.get("xyz", "0 0 0").strip()
	rpy_str = elem.attrib.get("rpy", "0 0 0").strip()
	xyz_vals = tuple(float(x) for x in xyz_str.split())  # type: ignore
	rpy_vals = tuple(float(x) for x in rpy_str.split())  # type: ignore
	return xyz_vals, rpy_vals


# ============================= URDF Parsing ============================= #

def parse_urdf(path: Path) -> Tuple[Dict[str, JointInfo], Dict[str, List[str]], str, ET.Element]:
	tree = ET.parse(path)
	root_elem = tree.getroot()

	link_elems = {l.attrib["name"]: l for l in root_elem.findall("link") if "name" in l.attrib}
	joint_infos: Dict[str, JointInfo] = {}
	children_map: Dict[str, List[str]] = {}
	child_links = set()

	for joint in root_elem.findall("joint"):
		name = joint.attrib.get("name", "")
		parent_elem = joint.find("parent")
		child_elem = joint.find("child")
		if parent_elem is None or child_elem is None:
			logger.warning("Joint missing parent/child: %s", name)
			continue
		parent = parent_elem.attrib.get("link")
		child = child_elem.attrib.get("link")
		if not parent or not child:
			logger.warning("Joint parent/child link name missing: %s", name)
			continue
		origin = joint.find("origin")
		xyz, rpy = parse_xyz_rpy(origin)
		T = make_transform(xyz, rpy)
		jtype = joint.attrib.get("type", "fixed")
		ji = JointInfo(name=name, parent=parent, child=child, T_parent_child=T, type=jtype)
		joint_infos[name] = ji
		children_map.setdefault(parent, []).append(child)
		child_links.add(child)

	all_links = set(link_elems.keys())
	roots = list(all_links - child_links)
	if not roots:
		raise ValueError("No root link detected.")
	root_link = roots[0]
	if len(roots) > 1:
		logger.warning("Multiple root links detected (%s); using %s", roots, root_link)

	# Cycle detection
	visiting = set()
	visited = set()

	def dfs(link: str):
		if link in visiting:
			raise ValueError("Cycle detected at link: " + link)
		if link in visited:
			return
		visiting.add(link)
		for c in children_map.get(link, []):
			dfs(c)
		visiting.remove(link)
		visited.add(link)

	dfs(root_link)

	return joint_infos, children_map, root_link, root_elem


# ============================= Transforms ============================= #

def compute_global_transforms(root_link: str, joint_infos: Dict[str, JointInfo], children_map: Dict[str, List[str]]) -> Dict[str, np.ndarray]:
	# Map child->joint for quick lookup
	child_to_joint: Dict[str, JointInfo] = {ji.child: ji for ji in joint_infos.values()}
	T_globals: Dict[str, np.ndarray] = {root_link: np.eye(4)}

	stack = [root_link]
	while stack:
		parent = stack.pop()
		for child in children_map.get(parent, []):
			ji = child_to_joint.get(child)
			if ji is None:
				logger.warning("Missing joint for child link %s", child)
				continue
			T_globals[child] = T_globals[parent] @ ji.T_parent_child
			stack.append(child)
	return T_globals


# ============================= Geometry Collection ============================= #

def _extract_mesh_info(geom_elem: ET.Element, urdf_dir: Path) -> Tuple[Path | None, Tuple[float, float, float]]:
	mesh_elem = geom_elem.find("mesh")
	if mesh_elem is None:
		return None, (1.0, 1.0, 1.0)
	filename = mesh_elem.attrib.get("filename")
	if not filename:
		return None, (1.0, 1.0, 1.0)
	# Resolve relative path (no package:// for now)
	mesh_path = (urdf_dir / filename).resolve()
	scale_attr = mesh_elem.attrib.get("scale")
	if scale_attr:
		parts = [float(x) for x in scale_attr.strip().split()]
		if len(parts) == 1:
			sx = sy = sz = parts[0]
		elif len(parts) == 3:
			sx, sy, sz = parts
		else:
			logger.warning("Invalid scale attribute '%s' in %s; using 1 1 1", scale_attr, filename)
			sx = sy = sz = 1.0
		scale = (sx, sy, sz)
	else:
		scale = (1.0, 1.0, 1.0)
	return mesh_path, scale


def collect_geometries(root_elem: ET.Element, urdf_path: Path) -> List[GeometrySpec]:
	urdf_dir = urdf_path.parent
	specs: List[GeometrySpec] = []
	# Build link->element map to maintain ordering
	for link_elem in root_elem.findall("link"):
		link_name = link_elem.attrib.get("name", "")
		if not link_name:
			continue
		# Visuals
		for idx, vis in enumerate(link_elem.findall("visual")):
			origin = vis.find("origin")
			xyz, rpy = parse_xyz_rpy(origin)
			T_local = make_transform(xyz, rpy)
			geom = vis.find("geometry")
			if geom is None:
				logger.warning("Link %s visual %d has no geometry", link_name, idx)
				continue
			path, scale = _extract_mesh_info(geom, urdf_dir)
			if path is None:
				logger.warning("Link %s visual %d has no mesh", link_name, idx)
				continue
			specs.append(GeometrySpec(link=link_name, kind="visual", path=path, scale=scale, T_local=T_local, index=idx))
		# Collisions
		for idx, col in enumerate(link_elem.findall("collision")):
			origin = col.find("origin")
			xyz, rpy = parse_xyz_rpy(origin)
			T_local = make_transform(xyz, rpy)
			geom = col.find("geometry")
			if geom is None:
				logger.warning("Link %s collision %d has no geometry", link_name, idx)
				continue
			path, scale = _extract_mesh_info(geom, urdf_dir)
			if path is None:
				logger.warning("Link %s collision %d has no mesh", link_name, idx)
				continue
			specs.append(GeometrySpec(link=link_name, kind="collision", path=path, scale=scale, T_local=T_local, index=idx))
	return specs


# ============================= Instantiation ============================= #

def load_mesh(path: Path):
	if not path.exists():
		raise FileNotFoundError(f"Mesh file not found: {path}")
	loaded = trimesh.load(path, force='mesh', skip_materials=True)
	if isinstance(loaded, trimesh.Scene):  # pragma: no cover (rare branch with force='mesh')
		# concatenate all geometry
		if not loaded.geometry:
			raise ValueError(f"Empty scene in mesh: {path}")
		meshes = [g for g in loaded.geometry.values()]
		loaded = trimesh.util.concatenate(meshes)
	if not isinstance(loaded, trimesh.Trimesh):
		raise TypeError(f"Unsupported mesh object {type(loaded)} from {path}")
	return loaded


def instantiate_geometries(specs: List[GeometrySpec], T_globals: Dict[str, np.ndarray], show_progress: bool = False) -> List[GeometryInstance]:
	mesh_cache: Dict[Path, trimesh.Trimesh] = {}
	instances: List[GeometryInstance] = []
	per_link_counters: Dict[Tuple[str, str], int] = {}
	iter_specs = specs
	if show_progress and tqdm is not None:
		iter_specs = tqdm(specs, desc="Instantiate", unit="geom")  # type: ignore
	for spec in iter_specs:  # type: ignore
		if spec.link not in T_globals:
			logger.warning("Link %s missing global transform; skipping geometry", spec.link)
			continue
		if spec.path not in mesh_cache:
			mesh_cache[spec.path] = load_mesh(spec.path)
		base_mesh = mesh_cache[spec.path]
		mesh_copy = base_mesh.copy()
		# Skip empty meshes (no faces)
		if mesh_copy.faces is None or len(mesh_copy.faces) == 0:
			logger.warning("Skipping empty mesh (no faces): %s (link=%s kind=%s)", spec.path, spec.link, spec.kind)
			continue
		sx, sy, sz = spec.scale
		if not (math.isclose(sx, 1.0) and math.isclose(sy, 1.0) and math.isclose(sz, 1.0)):
			mesh_copy.apply_scale([sx, sy, sz])
		T_final = T_globals[spec.link] @ spec.T_local
		mesh_copy.apply_transform(T_final)
		# Naming
		key = (spec.link, spec.kind)
		cnt = per_link_counters.get(key, 0)
		per_link_counters[key] = cnt + 1
		suffix = f"__v{cnt}" if spec.kind == "visual" else f"__c{cnt}"
		part_name = f"{spec.link}{suffix if cnt > 0 else ''}"  # first geometry keeps pure link name
		instances.append(GeometryInstance(mesh=mesh_copy, part_name=part_name, kind=spec.kind, link=spec.link, source_path=spec.path, scale=spec.scale, T_final=T_final))
	return instances


def _worker_instantiate(args: Tuple[GeometrySpec, np.ndarray]) -> Optional[Tuple[GeometrySpec, np.ndarray, np.ndarray]]:
	"""Worker: load (cached outside), scale & transform returns (spec, vertices, faces) arrays.
	Returns None if mesh empty or error."""
	spec, T_global = args
	try:
		mesh = load_mesh(spec.path)
		mesh = mesh.copy()
		if mesh.faces is None or len(mesh.faces) == 0:
			return None
		sx, sy, sz = spec.scale
		if not (math.isclose(sx,1.0) and math.isclose(sy,1.0) and math.isclose(sz,1.0)):
			mesh.apply_scale([sx, sy, sz])
		T_final = T_global @ spec.T_local
		mesh.apply_transform(T_final)
		return spec, mesh.vertices, mesh.faces
	except Exception:
		return None


def instantiate_geometries_parallel(specs: List[GeometrySpec], T_globals: Dict[str, np.ndarray], workers: int = 1, show_progress: bool = False) -> List[GeometryInstance]:
	"""Parallel variant (process-based) to speed up per-geometry transform.
	If workers <=1 falls back to sequential path.
	Note: materials/uv/normals recomputed later by export; only vertices/faces kept here.
	"""
	if workers <= 0:
		workers = max(1, mp.cpu_count() // 2)
	if workers <= 1:
		return instantiate_geometries(specs, T_globals, show_progress=show_progress)
	# Prepare inputs
	tasks: List[Tuple[GeometrySpec, np.ndarray]] = []
	for spec in specs:
		if spec.link not in T_globals:
			continue
		tasks.append((spec, T_globals[spec.link]))
	results: List[Optional[Tuple[GeometrySpec, np.ndarray, np.ndarray]]] = []
	with mp.Pool(processes=workers) as pool:
		it = pool.imap_unordered(_worker_instantiate, tasks, chunksize=4)
		if show_progress and tqdm is not None:
			it = tqdm(it, total=len(tasks), desc="Instantiate(mp)", unit="geom")  # type: ignore
		for res in it:  # type: ignore
			results.append(res)
	instances: List[GeometryInstance] = []
	per_link_counters: Dict[Tuple[str,str], int] = {}
	for item in results:
		if item is None:
			continue
		spec, verts, faces = item
		# Reconstruct trimesh object
		mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
		T_final = T_globals[spec.link] @ spec.T_local
		key = (spec.link, spec.kind)
		cnt = per_link_counters.get(key, 0)
		per_link_counters[key] = cnt + 1
		suffix = f"__v{cnt}" if spec.kind == "visual" else f"__c{cnt}"
		part_name = f"{spec.link}{suffix if cnt > 0 else ''}"
		instances.append(GeometryInstance(mesh=mesh, part_name=part_name, kind=spec.kind, link=spec.link, source_path=spec.path, scale=spec.scale, T_final=T_final))
	return instances


# ============================= OBJ Export ============================= #

def export_obj(instances: List[GeometryInstance], out_path: Path, kind_filter: str | None = None) -> None:
	if kind_filter:
		data = [i for i in instances if i.kind == kind_filter]
	else:
		data = instances
	if not data:
		logger.info("No instances to export for %s", kind_filter or 'all')
		return
	out_path.parent.mkdir(parents=True, exist_ok=True)
	v_ofs = 0
	vt_ofs = 0
	vn_ofs = 0
	with open(out_path, "w", encoding="utf-8") as f:
		f.write(f"# Generated by merge_urdf.py\n")
		# Dummy material lib reference to allow per-object distinct materials if needed
		f.write("mtllib merged_dummy.mtl\n")
		for inst in data:
			mesh = inst.mesh
			f.write(f"o {inst.part_name}\n")
			f.write(f"g {inst.part_name}\n")
			f.write(f"usemtl {inst.part_name}\n")
			# Vertices
			for v in mesh.vertices:
				f.write(f"v {v[0]:.9g} {v[1]:.9g} {v[2]:.9g}\n")
			has_uv = hasattr(mesh.visual, 'uv') and mesh.visual.uv is not None and len(mesh.visual.uv) == len(mesh.vertices)
			if has_uv:
				for uv in mesh.visual.uv:  # type: ignore
					f.write(f"vt {uv[0]:.9g} {uv[1]:.9g}\n")
			# Normals (compute if missing)
			if mesh.vertex_normals is None or len(mesh.vertex_normals) != len(mesh.vertices):  # pragma: no cover
				try:
					mesh.rezero()  # harmless
					mesh.compute_vertex_normals()
				except Exception:  # noqa
					pass
			has_normals = mesh.vertex_normals is not None and len(mesh.vertex_normals) == len(mesh.vertices)
			if has_normals:
				for n in mesh.vertex_normals:
					f.write(f"vn {n[0]:.9g} {n[1]:.9g} {n[2]:.9g}\n")
			# Faces
			faces = mesh.faces
			for face in faces:
				va, vb, vc = (face + 1 + v_ofs)
				# Local (per-mesh) indices for uv/normals use their own offsets
				if has_uv:
					uta, utb, utc = (face + 1 + vt_ofs)
				if has_normals:
					vna, vnb, vnc = (face + 1 + vn_ofs)
				if has_uv and has_normals:
					f.write(f"f {va}/{uta}/{vna} {vb}/{utb}/{vnb} {vc}/{utc}/{vnc}\n")
				elif has_uv and not has_normals:
					f.write(f"f {va}/{uta} {vb}/{utb} {vc}/{utc}\n")
				elif not has_uv and has_normals:
					f.write(f"f {va}//{vna} {vb}//{vnb} {vc}//{vnc}\n")
				else:
					f.write(f"f {va} {vb} {vc}\n")
			v_ofs += len(mesh.vertices)
			if has_uv:
				vt_ofs += len(mesh.vertices)
			if has_normals:
				vn_ofs += len(mesh.vertices)
	logger.info("Exported %d parts -> %s", len(data), out_path)


# ============================= CLI ============================= #

def build_arg_parser() -> argparse.ArgumentParser:
	p = argparse.ArgumentParser(description="Merge URDF visual/collision meshes into OBJ + mapping JSON with progress bar.")
	p.add_argument("urdf", type=Path, help="Path to URDF file")
	p.add_argument("-o", "--outdir", type=Path, required=False, help="Output directory (default: <URDF_DIR>/merged_model)")
	group = p.add_mutually_exclusive_group()
	group.add_argument("--visual-only", action="store_true", help="Only export visual meshes")
	group.add_argument("--collision-only", action="store_true", help="Only export collision meshes")
	p.add_argument("--log-level", default="INFO", help="Logging level (DEBUG, INFO, WARNING, ERROR)")
	p.add_argument("-nw", "--workers", type=int, default=0, help="Worker processes (<=0 => use half of CPU cores automatically)")
	return p


def run(urdf_path: Path, outdir: Optional[Path], export_visual: bool, export_collision: bool, workers: int = 1) -> Dict:
	if not urdf_path.exists():
		raise FileNotFoundError(f"URDF not found: {urdf_path}")
	if outdir is None:
		outdir = urdf_path.parent / "merged_model"
	joint_infos, children_map, root_link, root_elem = parse_urdf(urdf_path)
	logger.info("Root link: %s", root_link)
	T_globals = compute_global_transforms(root_link, joint_infos, children_map)
	specs = collect_geometries(root_elem, urdf_path)
	logger.info("Collected %d geometry specs", len(specs))
	# Always show progress if tqdm available
	instances = instantiate_geometries_parallel(specs, T_globals, workers=workers, show_progress=True)
	logger.info("Instantiated %d geometry meshes", len(instances))
	outdir.mkdir(parents=True, exist_ok=True)
 	# Export OBJ(s)
	if export_visual:
		export_obj(instances, outdir / "visual.obj", kind_filter="visual")
	if export_collision:
		export_obj(instances, outdir / "collision.obj", kind_filter="collision")

	# Always generate mapping.json
	mapping_path = outdir / "mapping.json"
	parts: List[Dict] = []
	include_kinds = set()
	if export_visual:
		include_kinds.add("visual")
	if export_collision:
		include_kinds.add("collision")
	urdf_dir = urdf_path.parent.resolve()
	for inst in instances:
		if inst.kind not in include_kinds:
			continue
		mesh = inst.mesh
		min_bb, max_bb = mesh.bounds
		T_list = [float(x) for x in inst.T_final.reshape(-1).tolist()]
		translation = [float(x) for x in inst.T_final[:3, 3].tolist()]
		quat_xyzw = Rotation.from_matrix(inst.T_final[:3, :3]).as_quat()  # x,y,z,w
		quat = [float(quat_xyzw[0]), float(quat_xyzw[1]), float(quat_xyzw[2]), float(quat_xyzw[3])]
		parts.append({
			"name": inst.part_name,
			"link": inst.link,
			"kind": inst.kind,
			"source_mesh": str(inst.source_path.resolve()),
			"source_mesh_rel": str(inst.source_path.resolve().relative_to(urdf_dir)) if inst.source_path.is_relative_to(urdf_dir) else None,
			"scale": list(inst.scale),
			"position": translation,
			"quat": quat,
			"transform_4x4": T_list,
			"aabb": {"min": [float(x) for x in min_bb.tolist()], "max": [float(x) for x in max_bb.tolist()]},
			"vertices": int(mesh.vertices.shape[0]),
			"faces": int(mesh.faces.shape[0]),
		})
	counts = {"visual": sum(1 for p in parts if p["kind"] == "visual"), "collision": sum(1 for p in parts if p["kind"] == "collision")}
	counts["total"] = counts["visual"] + counts["collision"]
	mapping = OrderedDict()
	mapping["version"] = 1
	mapping["timestamp"] = datetime.now(timezone.utc).isoformat()
	mapping["counts"] = counts
	mapping["urdf"] = str(urdf_path.resolve())
	mapping["root_link"] = root_link
	mapping["parts"] = parts
	mapping_path.parent.mkdir(parents=True, exist_ok=True)
	with open(mapping_path, "w", encoding="utf-8") as f:
		json.dump(mapping, f, indent=2)
		f.write("\n")
	logger.info("Wrote mapping JSON -> %s", mapping_path)
	return mapping


def main(argv: List[str] | None = None) -> int:
	parser = build_arg_parser()
	args = parser.parse_args(argv)
	logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO), format='[%(levelname)s] %(message)s')
	export_visual = True
	export_collision = True
	if args.visual_only:
		export_collision = False
	if args.collision_only:
		export_visual = False
	try:
		run(args.urdf, args.outdir, export_visual, export_collision, workers=args.workers)
	except Exception as e:  # pragma: no cover
		logger.error("Failed: %s", e)
		return 1
	return 0


if __name__ == "__main__":  # pragma: no cover
	raise SystemExit(main())

