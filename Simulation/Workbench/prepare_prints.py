"""Repack upstream A1 mini solids without changing geometry or Z orientation."""
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh

from engine import ROOT

BED = 180.0
MARGIN = 5.0
GAP = 6.0
DEST = ROOT / "Build/A1mini"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fits(rect, placed):
    x, y, w, h = rect
    if x < MARGIN or y < MARGIN or x+w > BED-MARGIN or y+h > BED-MARGIN:
        return False
    return all(x+w+GAP <= a or a+c+GAP <= x or y+h+GAP <= b or b+d+GAP <= y
               for a,b,c,d in placed)


def pack(pieces):
    plates = []
    for index in sorted(range(len(pieces)), key=lambda i: -np.prod(pieces[i].extents[:2])):
        p = pieces[index]
        found = None
        for plate_index in range(len(plates)+1):
            placed = [item[2] for item in plates[plate_index]] if plate_index < len(plates) else []
            xs = sorted({MARGIN} | {x+w+GAP for x,y,w,h in placed})
            ys = sorted({MARGIN} | {y+h+GAP for x,y,w,h in placed})
            candidates = []
            for rotated in (False, True):
                w,h = p.extents[:2][::-1] if rotated else p.extents[:2]
                for y in ys:
                    for x in xs:
                        rect = (x,y,float(w),float(h))
                        if fits(rect, placed):
                            candidates.append((y+h, x+w, rotated, rect))
            if candidates:
                _,_,rotated,rect = min(candidates)
                found = plate_index, rotated, rect
                break
        if found is None:
            raise ValueError(f"Solid {index} does not fit on the A1 mini without changing print orientation")
        plate_index, rotated, rect = found
        if plate_index == len(plates):
            plates.append([])
        plates[plate_index].append((index, rotated, rect))
    return plates


def main():
    DEST.mkdir(parents=True, exist_ok=True)
    all_manifest = {"printer": "Bambu Lab A1 mini", "bed_mm": [180,180,180],
                    "margin_mm": MARGIN, "part_gap_mm": GAP, "roles": {}}
    for role in ("Leader", "Follower"):
        source = ROOT / f"STL/SO101/{role}/BambuLabA1mini_{role}_SO101.stl"
        mesh = trimesh.load_mesh(source)
        pieces = list(mesh.split(only_watertight=False))
        names_by_faces = {}
        for individual in sorted((ROOT / "STL/SO101/Individual").glob("*.stl")):
            if "Assembly" in individual.name:
                continue
            for j, component in enumerate(trimesh.load_mesh(individual).split(only_watertight=False), 1):
                names_by_faces.setdefault(len(component.faces), []).append(individual.stem + f" (solid {j})")
        if not all(p.is_watertight for p in pieces):
            raise ValueError("Unexpected non-watertight source solid; inspect before repacking")
        plates = pack(pieces)
        folder = DEST / role.lower()
        folder.mkdir(exist_ok=True)
        manifest = {"source": str(source.relative_to(ROOT)), "source_sha256": digest(source),
                    "source_extents_mm": mesh.extents.tolist(), "solid_count": len(pieces), "plates": []}
        for plate_i, plate in enumerate(plates, 1):
            transformed, solids = [], []
            svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="720" height="780" viewBox="0 0 180 195">',
                   '<rect width="180" height="180" fill="#17212d"/>',
                   '<rect x="5" y="5" width="170" height="170" fill="none" stroke="#5d778b" stroke-width=".5"/>']
            for index, rotated, rect in plate:
                p = pieces[index].copy()
                if rotated:
                    p.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0,0,1]))
                x,y,w,h = rect
                p.apply_translation(np.array([x,y,0.])-p.bounds[0])
                assert p.bounds[0,2] >= -1e-5 and p.bounds[1,2] <= 180
                transformed.append(p)
                solids.append({"source_solid": index+1, "footprint_mm": list(rect),
                               "part_candidates_by_triangle_count": names_by_faces.get(len(p.faces), []),
                               "rotation_z_degrees": 90 if rotated else 0,
                               "height_mm": float(p.extents[2]), "triangles": len(p.faces),
                               "volume_mm3": float(abs(p.volume))})
                svg.extend([f'<rect x="{x:.2f}" y="{180-y-h:.2f}" width="{w:.2f}" height="{h:.2f}" fill="#368e94" stroke="#a5e3e5" stroke-width=".4"/>',
                            f'<text x="{x+2:.2f}" y="{180-y-h+6:.2f}" font-size="4" fill="white">Solid {index+1}</text>'])
            combined = trimesh.util.concatenate(transformed)
            target = folder / f"{role.lower()}_plate_{plate_i:02d}.stl"
            combined.export(target)
            svg.append(f'<text x="5" y="189" font-family="sans-serif" font-size="5" fill="#17212d">{role} · plate {plate_i} · 180 × 180 mm</text></svg>')
            target.with_suffix(".svg").write_text("\n".join(svg))
            manifest["plates"].append({"file": str(target.relative_to(ROOT)),
                "sha256": digest(target), "extents_mm": combined.extents.tolist(),
                "solids": solids})
        assert sum(len(p["solids"]) for p in manifest["plates"]) == len(pieces)
        assert sum(s["triangles"] for p in manifest["plates"] for s in p["solids"]) == len(mesh.faces)
        all_manifest["roles"][role.lower()] = manifest
        print(f"{role}: preserved {len(pieces)} solids on {len(plates)} bed-sized plates")
    (DEST / "manifest.json").write_text(json.dumps(all_manifest, indent=2)+"\n")


if __name__ == "__main__":
    main()
