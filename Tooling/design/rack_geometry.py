"""Independent rack interference guard on the viewer's geometry.

MuJoCo's contact solver still uses convex proxies. This surface query does not
apply forces and does not represent compliance, friction or latch mechanics.
"""

import fcl
import mujoco
import numpy as np
import trimesh


class RackGeometry:
    def __init__(self, model):
        ids, objects, labels, bounds, rack = [], [], [], [], []
        for geom in range(model.ngeom):
            if model.geom_group[geom] == 3:  # proxy, not the rendered part
                continue
            kind = model.geom_type[geom]
            body = model.body(model.geom_bodyid[geom]).name
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom)
            size = model.geom_size[geom]
            if kind == mujoco.mjtGeom.mjGEOM_MESH:
                mid = model.geom_dataid[geom]
                va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
                fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
                verts = model.mesh_vert[va : va + vn].astype(float)
                faces = model.mesh_face[fa : fa + fn]
                shape = fcl.BVHModel()
                shape.beginModel(len(verts), len(faces))
                shape.addSubModel(verts, faces)
                shape.endModel()
                local_bounds = np.array([verts.min(0), verts.max(0)])
                name = name or f"{body}/{model.mesh(mid).name}"
            elif kind == mujoco.mjtGeom.mjGEOM_BOX:
                shape = fcl.Box(*(2 * size))
                local_bounds = np.array([-size, size])
            elif kind in (
                mujoco.mjtGeom.mjGEOM_CAPSULE,
                mujoco.mjtGeom.mjGEOM_CYLINDER,
            ):
                radius, half = size[:2]
                mesh = (
                    trimesh.creation.capsule(radius=radius, height=2 * half)
                    if kind == mujoco.mjtGeom.mjGEOM_CAPSULE
                    else trimesh.creation.cylinder(radius=radius, height=2 * half)
                )
                shape = fcl.BVHModel()
                shape.beginModel(len(mesh.vertices), len(mesh.faces))
                shape.addSubModel(mesh.vertices, mesh.faces)
                shape.endModel()
                local_bounds = mesh.bounds
            else:
                continue
            ids.append(geom)
            objects.append(fcl.CollisionObject(shape))
            labels.append(name or body)
            bounds.append(local_bounds)
            rack.append(body.startswith(("dock_", "rack_gate_")))
        self.ids = np.array(ids)
        self.objects = objects
        self.labels = labels
        self.centres = np.mean(bounds, axis=1)[..., None]
        self.halfwidth = (np.diff(bounds, axis=1)[:, 0] / 2)[..., None]
        a, b = np.triu_indices(len(ids), 1)
        relevant = np.asarray(rack)[a] | np.asarray(rack)[b]
        self.a, self.b = a[relevant], b[relevant]
        self.request = fcl.CollisionRequest(num_max_contacts=1)

    def first_intersection(self, data):
        pos = data.geom_xpos[self.ids]
        rot = data.geom_xmat[self.ids].reshape(-1, 3, 3)
        centres = pos + (rot @ self.centres)[..., 0]
        radii = (np.abs(rot) @ self.halfwidth)[..., 0]
        a, b = self.a, self.b
        close = np.all(
            abs(centres[a] - centres[b]) <= radii[a] + radii[b] + 1e-9, axis=1
        )
        updated = set()
        for i, j in zip(a[close], b[close]):
            for k in (i, j):
                if k not in updated:
                    self.objects[k].setTransform(fcl.Transform(rot[k], pos[k]))
                    updated.add(k)
            if fcl.collide(
                self.objects[i], self.objects[j], self.request, fcl.CollisionResult()
            ):
                return self.labels[i], self.labels[j]
        return None
