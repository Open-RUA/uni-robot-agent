"""Hash-pinned URDF mesh clearance with per-link joint uncertainty bounds."""
from itertools import combinations, product
from pathlib import Path

import numpy as np

from uni_robot_agent.integrations.piper_x.body.core.contract import DeviceError, ErrorCode


class MeshChecks:
    def __init__(self, geometry, root, assets):
        import fcl

        self.fcl = fcl
        self.geometry = geometry
        self.objects = []
        joints = {j.find("child").get("link"): j for j in geometry.joints}
        expected = {(link.get("name"), index) for link in root.findall("link")
                    for index, _ in enumerate(link.findall("collision"))}
        if {(a.link, a.collision_index) for a in assets} != expected or len(assets) != len(expected):
            raise ValueError("Mesh assets must cover every URDF collision element exactly once")
        for asset in assets:
            link = root.find(f"./link[@name='{asset.link}']")
            element = link.findall("collision")[asset.collision_index]
            mesh = element.find("geometry/mesh")
            if mesh is None:
                raise ValueError("Mesh clearance requires URDF mesh collision elements")
            path = asset.file.resolve(geometry.installation.collision.parent)
            if path.name != Path(mesh.get("filename")).name:
                raise ValueError("Pinned mesh does not match its URDF element")
            data = path.read_bytes()
            if len(data) < 84 or len(data) != 84 + 50 * int.from_bytes(data[80:84], "little"):
                raise ValueError("Expected binary STL collision mesh")
            dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
            vertices = np.frombuffer(data, dtype=dtype, offset=84)["vertices"].reshape(-1, 3).astype(float)
            if not len(vertices) or not np.isfinite(vertices).all():
                raise ValueError("Invalid collision vertices")
            vertices *= np.array([float(x) for x in mesh.get("scale", "1 1 1").split()])
            origin = geometry._origin(element)
            vertices = vertices @ origin[:3, :3].T + origin[:3, 3]
            corners = np.array(list(product(*zip(vertices.min(axis=0), vertices.max(axis=0)))))
            triangles = np.arange(len(vertices), dtype=np.int32).reshape(-1, 3)
            model = fcl.BVHModel()
            model.beginModel(len(vertices), len(triangles))
            model.addSubModel(vertices, triangles)
            model.endModel()
            # For each ancestor axis, bound distance to every point on this link
            # for every possible intervening joint angle, including tracking error.
            bounds = np.zeros(6)
            ancestors = set()
            radius = float(np.linalg.norm(vertices, axis=1).max())
            child = asset.link
            while child in joints:
                joint = joints[child]
                name = joint.get("name")
                if name in geometry.model.joint_names:
                    index = geometry.model.joint_names.index(name)
                    ancestors.add(index)
                    bounds[index] = radius
                radius += float(np.linalg.norm(geometry._origin(joint)[:3, 3]))
                child = joint.find("parent").get("link")
            for name in geometry.installation.robot.arms:
                self.objects.append({"arm": name, "link": asset.link, "vertices": vertices,
                    "geometry": model, "object": fcl.CollisionObject(model), "bounds": bounds,
                    "ancestors": ancestors, "corners": corners})

    def check(self, positions, uncertainty):
        g = self.geometry
        cal = g.installation.calibration
        transforms = {name: g.links_at(name, q) for name, q in positions.items()}
        objects = [obj for obj in self.objects if obj["arm"] in positions]
        errors = {name: np.asarray((uncertainty or {}).get(name, np.zeros(6)), dtype=float)
                  for name in positions}
        if any(v.shape != (6,) or not np.isfinite(v).all() or (v < 0).any() for v in errors.values()):
            raise ValueError("Mesh uncertainty requires six nonnegative joint bounds")
        for obj in objects:
            name, link = obj["arm"], obj["link"]
            t = np.asarray(cal.arms[name].T_world_from_base) @ transforms[name][link]
            obj["object"].setTransform(self.fcl.Transform(t[:3, :3], t[:3, 3]))
            corners = obj["corners"] @ t[:3, :3].T + t[:3, 3]
            obj["aabb"] = (corners.min(axis=0), corners.max(axis=0))
            if link not in g.collision.table_exempt_links:
                lowest = float(np.min(obj["vertices"] @ t[2, :3]) + t[2, 3])
                if lowest - obj["bounds"] @ errors[name] <= cal.table_height_m + cal.collision_margin_m:
                    raise DeviceError(ErrorCode.LIMIT_VIOLATION, f"{name}/{link}: mesh intersects table clearance")
        for a, b in combinations(objects, 2):
            same = a["arm"] == b["arm"]
            if same and (a["link"] == b["link"] or frozenset((a["link"], b["link"])) in g.ignored):
                continue
            bounds_a, bounds_b = a["bounds"].copy(), b["bounds"].copy()
            if same:
                # Common ancestor motion is a shared rigid transform and cannot
                # change the distance between these two surfaces.
                for index in a["ancestors"] & b["ancestors"]:
                    bounds_a[index] = bounds_b[index] = 0.
            inflation = bounds_a @ errors[a["arm"]] + bounds_b @ errors[b["arm"]]
            margin = (cal.self_collision_margin_m if same and cal.self_collision_margin_m is not None
                      else cal.collision_margin_m)
            # AABB distance is a lower bound on mesh distance. Only skip exact
            # queries when the enclosing boxes already prove enough clearance.
            alo, ahi = a["aabb"]
            blo, bhi = b["aabb"]
            separation = np.maximum(0., np.maximum(alo - bhi, blo - ahi))
            if np.linalg.norm(separation) > margin + inflation:
                continue
            distance = self.fcl.distance(a["object"], b["object"],
                self.fcl.DistanceRequest(), self.fcl.DistanceResult())
            if not np.isfinite(distance) or distance <= margin + inflation:
                raise DeviceError(ErrorCode.LIMIT_VIOLATION,
                    f"{a['arm']}/{a['link']} to {b['arm']}/{b['link']}: mesh clearance failed")
