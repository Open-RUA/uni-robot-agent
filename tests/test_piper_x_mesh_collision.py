"""Mesh proximity tests use synthetic solids, never hardware acceptance."""
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

pytest.importorskip("fcl")
from piper_fixtures import deployment, write
from uni_robot_agent.integrations.piper_x.config import Installation, Model, digest
from uni_robot_agent.integrations.piper_x.hardware.planning import RobotGeometry


def mesh_rig(tmp_path):
    i = deployment(tmp_path)
    vertices = np.array([[x,y,z] for x in (-.005,.005) for y in (-.005,.005) for z in (-.005,.005)])
    faces = np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                      [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    dtype = np.dtype([("normal","<f4",(3,)),("vertices","<f4",(3,3)),("attribute","<u2")])
    data = np.zeros(len(faces), dtype=dtype)
    data["vertices"] = vertices[faces]
    file = tmp_path / "cube.stl"
    file.write_bytes(bytes(80)+len(faces).to_bytes(4,"little")+data.tobytes())
    root = ET.fromstring(i.urdf.read_bytes())
    assets=[]
    for link in root.findall("link"):
        for col in link.findall("collision"):
            geo=col.find("geometry")
            geo.clear()
            ET.SubElement(geo,"mesh",filename="cube.stl")
            assets.append({"link":link.get("name"),"file":{"path":"cube.stl","sha256":hashlib.sha256(file.read_bytes()).hexdigest()}})
    ET.ElementTree(root).write(i.urdf)
    model=i.model.model_dump(mode="json")
    model["urdf"]["sha256"]=hashlib.sha256(i.urdf.read_bytes()).hexdigest()
    collision=json.loads(i.collision.read_text())
    collision.update(urdf_sha256=model["urdf"]["sha256"],meshes=assets)
    model["collision"]=write(i.collision,collision)
    write(i.model_path,model)
    cal=i.calibration.model_dump(mode="json")
    cal["model_sha256"]=digest(Model.model_validate(model).model_dump(mode="json"))
    robot=i.robot.model_dump(mode="json")
    robot["calibration"]=write(i.calibration_path,cal)
    write(i.robot_path,robot)
    return Installation(i.path)


def test_mesh_checks_real_separation_table_and_cross_arm_overlap(tmp_path):
    i=mesh_rig(tmp_path);g=RobotGeometry(i)
    q={name:np.zeros(6) for name in i.robot.arms}
    g.check(q)
    assert g.mesh_checks is not None
    names=list(q)
    i.calibration.arms[names[1]].T_world_from_base=i.calibration.arms[names[0]].T_world_from_base
    with pytest.raises(Exception,match="mesh clearance"):
        g.check(q)
    i.calibration.table_height_m=1
    with pytest.raises(Exception,match="table"):
        g.check(q)


def test_mesh_hash_is_verified_at_startup(tmp_path):
    i=mesh_rig(tmp_path)
    (tmp_path/"cube.stl").write_bytes(b"wrong")
    with pytest.raises(ValueError,match="hash mismatch"):
        RobotGeometry(i)


def test_mesh_assets_must_cover_every_collision_element(tmp_path):
    i=mesh_rig(tmp_path)
    data=json.loads(i.collision.read_text())
    data["meshes"].pop()
    i.collision.write_text(json.dumps(data))
    with pytest.raises(ValueError,match="exactly once"):
        RobotGeometry(i)


def test_link_displacement_bounds_cover_joint_perturbations(tmp_path):
    i=mesh_rig(tmp_path);g=RobotGeometry(i)
    rng=np.random.default_rng(12)
    name=next(iter(i.robot.arms))
    for _ in range(8):
        q=rng.uniform(-1,1,6);dq=rng.uniform(-.015,.015,6)
        a=g.links_at(name,q);b=g.links_at(name,q+dq)
        for obj in g.mesh_checks.objects:
            if obj["arm"]!=name:continue
            v=obj["vertices"];ta=a[obj["link"]];tb=b[obj["link"]]
            displacement=np.linalg.norm(v@ta[:3,:3].T+ta[:3,3]-v@tb[:3,:3].T-tb[:3,3],axis=1).max()
            assert displacement<=obj["bounds"]@np.abs(dq)+1e-10


def test_mesh_uncertainty_and_nonfinite_states_rejected(tmp_path):
    i=mesh_rig(tmp_path);g=RobotGeometry(i)
    q={n:np.zeros(6) for n in i.robot.arms}
    with pytest.raises(Exception,match="clearance"):
        g.check(q,{n:np.ones(6) for n in q})
    q[next(iter(q))][0]=np.nan
    with pytest.raises(Exception,match="Joint range"):
        g.check(q)
