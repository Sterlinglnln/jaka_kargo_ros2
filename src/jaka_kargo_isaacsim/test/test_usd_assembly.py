"""USD authoring tests; these do not run PhysX or validate physical motion."""
import json
import os
from pathlib import Path
import sys

import pytest

pytest.importorskip("pxr")
from pxr import Plug
if os.environ.get("KARGO_PHYSX_PLUGIN_DIR"):
    Plug.Registry().RegisterPlugins(os.environ["KARGO_PHYSX_PLUGIN_DIR"])
from pxr import Gf, Usd, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/jaka_kargo_isaacsim/scripts"))
from assembly_scene import audit_robot, configure_robot_drives, create_scene, set_grasp, PART_PATH, JOINT_PATH, world_matrix

CONFIG = json.loads((ROOT / "src/jaka_kargo_isaacsim/config/assembly_scene.json").read_text())


@pytest.fixture
def stage():
    result = Usd.Stage.Open(str(ROOT / "src/jaka_kargo_description/urdf/jaka_kargo/jaka_kargo_moveit.usd"))
    result.SetEditTarget(result.GetSessionLayer())
    return result


def test_model_contract_and_reject_wrong_limit(stage):
    path = str(ROOT / "src/jaka_kargo_description/urdf/jaka_kargo.urdf")
    assert len(audit_robot(stage, path, CONFIG)) == 21
    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(CONFIG["robot_prim"] + "/joints/arm_lj1"))
    joint.GetUpperLimitAttr().Set(10.0)
    with pytest.raises(RuntimeError, match="limit mismatch"):
        audit_robot(stage, path, CONFIG)


def test_fixed_joint_lifecycle_without_teleport(stage):
    original = stage.GetRootLayer().ExportToString()
    create_scene(stage, CONFIG, str(ROOT / "src/jaka_kargo_moveit_config/config/jaka_kargo.srdf"))
    part = stage.GetPrimAtPath(PART_PATH)
    assert part.HasAPI(UsdPhysics.RigidBodyAPI)
    assert part.HasAPI(UsdPhysics.CollisionAPI)
    assert UsdPhysics.MassAPI(part).GetMassAttr().Get() == pytest.approx(0.2)
    assert all(v > 0 for v in UsdPhysics.MassAPI(part).GetDiagonalInertiaAttr().Get())
    # Position the virtual tool in this authoring fixture, never the part.
    flange = UsdGeom.Xformable(stage.GetPrimAtPath(CONFIG["flange_prim"]))
    matrix = Gf.Matrix4d(1.0)
    position = CONFIG["objects"]["part_a"]["position"]
    half = CONFIG["objects"]["part_a"]["size"][2] / 2
    matrix.SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), 180))
    matrix.SetTranslateOnly(Gf.Vec3d(position[0], position[1], position[2] + half + CONFIG["tcp_offset"][2]))
    flange.MakeMatrixXform().Set(matrix)
    before = world_matrix(stage, PART_PATH)
    set_grasp(stage, CONFIG, True)
    joint = UsdPhysics.FixedJoint(stage.GetPrimAtPath(JOINT_PATH))
    assert joint.GetExcludeFromArticulationAttr().Get()
    assert joint.GetBody0Rel().GetTargets()[0] == CONFIG["flange_prim"]
    assert joint.GetBody1Rel().GetTargets()[0] == PART_PATH
    body0 = world_matrix(stage, CONFIG["flange_prim"])
    anchor = body0.Transform(Gf.Vec3d(joint.GetLocalPos0Attr().Get()))
    assert (anchor - before.ExtractTranslation()).GetLength() < 1e-6
    local0 = Gf.Matrix4d(1.0)
    local0.SetRotate(Gf.Rotation(Gf.Quatd(joint.GetLocalRot0Attr().Get())))
    local0.SetTranslateOnly(Gf.Vec3d(joint.GetLocalPos0Attr().Get()))
    assert Gf.IsClose(local0 * body0, before, 1e-6)
    assert world_matrix(stage, PART_PATH) == before
    set_grasp(stage, CONFIG, False)
    assert not stage.GetPrimAtPath(JOINT_PATH)
    assert world_matrix(stage, PART_PATH) == before
    assert stage.GetRootLayer().ExportToString() == original


def test_reject_grasp_at_distance(stage):
    create_scene(stage, CONFIG, str(ROOT / "src/jaka_kargo_moveit_config/config/jaka_kargo.srdf"))
    with pytest.raises(RuntimeError, match="too far"):
        set_grasp(stage, CONFIG, True)
    assert not stage.GetPrimAtPath(JOINT_PATH)


def test_session_holding_drives_units_and_effort(stage):
    path = str(ROOT / "src/jaka_kargo_description/urdf/jaka_kargo.urdf")
    names = audit_robot(stage, path, CONFIG)
    root_before = stage.GetRootLayer().ExportToString()
    joint = stage.GetPrimAtPath(CONFIG["robot_prim"] + "/joints/arm_lj5")
    old_force = UsdPhysics.DriveAPI(joint, "angular").GetMaxForceAttr().Get()
    configure_robot_drives(stage, CONFIG, names)
    drive = UsdPhysics.DriveAPI(joint, "angular")
    import math
    assert drive.GetStiffnessAttr().Get() == pytest.approx(CONFIG["joint_drive"]["stiffness"] * math.pi / 180)
    assert drive.GetDampingAttr().Get() == pytest.approx(CONFIG["joint_drive"]["damping"] * math.pi / 180)
    assert drive.GetMaxForceAttr().Get() == old_force
    assert drive.GetTargetPositionAttr().Get() == 0.0
    linear = UsdPhysics.DriveAPI(stage.GetPrimAtPath(CONFIG["robot_prim"] + "/joints/body_j1"), "linear")
    assert linear.GetStiffnessAttr().Get() == CONFIG["joint_drive"]["stiffness"]
    assert stage.GetRootLayer().ExportToString() == root_before


def test_concave_base_keeps_right_arm_collision_check_enabled(stage):
    original = stage.GetRootLayer().ExportToString()
    create_scene(stage, CONFIG, str(ROOT / "src/jaka_kargo_moveit_config/config/jaka_kargo.srdf"))
    base = stage.GetPrimAtPath(CONFIG["robot_prim"] + "/base_link/collisions")
    assert UsdPhysics.MeshCollisionAPI(base).GetApproximationAttr().Get() == "convexDecomposition"
    assert UsdPhysics.CollisionAPI(base).GetCollisionEnabledAttr().Get()
    arm = stage.GetPrimAtPath(CONFIG["robot_prim"] + "/arm_rj2_link")
    excluded = UsdPhysics.FilteredPairsAPI(arm).GetFilteredPairsRel().GetTargets()
    assert CONFIG["robot_prim"] + "/base_link" not in [str(p) for p in excluded]
    assert stage.GetRootLayer().ExportToString() == original
