"""In-memory USD assembly scene. No robot model files are saved or rewritten."""
import math
import xml.etree.ElementTree as ET

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema


SCENE_PATH = "/World/Assembly"
PART_PATH = SCENE_PATH + "/part_a"
JOINT_PATH = SCENE_PATH + "/grasp_joint"


def audit_robot(stage, urdf_path, config):
    """Reject a different robot, joint type, scale, or joint limit contract."""
    if UsdGeom.GetStageUpAxis(stage) != "Z" or abs(UsdGeom.GetStageMetersPerUnit(stage) - 1) > 1e-9:
        raise RuntimeError("Assembly requires a Z-up USD stage measured in meters")
    root = stage.GetPrimAtPath(config["articulation_root"])
    if not root or not root.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise RuntimeError("Configured articulation root is missing PhysicsArticulationRootAPI")
    joints = {p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}
    robot = ET.parse(urdf_path).getroot()
    names = ([f"arm_{side}j{i}" for side in ("l", "r") for i in range(1, 8)]
             + [f"body_j{i}" for i in range(1, 5)] + ["agv_x", "agv_y", "agv_yaw"])
    for name in names:
        source = robot.find(f"joint[@name='{name}']")
        prim = joints.get(name)
        if source is None or prim is None:
            raise RuntimeError(f"Missing joint: {name}")
        linear = source.get("type") == "prismatic"
        kind = UsdPhysics.PrismaticJoint if linear else UsdPhysics.RevoluteJoint
        if not prim.IsA(kind):
            raise RuntimeError(f"Joint type mismatch: {name}")
        joint = kind(prim)
        scale = 1.0 if linear else math.pi / 180.0
        for field, attr in (("lower", joint.GetLowerLimitAttr()), ("upper", joint.GetUpperLimitAttr())):
            if abs(attr.Get() * scale - float(source.find("limit").get(field))) > 1e-5:
                raise RuntimeError(f"Joint limit mismatch: {name}/{field}")
        for index, tag in enumerate(("parent", "child")):
            relation = joint.GetBody0Rel() if index == 0 else joint.GetBody1Rel()
            targets = relation.GetTargets()
            if len(targets) != 1 or targets[0].name != source.find(tag).get("link"):
                raise RuntimeError(f"Joint topology mismatch: {name}/{tag}")
    # Match the existing SRDF's collision exclusions in the physical robot too.
    return names


def configure_robot_drives(stage, config, names):
    """Author holding gains before Play, including correct USD angular units.

    Runtime tensor gains alone can be lost when USD changes rebuild an articulation.
    Session-layer gains survive that rebuild without changing the source robot asset.
    """
    gains = config["joint_drive"]
    joints = {p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}
    for name in names:
        prim = joints[name]
        angular = prim.IsA(UsdPhysics.RevoluteJoint)
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular" if angular else "linear")
        # USD angular drives use torque/degree; the tensor API uses torque/radian.
        scale = math.pi / 180.0 if angular else 1.0
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(gains["stiffness"] * scale)
        drive.CreateDampingAttr(gains["damping"] * scale)
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateTargetVelocityAttr(0.0)


def create_scene(stage, config, srdf_path):
    """Static supports and a dynamic cube, with explicit mass and box inertia."""
    old_ground = stage.GetPrimAtPath("/World/flat_plane")
    if old_ground:
        old_ground.SetActive(False)
    # A single hull fills the base CAD's concavities and intersects arm_rj2
    # by ~43 mm at zero. Preserve collision checking with a concave approximation.
    base_collider = stage.GetPrimAtPath(config["robot_prim"] + "/base_link/collisions")
    if not base_collider or not base_collider.HasAPI(UsdPhysics.CollisionAPI):
        raise RuntimeError("Expected the existing merged base collision geometry")
    UsdPhysics.MeshCollisionAPI.Apply(base_collider).CreateApproximationAttr("convexDecomposition")
    decomposition = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(base_collider)
    decomposition.CreateHullVertexLimitAttr(64)
    decomposition.CreateMaxConvexHullsAttr(64)
    decomposition.CreateVoxelResolutionAttr(1000000)
    # Match the scene's small contact offset instead of the importer's inflated shell.
    for prim in stage.Traverse():
        if prim.GetPath().HasPrefix(config["robot_prim"]) and prim.HasAPI(UsdPhysics.CollisionAPI):
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(0.002)
            collision.CreateRestOffsetAttr(0.0)
    UsdGeom.Xform.Define(stage, SCENE_PATH)
    mat = UsdShade.Material.Define(stage, SCENE_PATH + "/material")
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(config["friction"][0])
    api.CreateDynamicFrictionAttr(config["friction"][1])
    api.CreateRestitutionAttr(0.0)
    for name, spec in config["objects"].items():
        cube = UsdGeom.Cube.Define(stage, SCENE_PATH + "/" + name)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*spec["position"]))
        cube.AddScaleOp().Set(Gf.Vec3f(*spec["size"]))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*spec["color"])])
        prim = cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        contact.CreateContactOffsetAttr(0.002)
        contact.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        if "mass" in spec:
            # Keep the rigid body's scale at identity: joint frames and inertia use meters.
            cube.GetOrderedXformOps()[1].Set(Gf.Vec3f(1.0))
            cube.CreateSizeAttr(spec["size"][0])
            if len(set(spec["size"])) != 1:
                raise RuntimeError("The dynamic assembly part must be a cube")
            UsdPhysics.RigidBodyAPI.Apply(prim)
            mass = UsdPhysics.MassAPI.Apply(prim)
            mass.CreateMassAttr(spec["mass"])
            mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0))
            a, b, c = spec["size"]
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(
                spec["mass"] * (b*b + c*c) / 12,
                spec["mass"] * (a*a + c*c) / 12,
                spec["mass"] * (a*a + b*b) / 12))
    # Visual target B is non-colliding; the underlying base is the support collider.
    marker = UsdGeom.Cube.Define(stage, SCENE_PATH + "/target_b")
    marker.CreateSizeAttr(1.0)
    base = config["objects"]["assembly_base"]
    marker.AddTranslateOp().Set(Gf.Vec3d(config["target_position"][0], config["target_position"][1],
                                      base["position"][2] + base["size"][2] / 2 + 0.001))
    marker.AddScaleOp().Set(Gf.Vec3f(0.10, 0.10, 0.001))
    marker.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.9, 0.2)])
    robot = config["robot_prim"]
    for exclusion in ET.parse(srdf_path).getroot().findall("disable_collisions"):
        first = stage.GetPrimAtPath(robot + "/" + exclusion.get("link1"))
        second = stage.GetPrimAtPath(robot + "/" + exclusion.get("link2"))
        if first and second:
            UsdPhysics.FilteredPairsAPI.Apply(first).CreateFilteredPairsRel().AddTarget(second.GetPath())
    PhysxSchema.PhysxArticulationAPI.Apply(
        stage.GetPrimAtPath(config["articulation_root"])).CreateEnabledSelfCollisionsAttr(True)


def world_matrix(stage, path):
    return UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def tcp_matrix(stage, config):
    offset = Gf.Matrix4d(1.0)
    offset.SetTranslate(Gf.Vec3d(*config["tcp_offset"]))
    return offset * world_matrix(stage, config["flange_prim"])


def set_grasp(stage, config, attach):
    """Create/remove a maximal fixed joint at the current relative transform.

    No transform of the part is changed during attachment, carrying or release.
    """
    if not attach:
        if stage.GetPrimAtPath(JOINT_PATH):
            stage.RemovePrim(JOINT_PATH)
        return
    if stage.GetPrimAtPath(JOINT_PATH):
        raise RuntimeError("Part is already attached")
    part_world = world_matrix(stage, PART_PATH)
    tcp = tcp_matrix(stage, config).ExtractTranslation()
    # Virtual tool attaches at the top of the cube, not at an arbitrary distance.
    top = part_world.Transform(Gf.Vec3d(0, 0, config["objects"]["part_a"]["size"][2] / 2))
    if (top - tcp).GetLength() > config["grasp_distance"]:
        raise RuntimeError(f"TCP is too far from part top: {(top - tcp).GetLength():.4f} m")
    local = part_world * world_matrix(stage, config["flange_prim"]).GetInverse()
    joint = UsdPhysics.FixedJoint.Define(stage, JOINT_PATH)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(config["flange_prim"])])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(PART_PATH)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(local.ExtractTranslation()))
    joint.CreateLocalRot0Attr(Gf.Quatf(local.ExtractRotationQuat()))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0))
    joint.CreateExcludeFromArticulationAttr(True)
    joint.CreateCollisionEnabledAttr(False)
