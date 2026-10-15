"""Optional contact/drive observations for physical startup troubleshooting."""
import json

from pxr import PhysxSchema, UsdPhysics, PhysicsSchemaTools
from omni.physx import get_physx_simulation_interface


class PhysicsDiagnostics:
    def __init__(self, stage):
        self.contacts = {}
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self.on_contact)
        self.last_report = -100.0

    def on_contact(self, headers, data):
        for header in headers:
            if not header.num_contact_data:
                continue
            names = tuple(sorted([str(PhysicsSchemaTools.intToSdfPath(header.actor0)),
                                  str(PhysicsSchemaTools.intToSdfPath(header.actor1))]))
            points = data[header.contact_data_offset:header.contact_data_offset + header.num_contact_data]
            depth = min(float(point.separation) for point in points)
            self.contacts[names] = min(depth, self.contacts.get(names, 0.0))

    def report(self, robot, sim_time):
        if sim_time - self.last_report < 1.0:
            return
        self.last_report = sim_time
        action = robot.get_applied_action()
        kp, kd = robot.get_articulation_controller().get_gains()
        q = robot.get_joint_positions()
        fields = {}
        for name in ['arm_lj1', 'arm_lj2', 'arm_rj1', 'body_j1', 'body_j2']:
            i = robot.get_dof_index(name)
            fields[name] = {'q':float(q[i]), 'target':float(action.joint_positions[i]),
                            'kp':float(kp[i]), 'kd':float(kd[i])}
        pairs = [{'pair':list(pair),'separation':depth} for pair, depth in sorted(self.contacts.items(), key=lambda item:item[1])[:12]]
        print('ASSEMBLY_PHYSICS ' + json.dumps({'time':sim_time,'drives':fields,'contacts':pairs}), flush=True)
        self.contacts.clear()
