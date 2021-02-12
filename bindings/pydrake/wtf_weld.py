import math
import time

from pydrake.systems.framework import (DiagramBuilder,)
from pydrake.systems.primitives import (ConstantVectorSource,)
from pydrake.geometry import (Box,
                              DrakeVisualizer)
from pydrake.math import (RigidTransform, RollPitchYaw)
from pydrake.multibody.plant import (AddMultibodyPlantSceneGraph, CoulombFriction, Propeller, PropellerInfo)
from pydrake.multibody.tree import (SpatialInertia, RotationalInertia, PlanarJoint)

from pydrake.systems.analysis import Simulator

def make_box(mbp, name):
    inertia = SpatialInertia.MakeFromCentralInertia(1, [0, 0, 0], RotationalInertia(1/600, 1/120, 1/120))
    body = mbp.AddRigidBody(name, inertia)
    shape = Box(1, 0.1, 0.1)
    mbp.RegisterVisualGeometry(
        body=body, X_BG=RigidTransform(), shape=shape, name=f"{name}_visual",
        diffuse_color=[1., 0.64, 0.0, 0.5])
    body_friction = CoulombFriction(static_friction=0.6,
                                    dynamic_friction=0.5)
    mbp.RegisterCollisionGeometry(
        body=body, X_BG=RigidTransform(), shape=shape,
        name="{name}_collision", coulomb_friction=body_friction)
    return body

def make_mbp(builder):
    mbp, sg = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    robot_body = make_box(mbp, "robot")
    planar_joint = mbp.AddJoint(
        PlanarJoint(name="robot_planar_joint",
                    frame_on_parent=mbp.world_frame(),
                    frame_on_child=robot_body.body_frame(),
                    damping=[1,1,1]))

    blocker1 = make_box(mbp, "blocker1")
    mbp.WeldFrames(A=blocker1.body_frame(), B=mbp.world_frame(), X_AB=RigidTransform([2,2,0]))
    return mbp, sg

def add_thrusters(builder, mbp):
    robot_body_index = mbp.GetBodyByName("robot").index()
    robot_tail_forward = RigidTransform(p=[-0.5, 0, 0], rpy=RollPitchYaw([math.pi/2, 0, 0]))
    robot_tail_clockwise = RigidTransform(p=[-0.5, 0, 0], rpy=RollPitchYaw([math.pi/2, 0, math.pi/2]))
    thrusters = builder.AddSystem(
        Propeller([PropellerInfo(robot_body_index, X_BP=robot_tail_forward),
                   PropellerInfo(robot_body_index, X_BP=robot_tail_clockwise)]))
    builder.Connect(thrusters.get_spatial_forces_output_port(), mbp.get_applied_spatial_force_input_port())
    builder.Connect(mbp.get_body_poses_output_port(), thrusters.get_body_poses_input_port())
    return thrusters

def run(sleeptime=0.001, thrust = [10, 2]):
    builder = DiagramBuilder()
    mbp, sg = make_mbp(builder)
    mbp.Finalize()
    DrakeVisualizer.AddToBuilder(builder, sg)

    thrusters = add_thrusters(builder, mbp)
    cmd = builder.AddSystem(ConstantVectorSource(thrust))
    builder.Connect(cmd.get_output_port(), thrusters.get_command_input_port())

    diagram = builder.Build()
    simulator = Simulator(diagram)
    for step in range(1000):
        simulator.AdvanceTo(0.001 * step)
        mbp_context = diagram.GetSubsystemContext(mbp, simulator.get_context())
        time.sleep(sleeptime)

run()
