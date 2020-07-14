import math
import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (DiagramBuilder, MultibodyPlant,
                         MultibodyPositionToGeometryPose, Parser,
                         SceneGraph, Simulator, Solve)
from pydrake.common import FindResourceOrThrow

builder = DiagramBuilder()

plant = builder.AddSystem(MultibodyPlant(time_step=0.0))
scene_graph = builder.AddSystem(SceneGraph())
plant.RegisterAsSourceForSceneGraph(scene_graph)
builder.Connect(plant.get_geometry_poses_output_port(),
                scene_graph.get_source_pose_port(
                    plant.get_source_id()))
builder.Connect(scene_graph.get_query_output_port(),
                plant.get_geometry_query_input_port())


file_name = FindResourceOrThrow("drake/gh13678/models/one_d_hopper.urdf")

Parser(plant).AddModelFromFile(file_name)

plant.Finalize()
diagram = builder.Build()

context = diagram.CreateDefaultContext()

# print(context.get_discrete_state_vector())
print(context)
print(f"groups {context.num_discrete_state_groups()}")
print(f"discrete state {context.get_discrete_state()}, len {context.get_discrete_state().num_groups()}")
