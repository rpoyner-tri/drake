#from meshcat.servers.zmqserver import start_zmq_server_as_subprocess
from pydrake.all import (
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    FindResourceOrThrow,
    ConnectMeshcatVisualizer,
    Parser,
    RigidTransform,
    RollPitchYaw
)


#proc, zmq_url, web_url = start_zmq_server_as_subprocess()


def visualize_gripper_frames(X_G):

    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=0.0
    )

    parser = Parser(plant, scene_graph)

    for key, pose in X_G.items():
        fn = FindResourceOrThrow('drake/temp/box.sdf')
        g = parser.AddModelFromFile(
            fn,
            f'location_{key}'
        )

        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName('box_link', g),
            pose
        )

    plant.Finalize()

    # meshcat = ConnectMeshcatVisualizer(
    #     builder, scene_graph, zmq_url=zmq_url
    # )
    # meshcat.load()

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    diagram.Publish(context)


transforms = {}

for i in range(5):

    transforms[i] = RigidTransform(
        RollPitchYaw([0., 0., 0.]),
        [0., 0., 0.]
    )

print('Number of frame to draw', len(transforms))
visualize_gripper_frames(transforms)

# while True:
#     continue
