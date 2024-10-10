# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.16.4
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# # Authoring a Multibody Simulation For instructions on how to run these
# tutorial notebooks, please see the [index](./index.ipynb).
#

# This tutorial provides some tools to help you create a new scene description
# file that can be parsed into Drake's multibody physics engine
# (MultibodyPlant) and geometry engine (SceneGraph).

# ## Scene file formats: URDF and SDFormat
#
# The most important formats for creating multibody scenarios in Drake are the
# [Unified Robot Description Format (URDF)](http://wiki.ros.org/urdf) and the
# [Simulation Description Format (SDFormat)](http://sdformat.org/).
#
# They are both XML formats to describe robots or objects for robot simulators
# or visualization, and are fairly similar in syntax.
#
# In a high-level sense, you express different components of your robot using
# `<link>` tags and connect them via `<joint>` tags. Each `<link>` has three
# major subtags, `<visual>`, `<collision>`, and `<inertial>`, for its
# visualization, planning/collision checking, and dynamics aspects. For
# `<visual>` and `<collision>`, you can either use primitives (box, sphere,
# cylinder, etc.) or meshes (.gltf or .obj) to represent the underlying
# geometry.
#
# Here are some useful resources specifically for
# [URDF](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch)
# and [SDFormat](https://classic.gazebosim.org/tutorials?tut=build_model)
# creation.
#
# ### URDF vs. SDFormat
#
# While URDF is the standardized format in ROS, it's lacking many features to
# describe a more complex scene. For example, URDF can only specify the
# kinematic and dynamic properties of a single robot in isolation. It can't
# specify joint loops and friction properties. Additionally, it can't specify
# things that are not robots, such as lights, heightmaps, etc.
#
# SDFormat was created to solve the shortcomings of URDF. SDFormat is a
# complete description for everything from the world level down to the robot
# level. This scalability makes it more suitable for sophisticated simulations.
#
# This tutorial will primarily focus on leveraging SDFormat, but the
# differences in using URDF should be minimal with some syntax changes.
#
# ### Mesh file formats
#
# You can use a mesh file for any of your robot `<link>` entries. The file
# formats that Drake supports (and the nature of that support) is [documented
# here](https://drake.mit.edu/doxygen_cxx/group__geometry__file__formats.html).
# Generally, glTF (`.gltf`) is the preferred format in Drake for
# visualization. OBJ (`.obj`) is another well-supported alternative. If you
# have other file formats, [Meshlab](https://www.meshlab.net/), an open-source
# software, is a handy tool to convert common formats to a `.obj`.

# +
# Import some basic libraries and functions for this tutorial.
import numpy as np
import os

from pydrake.common import temp_directory
from pydrake.geometry import SceneGraphConfig, StartMeshcat
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer
# -

# Start the visualizer. The cell will output an HTTP link after the execution.
# Click the link and a MeshCat tab should appear in your browser.
meshcat = StartMeshcat()

# ## Viewing models
#
# *Make sure you have the MeshCat tab opened in your browser; the link is shown
# *immediately above.*
#
# Drake provides a `ModelVisualizer` class to visualize models
# interactively. This class will help as we start to produce our own robot
# description files, or port description files over from another
# simulator. We'll show examples in the cells below, using a couple of
# pre-existing models provided by Drake.
#
# After running each of the two example cells, switch to the MeshCat tab to see
# the robot. Click **Open Controls** to unfold the control panel. Try adjusting
# the sliding bars to observe the kinematics of the robot.
#
# In the control panel, unfold the **▶ Scene / ▶ drake** menu. By default, only
# the "illustration" geomtry is displayed (the Drake name for visual
# geometry). Toggle the "proximity" checkbox to also show the collision
# geometry (in red), or the "inertia" checkbox to also show each body's
# equivalent inertia ellipsoid (in blue). Use the α sliders to adjust the
# transparancy of the geometry. When debugging a simulation, it's important to
# keep these extra views in mind; usually, the illustration geometry does not
# tell the full story of simulated dynamics.

# +
# First we'll choose one of Drake's example model files, a KUKA iiwa arm.
iiwa7_model_url = (
    "package://drake_models/iiwa_description/sdf/"
    "iiwa7_with_box_collision.sdf")

# Create a model visualizer and add the robot arm.
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModels(url=iiwa7_model_url)

# When this notebook is run in test mode it needs to stop execution without
# user interaction. For interactive model visualization you won't normally
# need the 'loop_once' flag.
test_mode = True if "TEST_SRCDIR" in os.environ else False

# Start the interactive visualizer.
# Click the "Stop Running" button in MeshCat when you're finished.
visualizer.Run(loop_once=test_mode)

# +
# Choose another one of Drake's example model files, a Schunk WSG gripper.
schunk_wsg50_model_url = (
    "package://drake_models/wsg_50_description/sdf/"
    "schunk_wsg_50_with_tip.sdf")

# Create a NEW model visualizer and add the robot gripper.
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModels(url=schunk_wsg50_model_url)

# Start the interactive visualizer.
# Click the "Stop Running" button in MeshCat when you're finished.
visualizer.Run(loop_once=test_mode)
# -

# ## Creating custom models Besides loading the existing SDFormat files in
# Drake, you can also create your own SDFormat model and visualize it in this
# tutorial. The data can be in a file or in a string.
#
# We can create a very simple SDFormat that contains one model with a single
# link. Inside the link, we declare the mass and inertia properties, along with
# a primitive cylinder for the visual and collision geometries.
#
# You can modify the snippet below to change the size or material property of
# the cylinder.

# Define a simple cylinder model.
cylinder_sdf = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="cylinder">
    <pose>0 0 0 0 0 0</pose>
    <link name="cylinder_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.005833</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.005833</iyy>
          <iyz>0.0</iyz>
          <izz>0.005</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.2</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.2</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

# In addition to the `AddModels` method, the `ModelVisualizer` class provides
# access to its `Parser` object; you can access the full parser API to add
# models, e.g., from the string buffer we just created.

# +
# Visualize the cylinder from the SDFormat string you just defined.
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModelsFromString(cylinder_sdf, "sdf")

# Click the "Stop Running" button in MeshCat when you're finished.
visualizer.Run(loop_once=test_mode)
# -

# ### Visual and collision geometry
#
# In the KUKA arm example, if you toggle the `drake/proximity` checkbox in the
# MeshCat control panel a couple of times, you should see red boxes enveloping
# the KUKA arm appear and disappear. Those are the collision geometries defined
# in `iiwa7_with_box_collision.sdf` that are usually consumed by a motion
# planning or collision checking module when running the simulation.
#
# Even though we can use the same mesh to represent both the visual and
# collision geometry, approximating a complex mesh, like the KUKA arm, by
# primitive shapes can reduce the computation enormously. It's easier to check
# whether two cylinders collide than two irregular cylinder-like meshes. For
# that reason, we tend to load mesh files as the visual geometry but utilize
# various primitives as the collision geometry.
#
# ### Define collision geometry for your model
#
# As collision geometry is merely an approximation for the actual shape of your
# model, we want the approximation to be reasonably close to reality. A rule of
# thumb is to completely envelop the actual shape but not inflate it too
# much. For example, rather than trying to cover an L-shape model with one
# giant box, using two boxes or cylinders can actually better represent the
# shape.
#
# It's a balancing act between the fidelity of the approximation and the
# computation cycles saved. When in doubt, start with a rough approximation
# around the actual shape and see if any undesired behavior is introduced,
# e.g., the robot thinks it's in a collision when it's apparently not. Identify
# the questionable part of the collision geometry and replace it with a more
# accurate approximation, and then iterate.
#
# ### Creating an SDFormat wrapper around a mesh file
#
# You might have a mesh file of an object to manipulate and want to add it to a
# simulation. The easiest option is to pass the OBJ file to `Parser.AddModels`
# directly. Parsing it directly will use default assumptions for scale, mass,
# etc.
#
# In case those defaults are insufficient, you should create an SDFormat
# wrapper file to specify the additional properties (mass, inertia, scale,
# compliance, etc.) and load that file instead. You can use the
# [pydrake.multibody.mesh_to_model](https://drake.mit.edu/pydrake/pydrake.multibody.mesh_to_model.html)
# command-line tool to generate a baseline SDFormat file that you can then
# further customize.
#
# Another interesting up-and-coming option is
# [obj2mjcf](https://github.com/kevinzakka/obj2mjcf/), which also does mesh
# reprocessing. Drake can load MuJoCo XML files, but does not yet quite support
# enough of the MuJoCo file format to inter-operate well with obj2mjcf. You
# might be able to get it working with some fiddling.
#
# ### Use a mesh as collision geometry
#
# In some cases you need to have a detailed collision geometry for your
# simulation, e.g., in the case of dexterous manipulation for objects with an
# irregular shape, it might be justifiable to use a mesh as the collision
# geometry directly.
#
# When an OBJ mesh is served as the collision geometry for a basic contact
# model, i.e., the point contact model, Drake will internally compute the
# convex hull of the mesh and use that instead. If you need a non-convex
# collision geometry, it's suggested to decompose your mesh to various convex
# shapes via a convex decomposition tool. There are many similar tools
# available that are mostly thin wrappers on
# [V-HACD](https://github.com/kmammou/v-hacd/). Among all,
# [convex_decomp_to_sdf](https://github.com/gizatt/convex_decomp_to_sdf) is a
# wrapper that we often use for Drake.
#
# However, for a more complex contact model that Drake provides, such as the
# hydroelastic contact model, Drake can directly utilize the actual mesh for
# its contact force computation. Refer to [Hydroelastic user
# guide](https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html)
# for more information.

# ### Drake extensions to SDFormat/URDF
#
# Hopefully, you now have a clear picture of how to create, load, and visualize
# basic SDFormat and URDF models in Drake via MeshCat.
#
# In Drake, we extend URDF and SDFormat to allow access to Drake-specific
# features by adding Drake's custom tags. In the following example,
# `drake:compliant_hydroelastic` custom tag is added under the `collision` tag
# to declare a different contact model for a particular geometry. On the other
# hand, there are also features in both formats that Drake's parser doesn't
# support. The parser will either issue a warning, ignore it silently, or a
# combination of both.
#
# Considering this is a more advanced topic, check [Drake's
# documentation](https://drake.mit.edu/doxygen_cxx/group__multibody__parsing.html)
# for a full list of supported and unsupported tags in both formats.
#
# ```
# <link name="example_link">
#   <inertial>
#     ...
#   </inertial>
#   <visual name="example_visual">
#     ...
#   </visual>
#   <collision name="example_collision">
#     <pose>0 0 0 0 0 0</pose>
#     <geometry>
#       ...
#     </geometry>
#     <drake:proximity_properties>
#       ...
#       <drake:compliant_hydroelastic/>
#     </drake:proximity_properties>
#   </collision>
# </link>
# ```

# ## Creating (or porting) a "scene" with multiple robots/objects
#
# Finally, we are going to look at a more realistic simulation that contains
# multiple objects interacting with each other. In the simulation, we will load
# three objects, i.e., a cracker box from Drake, and a custom cylinder and
# table we created in this tutorial.
#
# At the beginning of the simulation, two objects are posed at certain heights,
# and then they free fall to the tabletop with gravity.
#
# ### Create a simplified table
#
# This is similar to the cylinder example above but here we create and save the
# XML content to an SDFormat file to use in our simulation.

# +
# Create a Drake temporary directory to store files.
# Note: this tutorial will create a temporary file (table_top.sdf)
# in the `/tmp/robotlocomotion_drake_xxxxxx` directory.
temp_dir = temp_directory()

# Create a table top SDFormat model.
table_top_sdf_file = os.path.join(temp_dir, "table_top.sdf")
table_top_sdf = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="table_top">
    <link name="table_top_link">
      <visual name="visual">
        <pose>0 0 0.445 0 0 0</pose>
        <geometry>
          <box>
            <size>0.55 1.1 0.05</size>
          </box>
        </geometry>
        <material>
         <diffuse>0.9 0.8 0.7 1.0</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <pose>0 0 0.445  0 0 0</pose>
        <geometry>
          <box>
            <size>0.55 1.1 0.05</size>
          </box>
        </geometry>
      </collision>
    </link>
    <frame name="table_top_center">
      <pose relative_to="table_top_link">0 0 0.47 0 0 0</pose>
    </frame>
  </model>
</sdf>

"""

with open(table_top_sdf_file, "w") as f:
    f.write(table_top_sdf)


# -

# ### Drake terminology
#
# In Drake, a
# [`System`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_system.html)
# is the building block that has input/output ports to connect with other
# Systems. For example, MultibodyPlant and SceneGraph are both Systems. A
# [`Diagram`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_diagram.html)
# is used to represent a meta-system that may have several interconnected
# Systems that function collectively.
#
# Each System and Diagram has its own
# [`Context`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_context.html)
# to represent its state and will be updated as the simulation progresses.
#
# The Context and the Diagram are the only two pieces of information a
# simulator needs to run. Given the same Context of a Diagram, the simulation
# should be completely deterministic and repeatable.
#
# Refer to [Modeling Dynamical
# Systems](https://github.com/RobotLocomotion/drake/blob/master/tutorials/dynamical_systems.ipynb),
# which covers more details on the relevant topics.
#
# *Note: Drake uses [Doxygen C++
# *Documentation](https://drake.mit.edu/doxygen_cxx/index.html) as the primary
# *API documentation, but it also provides [Python API
# *documentation](https://drake.mit.edu/pydrake/) for Python users.*
#
# ### Load different objects into a "scene"
#
# In the `create_scene()` function, we first create a
# `pydrake.multibody.MultibodyPlant`, a `pydrake.multibody.SceneGraph`, and a
# `pydrake.multibody.parsing.Parser`.
#
# The parser is used to load the models into a MultibodyPlant. One thing to
# note in this example is we fix (or "weld") the table with respect to the
# world while treating the cracker box and the cylinder as free bodies. Once
# the MultibodyPlant is all set up properly, the function returns a diagram
# that a Drake Simulator consumes (a default context is used in this case).

def create_scene(sim_time_step):
    # Clean up the Meshcat instance.
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=sim_time_step)
    parser = Parser(plant)

    # Loading models.
    # Load the table top and the cylinder we created.
    parser.AddModelsFromString(cylinder_sdf, "sdf")
    parser.AddModels(table_top_sdf_file)
    # Load a cracker box from Drake.
    parser.AddModels(
        url="package://drake_models/ycb/003_cracker_box.sdf")
    # Load an OBJ file from Drake, with no SDFormat wrapper file. In this case,
    # the mass and inertia are inferred based on the volume of the mesh as if
    # it were filled with water, and the mesh is used for both collision and
    # visual geometry.
    parser.AddModels(
        url="package://drake_models/ycb/meshes/004_sugar_box_textured.obj")

    # Weld the table to the world so that it's fixed during the simulation.
    table_frame = plant.GetFrameByName("table_top_center")
    plant.WeldFrames(plant.world_frame(), table_frame)
    # Finalize the plant after loading the scene.
    plant.Finalize()
    # We use the default context to calculate the transformation of the table
    # in world frame but this is NOT the context the Diagram consumes.
    plant_context = plant.CreateDefaultContext()

    # Set the initial pose for the free bodies, i.e., the custom cylinder,
    # the cracker box, and the sugar box.
    cylinder = plant.GetBodyByName("cylinder_link")
    X_WorldTable = table_frame.CalcPoseInWorld(plant_context)
    X_TableCylinder = RigidTransform(
        RollPitchYaw(np.asarray([90, 0, 0]) * np.pi / 180), p=[0, 0, 0.5])
    X_WorldCylinder = X_WorldTable.multiply(X_TableCylinder)
    plant.SetDefaultFreeBodyPose(cylinder, X_WorldCylinder)

    cracker_box = plant.GetBodyByName("base_link_cracker")
    X_TableCracker = RigidTransform(
        RollPitchYaw(np.asarray([45, 30, 0]) * np.pi / 180), p=[0, 0, 0.8])
    X_WorldCracker = X_WorldTable.multiply(X_TableCracker)
    plant.SetDefaultFreeBodyPose(cracker_box, X_WorldCracker)

    sugar_box = plant.GetBodyByName("004_sugar_box_textured")
    X_TableSugar = RigidTransform(p=[0, -0.25, 0.8])
    X_WorldSugar = X_WorldTable.multiply(X_TableSugar)
    plant.SetDefaultFreeBodyPose(sugar_box, X_WorldSugar)

    # Add visualization to see the geometries.
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    diagram = builder.Build()
    return diagram


# ## Running a simple simulation
#
# We have everything we need to launch the simulator! Run the following code
# block to start the simulation and visualize it in your MeshCat tab.
#
# This simple simulation represents a passive system in that the objects fall
# purely due to gravity without other power sources. Did they do what you
# expect? You can also use the **reset** and **play** buttons in the MeshCat
# tab to re-run the simulation.
#
# Try adjusting the `sim_time_step` and re-run the simulation. Start with a
# small value and increase it gradually to see if that changes the behavior.

# +
def initialize_simulation(diagram):
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.)
    return simulator


def run_simulation(sim_time_step):
    diagram = create_scene(sim_time_step)
    simulator = initialize_simulation(diagram)
    meshcat.StartRecording()
    finish_time = 0.1 if test_mode else 2.0
    simulator.AdvanceTo(finish_time)
    meshcat.PublishRecording()


# Run the simulation with a small time step. Try gradually increasing it!
run_simulation(sim_time_step=0.0001)


# -

# ## Get a quick preview of hydroelastic contact
#
# With a small change to configuration, we can run the same simulation with
# hydroelastic contact for all bodies, without editing the model files. This
# method applies reasonable defaults everywhere, but it may not be optimal for
# a given scene. To get finer control, editing model files will then be
# necessary. See the [basic hydroelastic
# contact](http://localhost:8888/notebooks/hydroelastic_contact_basics.ipynb)
# tutorial for more information.

# +
def run_all_hydroelastic_simulation(sim_time_step):
    diagram = create_scene(sim_time_step)

    # SceneGraphConfig lets us change default proximity properties for
    # everything.
    scene_graph = diagram.GetSubsystemByName("scene_graph")
    config = SceneGraphConfig()
    config.default_proximity_properties.compliance_type = "compliant"
    config.default_proximity_properties.hunt_crossley_dissipation = 1.0
    scene_graph.set_config(config)

    simulator = initialize_simulation(diagram)
    meshcat.StartRecording()
    finish_time = 0.1 if test_mode else 2.0
    simulator.AdvanceTo(finish_time)
    meshcat.PublishRecording()


# Run the simulation, similar to the one above. Notice that the contact physics
# are now different, and the that contact force arrows are red, instead of
# green.
run_all_hydroelastic_simulation(sim_time_step=0.002)
# -

# ## Debugging your MultibodyPlant/SceneGraph
#
# Sometimes people get surprising results, e.g., unreasonable behaviors in
# simulation or program crash, due to the discrepancy between the simulation
# setup and the real-world physics properties.
#
# ### Debugging the inertial property One common scenario for that is a lack of
# inertial properties for some of the simulated objects. The time step of the
# simulation may become extremely small (e.g., < 0.001s) due to the poorly
# specified system. Alternatively, you may receive an error message about
# `Delta > 0` or a warning that the inertial matrix is not physically valid.
#
# Double-check the inertial properties, especially if the dynamic behavior is
# the focus of the simulation. To visually assess the inertial properties, use
# the inertia visualization layer in `ModelVisualizer`. In the controls, under
# `Scene >> drake`, check the `inertia` checkbox.
#
# To automatically calculate inertias for a URDF or SDFormat model, try the
# [fix_inertia](https://drake.mit.edu/pydrake/pydrake.multibody.fix_inertia.html)
# tool.
#
# ### Debugging the mass property You don't need to specify the mass of an
# object if it's welded to the world. However, an error will be triggered if
# you have a movable object with zero mass as the simulation is not yet fully
# specified.
#
# Hint: Does the mass/inertia of the movable objects seem reasonable? Try
# modifying them and rerun the simulation to observe changes.

# ## Next steps
#
# This tutorial helps you set up the physics (MultibodyPlant) and geometry
# engines (SceneGraph) and visualize the simulation in MechCat. However, most
# robotics simulations require more. Next, you might need to model the sensors,
# the low-level control system, and eventually even the high-level perception,
# planning, and control systems for a real-world robot platform.
#
# Here are some other resources to help you explore further.
#
# - [Drake MultibodyPlant](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html)  # noqa
# - [Drake SceneGraph](https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_scene_graph.html)  # noqa
# - [Introduction to the basic robot pick-and-place using Drake](https://manipulation.csail.mit.edu/pick.html)  # noqa
# - Tutorial on [basic hydroelastic contact](./hydroelastic_contact_basics.ipynb)  # noqa

#
