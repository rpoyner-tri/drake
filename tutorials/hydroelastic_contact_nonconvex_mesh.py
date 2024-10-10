# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.16.4
#   kernelspec:
#     display_name: Python 3 (ipykernel)
#     language: python
#     name: python3
# ---

# # Hydroelastic Contact: Nonconvex Mesh For instructions on how to run these
# tutorial notebooks, please see the [index](./index.ipynb).
#
# If you are not familiar with Drake's hydroelastic contact, study
# [hydroelastic_contact_basics.ipynb](./hydroelastic_contact_basics.ipynb). You
# can also find more information in Hydroelastic Contact User Guide
# [here.](https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html)

# ## Introduction
#
# This tutorial shows you how to set up simulations using
# compliant-hydroelastic nonconvex meshes. We'll use a simple example of a bell
# pepper dropped onto a bowl on a table top, with all three objects represented
# by compliant-hydroelastic meshes. Contact forces are calculated and
# visualized.

# +
import os
from pathlib import Path

from IPython.display import Code

from pydrake.geometry import StartMeshcat
from pydrake.math import RigidTransform
from pydrake.multibody.meshcat import (ContactVisualizer,
                                       ContactVisualizerParams)
from pydrake.multibody.parsing import PackageMap, Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import (ApplyVisualizationConfig, ModelVisualizer,
                                   VisualizationConfig)
# -

# ## Start MeshCat
#
# See the section [Viewing
# models](./authoring_multibody_simulation.ipynb#Viewing-models) in the
# tutorial [Authoring a Multibody
# Simulation](./authoring_multibody_simulation.ipynb) for an introduction to
# MeshCat.

# Start the visualizer. The cell will output an HTTP link after the execution.
# Click the link and a MeshCat tab should appear in your browser.
meshcat = StartMeshcat()

# ## Create compliant-hydroelastic bell pepper in SDFormat
#
# *Make sure you have the MeshCat tab opened in your browser; the link is shown
# *immediately above.*
#
# We will load a compliant-hydroelastic bell pepper from an SDFormat file. We
# will show the file for you to read, and then use `ModelVisualizer` to display
# it.
#
# The file specifies inertia in the `<inertial>` block. See
# [mesh_to_model](https://drake.mit.edu/pydrake/pydrake.multibody.mesh_to_model.html)
# to compute the inertia matrix.
#
# The file specifies visual geometry using a triangle surface mesh and a
# collision geometry using a tetrahedral volume mesh from a VTK file.
#
# The file's `<drake:proximity_properties>` stanza will control hydroelastic
# contacts. Look in particular at the `<drake:hydroelastic_modulus>`.
#
# In the MeshCat tab, you should toggle the "proximity" checkbox to show the
# collision geometry, which is the tetrahedral mesh that fits the visual
# geometry's triangle mesh.  See the section *Viewing models* in
# [authoring_multibody_simulation.ipynb](./authoring_multibody_simulation.ipynb)
# for more details.

# Show the contents of the SDFormat file.
bell_pepper_url = "package://drake_models/veggies/yellow_bell_pepper_no_stem_low.sdf"  # noqa
bell_pepper_str =
Path(PackageMap().ResolveUrl(bell_pepper_url)).read_text(encoding="utf-8")
Code(bell_pepper_str, language="xml")

# Visualize the SDFormat file you just defined.
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.AddModels(url=bell_pepper_url)
visualizer.Run(loop_once=True)

# ## Create compliant-hydroelastic bowl in URDF
#
# We will load a compliant-hydroelastic bowl with URDF file. We will show the
# file for you to read, and then use `ModelVisualizer` to display it.
#
# The file specifies inertia in the `<inertial>` block. See
# [mesh_to_model](https://drake.mit.edu/pydrake/pydrake.multibody.mesh_to_model.html)
# to compute the inertia matrix.
#
# The file specifies visual geometry using a triangle surface mesh and a
# collision geometry using a tetrahedral volume mesh from a VTK file.
#
# In the `<drake:proximity_properties>` block, the
# `<drake:hydroelastic_modulus>` is set to 1e7 Pascals, so the bowl is stiffer
# than the bell pepper.

# Show the contents of the URDF file.
bowl_url = "package://drake_models/dishes/evo_bowl_compliant.urdf"
bowl_str = Path(PackageMap().ResolveUrl(bowl_url)).read_text(encoding="utf-8")
Code(bowl_str, language="xml")

# (Drake users can ignore this cell. When Drake regression testing runs in CI,
# we need to use a coarser tetrahedral mesh to improve debug performance.)
test_mode = "TEST_SRCDIR" in os.environ
if test_mode:
    bowl_str = bowl_str.replace("evo_bowl_fine44k.vtk",
                                "evo_bowl_coarse3k.vtk")

# Visualize the URDF file.
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModels(file_contents=bowl_str, file_type="urdf")
visualizer.Run(loop_once=True)

# ## Create compliant-hydroelastic table top in URDF
#
# The following URDF file specifies a compliant-hydroelastic box for a table
# top.  We demonstrate how to set relevant hydroelastic properties in URDF;
# however, Drake prefers SDFormat to URDF.
#
# Both the `<visual>` and `<collision>` geometries are boxes of the same size.
#
# In the `<drake:proximity_properties>` block, we will set
# `<drake:hydroelastic_modulus>` to 1e7 Pascals.
#
# We do not specify the inertia matrix of the table top because, in the next
# section when we set up `Diagram`, we will fix the table top to the world
# frame. It will not move.

# Show the contents of the URDF file.
table_top_url = "package://drake_models/dishes/table_top.urdf"
table_top_str = Path(PackageMap().ResolveUrl(table_top_url)).read_text(
    encoding="utf-8")
Code(table_top_str, language="xml")

# Visualize the URDF file.
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.AddModels(url=table_top_url)
visualizer.Run(loop_once=True)


# ## Create Diagram of the scene
#
# The function `add_scene()` below will create a scene using the assets that we
# created. It will use `Parser` to add the URDF and SDFormat strings into the
# scene. After this step, the next section will add visualization.

def add_scene(time_step):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(
            time_step=time_step,
            discrete_contact_approximation="lagged"),
        builder)
    parser = Parser(plant)

    # Load the assets that we created.
    parser.AddModels(url=bell_pepper_url)
    parser.AddModels(file_contents=bowl_str, file_type="urdf")
    parser.AddModels(url=table_top_url)

    # Weld the table top to the world so that it's fixed during simulation.
    # The top surface passes the world's origin.
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("top_surface"))

    # Finalize the plant after loading the scene.
    plant.Finalize()

    # Place the bowl on top of the table.
    X_WB = RigidTransform(p=[0, 0, 0.03])
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("bowl"), X_WB)

    # Drop the bell pepper from above the rim of the bowl.
    X_WC = RigidTransform(p=[-0.06, 0, 0.30])
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName(
        "yellow_bell_pepper_no_stem"), X_WC)

    return builder, plant


# ## Set up visualization
#
# The function `add_viz()` below will create visualization. First we will call
# `ApplyVisualizationConfig()` to visualize our assets. At this step we will
# set `publish_contacts=False`, so we can customize contact visualization
# afterwards.
#
# To visualize contact result, we will add `ContactVisualizer` with
# `newtons_per_meter= 20` and `newtons_meters_per_meter= 0.1`. It will draw a
# red arrow of length 1 meter for each force of 20 newtons and a blue arrow of
# length 1 meter for each torque of 0.1 newton\*meters. The next section will
# run the simulation.

def add_viz(builder, plant):
    ApplyVisualizationConfig(
        builder=builder, meshcat=meshcat,
        config=VisualizationConfig(
                 publish_contacts=False))
    ContactVisualizer.AddToBuilder(
        builder=builder, plant=plant, meshcat=meshcat,
        params=ContactVisualizerParams(
                 newtons_per_meter=20,
                 newton_meters_per_meter=0.1))


# ## Run simulation
#
# We will run the simulation. In MeshCat, the red arrow will represent the
# force `f`, and the blue arrow will represent the torque `tau`. You should see
# the contact patch moving around together with the force and torque vectors.
#
# After running the code below, playback with `timeScale` = 0.1 to appreciate
# the contact dynamics. You should see the force and torque vectors oscillate
# synchronously with the rocking bell pepper and bowl. See the section
# *Playback recording of the simulation* in
# [hydroelastic_contact_basics.ipynb](./hydroelastic_contact_basics.ipynb) for
# more details.
#
# Currently playing back the simulation will show contact force and torque
# correctly; however, it does not show contact patch appropriately, which could
# be confusing. Issue
# [19142](https://github.com/RobotLocomotion/drake/issues/19142) explains the
# problem in more details.

# +
# Clear MeshCat window from the previous blocks.
meshcat.Delete()
meshcat.DeleteAddedControls()

time_step = 1e-2
builder, plant = add_scene(time_step)
add_viz(builder, plant)

diagram = builder.Build()

simulator = Simulator(diagram)

# In interactive mode, simulate for longer time.
# In test mode, simulate for shorter time.
sim_time = 2 if not test_mode else 0.01

meshcat.StartRecording()
simulator.set_target_realtime_rate(1)
simulator.AdvanceTo(sim_time)
meshcat.StopRecording()
meshcat.PublishRecording()
# -

# ## Download simulation result into a html file for sharing
#
# You can download the simulation result into a self-contained html file,
# allowing others to playback the simulated results without simulating. The
# following code prints the URL for downloading. Click on the printed URL to
# download.

print(f"{meshcat.web_url()}/download")

# ## Further reading
#
# * [Hydroelastic Contact User Guide](https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html)  # noqa
#
# * Elandt, R., Drumwright, E., Sherman, M., & Ruina, A. (2019, November). A
# * pressure field model for fast, robust approximation of net contact force
# * and moment between nominally rigid objects. In 2019 IEEE/RSJ International
# * Conference on Intelligent Robots and Systems(IROS)
# * (pp. 8238-8245). IEEE. [link](https://arxiv.org/abs/1904.11433)
#
# * Masterjohn, J., Guoy, D., Shepherd, J., & Castro, A. (2022). Velocity Level
# * Approximation of Pressure Field Contact Patches. IEEE Robotics and
# * Automation Letters 7, no. 4 (2022):
# * 11593-11600. [link](https://arxiv.org/abs/2110.04157v2)
#
# * Elandt, R. (2022, December). Pressure Field Contact. Dissertation. Cornell
# * University. [link](https://ecommons.cornell.edu/handle/1813/112919)
