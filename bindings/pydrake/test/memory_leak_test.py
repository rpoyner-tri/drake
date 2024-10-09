"""Eventually this program might grow up to be an actual regression test for
memory leaks, but for now it merely serves to demonstrate such leaks.

Currently, it neither asserts the absence of leaks (i.e., a real test) nor the
presence of leaks (i.e., an expect-fail test) -- instead, it's a demonstration
that we can instrument and observe by hand, to gain traction on the problem.
"""

import argparse
import dataclasses
import functools
import gc
import resource
import sys
import textwrap
import weakref

from pydrake.planning import RobotDiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource

from pydrake.common import RandomGenerator
from pydrake.common.schema import Rotation, Transform
from pydrake.lcm import DrakeLcmParams
from pydrake.geometry import Meshcat
from pydrake.manipulation import ApplyDriverConfigs, IiwaDriver
from pydrake.geometry import SceneGraphConfig
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.multibody.parsing import (
    LoadModelDirectivesFromString,
    ProcessModelDirectives,
)
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    Simulator,
    SimulatorConfig,
)
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.lcm import ApplyLcmBusConfig
from pydrake.systems.sensors import ApplyCameraConfig, CameraConfig
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig


@dataclasses.dataclass
class RepetitionDetail:
    """Captures some details of an instrumented run: an iteration counter, and
    the count of allocated memory blocks."""
    i: int
    blocks: int | None = None
    maxrss: int | None = None


@dataclasses.dataclass(frozen=True)
class Sentinel:
    finalizer: weakref.finalize
    name: str


def _get_maxrss():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss


def _object_generation(o):
    for gen in range(3):
        gen_list = gc.get_objects(generation=gen)
        gen_id_list = [id(x) for x in gen_list]
        if id(o) in gen_id_list:
            return gen
    assert False  # Provided a non GC object handle.


def _make_sentinel(obj, name):
    print(f"made {name} {hex(id(obj))}")

    def done(oid):
        print(f"unmade {name} {hex(oid)}")
    return Sentinel(finalizer=weakref.finalize(obj, done, id(obj)),
                    name=name)


def _dut_simple_source():
    """A device under test that creates and destroys a leaf system."""
    source = ConstantVectorSource([1.0])
    return {_make_sentinel(source, "simple source")}


def _counts_for_cycle_parts(o, name):
    o_count = sys.getrefcount(o)
    dict_count = sys.getrefcount(o.__dict__)
    if hasattr(o, "_pydrake_ref_cycle_peers"):
        set_count = sys.getrefcount(o._pydrake_ref_cycle_peers)
    else:
        set_count = 0
    print(f"{name}: o count {o_count} dict count {dict_count}"
          f" set count {set_count}")


def _dut_trivial_simulator():
    """A device under test that creates and destroys a simulator that contains
    only a single, simple subsystem."""
    builder = DiagramBuilder()
    got = builder.AddSystem(ConstantVectorSource([1.0]))
    got2 = builder.AddSystem(ConstantVectorSource([1.0]))
    diagram = builder.Build()
    simulator = Simulator(system=diagram)
    simulator.AdvanceTo(1.0)
    _counts_for_cycle_parts(builder, "builder")
    _counts_for_cycle_parts(got, "source")
    _counts_for_cycle_parts(got2, "source2")
    _counts_for_cycle_parts(diagram, "diagram")
    _counts_for_cycle_parts(simulator, "simulator")
    return {_make_sentinel(builder, "trivial builder"),
            _make_sentinel(got, "trivial source"),
            _make_sentinel(got2, "trivial source2"),
            _make_sentinel(diagram, "trivial diagram"),
            _make_sentinel(simulator, "trivial simulator")}


def _dut_mixed_language_simulator():
    """A device under test that creates and destroys a simulator that contains
    subsystems written in both C++ and Python."""
    builder = RobotDiagramBuilder()
    builder.builder().AddSystem(ConstantVectorSource([1.0]))
    diagram = builder.Build()
    simulator = Simulator(system=diagram)
    simulator.AdvanceTo(1.0)
    context = simulator.get_context()
    plant = diagram.plant()
    plant_context = plant.GetMyContextFromRoot(context)
    plant.EvalSceneGraphInspector(plant_context)
    return {_make_sentinel(simulator, "mixed simulator")}


@functools.cache
def _get_meshcat_singleton():
    return Meshcat()


def _dut_full_example():
    """A device under test that creates and destroys a simulator that contains
    everything a full-stack simulation would ever use."""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(
        plant_config=MultibodyPlantConfig(
            time_step=0.01,
        ),
        scene_graph_config=SceneGraphConfig(),
        builder=builder,
    )
    directives = LoadModelDirectivesFromString(textwrap.dedent("""  # noqa
    directives:
    - add_model:
        name: amazon_table
        file: package://drake_models/manipulation_station/amazon_table_simplified.sdf
    - add_weld:
        parent: world
        child: amazon_table::amazon_table
    - add_model:
        name: iiwa
        file: package://drake_models/iiwa_description/urdf/iiwa14_primitive_collision.urdf
        default_joint_positions:
          iiwa_joint_1: [-0.2]
          iiwa_joint_2: [0.79]
          iiwa_joint_3: [0.32]
          iiwa_joint_4: [-1.76]
          iiwa_joint_5: [-0.36]
          iiwa_joint_6: [0.64]
          iiwa_joint_7: [-0.73]
    - add_frame:
        name: iiwa_on_world
        X_PF:
          base_frame: world
          translation: [0, -0.7, 0.1]
          rotation: !Rpy { deg: [0, 0, 90] }
    - add_weld:
        parent: iiwa_on_world
        child: iiwa::base
    - add_model:
        name: wsg
        file: package://drake_models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf
        default_joint_positions:
          left_finger_sliding_joint: [-0.02]
          right_finger_sliding_joint: [0.02]
    - add_frame:
        name: wsg_on_iiwa
        X_PF:
          base_frame: iiwa_link_7
          translation: [0, 0, 0.114]
          rotation: !Rpy { deg: [90, 0, 90] }
    - add_weld:
        parent: wsg_on_iiwa
        child: wsg::body
    - add_model:
        name: bell_pepper
        file: package://drake_models/veggies/yellow_bell_pepper_no_stem_low.sdf
        default_free_body_pose:
          flush_bottom_center__z_up:
            base_frame: amazon_table::amazon_table
            translation: [0, 0.10, 0.20]
    """))
    added_models = ProcessModelDirectives(
        plant=plant,
        directives=directives,
    )
    plant.Finalize()
    lcm_buses = ApplyLcmBusConfig(
        builder=builder,
        lcm_buses={
            "default": DrakeLcmParams(),
        },
    )
    ApplyDriverConfigs(
        builder=builder,
        sim_plant=plant,
        models_from_directives=added_models,
        lcm_buses=lcm_buses,
        driver_configs={
            "iiwa": IiwaDriver(
                hand_model_name="wsg",
            ),
        },
    )
    ApplyCameraConfig(
        builder=builder,
        lcm_buses=lcm_buses,
        config=CameraConfig(
            name="camera_0",
            X_PB=Transform(
                translation=[1.5, 0.8, 1.25],
                rotation=Rotation(value=Rotation.Rpy(deg=[-120, 5, 125])),
            ),
        ),
    )
    ApplyVisualizationConfig(
        builder=builder,
        lcm_buses=lcm_buses,
        config=VisualizationConfig(),
        meshcat=_get_meshcat_singleton(),
    )
    diagram = builder.Build()
    simulator = Simulator(system=diagram)
    ApplySimulatorConfig(
        simulator=simulator,
        config=SimulatorConfig(),
    )
    random = RandomGenerator(22)
    diagram.SetRandomContext(simulator.get_mutable_context(), random)
    simulator.AdvanceTo(0.5)
    return {_make_sentinel(simulator, "full simulator")}


def _report_sentinels(sentinels, message):
    print(message)
    for sentinel in sentinels:
        print(f"sentinel for {sentinel.name}")
        finalizer = sentinel.finalizer
        print(f"sentinel alive? {finalizer.alive}")
        if finalizer.alive:
            o = finalizer.peek()[0]
            print(f"generation: {_object_generation(o)}")
            print(f"referrers: {gc.get_referrers(o)}")
            print(f"referents: {gc.get_referents(o)}")
            print(f"is_tracked: {gc.is_tracked(o)}")


def _repeat(*, dut: callable, count: int) -> list[RepetitionDetail]:
    """Returns the details of calling dut() for count times in a row."""
    # Pre-allocate all of our return values.
    details = [RepetitionDetail(i=i) for i in range(count)]
    gc.collect()
    tare_blocks = sys.getallocatedblocks()
    tare_maxrss = _get_maxrss()
    # Call the dut repeatedly, keeping stats as we go.
    for i in range(count):
        sentinels = dut()
        _report_sentinels(sentinels, "before collect")
        gc.collect()
        _report_sentinels(sentinels, "after collect")
        details[i].blocks = sys.getallocatedblocks() - tare_blocks
        details[i].maxrss = _get_maxrss() - tare_maxrss
    return details


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--count",
        metavar="N",
        type=int,
        default=5,
        help="Number of iterations to run",
    )
    parser.add_argument(
        "--dut",
        metavar="NAME",
        help="Chooses a device under test; when not given, all DUTs are run.",
    )
    args = parser.parse_args()
    all_duts = dict([
        (dut.__name__[5:], dut)
        for dut in [
            _dut_simple_source,
            _dut_trivial_simulator,
            _dut_mixed_language_simulator,
            _dut_full_example,
        ]
    ])
    if args.dut:
        run_duts = {args.dut: all_duts[args.dut]}
    else:
        run_duts = all_duts
    for name, dut in run_duts.items():
        details = _repeat(dut=dut, count=args.count)
        print(f"RUNNING: {name}")
        for x in details:
            print(x)


assert __name__ == "__main__", __name__
sys.exit(_main())
