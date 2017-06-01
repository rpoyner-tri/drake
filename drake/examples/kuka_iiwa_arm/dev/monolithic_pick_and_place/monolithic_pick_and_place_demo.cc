#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "bot_core/robot_state_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/state_machine_system.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"
#include "drake/examples/kuka_iiwa_arm/robot_plan_interpolator.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
"Number of seconds to simulate.");
DEFINE_uint64(box_choice, 1, "ID of the box to pick.");
DEFINE_double(orientation, 2 * M_PI, "Yaw angle of the box.");
DEFINE_int32(start_position, 0, "Position index to start from");
DEFINE_int32(end_position, 0, "Position index to start from");

using robotlocomotion::robot_plan_t;

namespace drake {
using manipulation::schunk_wsg::SchunkWsgTrajectoryGenerator;
using manipulation::schunk_wsg::SchunkWsgStatusSender;
using systems::RigidBodyPlant;
using systems::Simulator;

namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
namespace {

const char kIiwaUrdf[] =
    "/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const char kIiwaEndEffectorName[] = "iiwa_link_ee";

// The `z` coordinate of the top of the table in the world frame.
// The quantity 0.736 is the `z` coordinate of the frame associated with the
// 'surface' collision element in the SDF. This element uses a box of height
// 0.057m thus giving the surface height (`z`) in world coordinates as
// 0.736 + 0.057 / 2.
const double kTableTopZInWorld = 0.736 + 0.057 / 2;

// Coordinates for kRobotBase originally from iiwa_world_demo.cc.
// The intention is to center the robot on the table.
// TODO(sam.creasey) fix this
const Eigen::Vector3d kRobotBase(0, 0, kTableTopZInWorld);
const Eigen::Vector3d kTableBase(0.243716, 0.625087, 0.);

std::unique_ptr<systems::RigidBodyPlant<double>> BuildCombinedPlant(
    const std::vector<Eigen::Vector3d>& post_positions,
    const Eigen::Vector3d& table_position,
    const Eigen::Vector3d& box_position,
    const Eigen::Vector3d& box_orientation,
    ModelInstanceInfo<double>* iiwa_instance,
    ModelInstanceInfo<double>* wsg_instance,
    ModelInstanceInfo<double>* box_instance) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", kIiwaUrdf);
  tree_builder->StoreModel("table",
                           "/examples/kuka_iiwa_arm/models/table/"
                           "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel(
      "box_small",
      "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf");

  tree_builder->StoreModel("box_medium",
                           "/examples/kuka_iiwa_arm/models/objects/"
                           "block_for_pick_and_place_mid_size.urdf");

  tree_builder->StoreModel("box_large",
                           "/examples/kuka_iiwa_arm/models/objects/"
                           "block_for_pick_and_place_large_size.urdf");
  tree_builder->StoreModel("yellow_post",
                           "/examples/kuka_iiwa_arm/models/objects/"
                           "yellow_post.urdf");
  tree_builder->StoreModel(
      "wsg",
      "/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_ball_contact.sdf");

  // The main table which the arm sits on.
  tree_builder->AddFixedModelInstance("table",
                                      kTableBase,
                                      Eigen::Vector3d::Zero());
  tree_builder->AddFixedModelInstance("table",
                                      kTableBase + table_position,
                                      Eigen::Vector3d::Zero());
  for (const Eigen::Vector3d& post_location : post_positions) {
    tree_builder->AddFixedModelInstance("yellow_post",
                                        post_location,
                                        Eigen::Vector3d::Zero());
  }
  tree_builder->AddGround();
  // Chooses an appropriate box.
  int box_id = 0;
  int iiwa_id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  *iiwa_instance = tree_builder->get_model_info_for_instance(iiwa_id);
  switch (FLAGS_box_choice) {
    case 1:
      box_id = tree_builder->AddFloatingModelInstance("box_small", box_position,
                                                      box_orientation);
      break;
    case 2:
      box_id = tree_builder->AddFloatingModelInstance(
          "box_medium", box_position, box_orientation);
      break;
    case 3:
      box_id = tree_builder->AddFloatingModelInstance("box_large", box_position,
                                                      box_orientation);
      break;
    default: DRAKE_ABORT_MSG("Chosen box should be between 0 and 4.");
      break;
  }
  *box_instance = tree_builder->get_model_info_for_instance(box_id);

  int wsg_id = tree_builder->AddModelInstanceToFrame(
      "wsg", Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);
  *wsg_instance = tree_builder->get_model_info_for_instance(wsg_id);

  return std::make_unique<systems::RigidBodyPlant<double>>(
      tree_builder->Build());
}


int DoMain(void) {

  // Locations for the posts from physical pick and place tests with
  // the iiwa+WSG.
  std::vector<Eigen::Vector3d> post_locations;
  // TODO(sam.creasey) this should be 1.10 in the Y direction.
  post_locations.push_back(Eigen::Vector3d(0.00, 1.00, 0));  // position A
  post_locations.push_back(Eigen::Vector3d(0.80, 0.36, 0));  // position B
  post_locations.push_back(Eigen::Vector3d(0.30, -0.9, 0));  // position D
  post_locations.push_back(Eigen::Vector3d(-0.1, -1.0, 0));  // position E
  post_locations.push_back(Eigen::Vector3d(-0.47, -0.8, 0));  // position F

  // Location for the extra table from the pick and place tests.
  Eigen::Vector3d table_position(0.9, -0.36, -0.07);  // position C

  Eigen::Vector3d post_height_offset(0, 0, 0.27);

  // TODO(sam.creasey) select only one of these
  std::vector<Isometry3<double>> place_locations;
  Isometry3<double> place_location;
#if 0
  place_location.translation() = post_locations[0] + post_height_offset;
  place_location.linear() = Matrix3<double>(
      AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
  place_locations.push_back(place_location);
#endif

  place_location.translation() = post_locations[1] + post_height_offset;
  place_location.linear().setIdentity();
  place_locations.push_back(place_location);

  place_location.translation() = table_position;
  place_location.linear().setIdentity();
  place_locations.push_back(place_location);

  place_location.translation() = post_locations[2] + post_height_offset;
  place_location.linear() = Matrix3<double>(
      AngleAxis<double>(-M_PI / 2., Vector3<double>::UnitZ()));
  place_locations.push_back(place_location);

  place_location.translation() = post_locations[3] + post_height_offset;
  place_location.linear() = Matrix3<double>(
      AngleAxis<double>(-M_PI / 2., Vector3<double>::UnitZ()));
  place_locations.push_back(place_location);

  place_location.translation() = post_locations[4] + post_height_offset;
  place_location.linear() = Matrix3<double>(
      AngleAxis<double>(-M_PI / 2., Vector3<double>::UnitZ()));
  place_locations.push_back(place_location);

  drake::log()->info("1: Place location 0 {}",
                     place_locations[0].translation().transpose());

  Eigen::Vector3d box_origin(0, 0, kTableTopZInWorld + 0.1);
  box_origin += place_locations[FLAGS_start_position].translation();

  drake::log()->info("Box origin {}", box_origin.transpose());
  drake::log()->info("2: Place location 0 {}",
                     place_locations[0].translation().transpose());

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  ModelInstanceInfo<double> iiwa_instance, wsg_instance, box_instance;

  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant(post_locations, table_position,
                         box_origin, Vector3<double>(0, 0, FLAGS_orientation),
                         &iiwa_instance, &wsg_instance, &box_instance);


  auto plant = builder.AddSystem<IiwaAndWsgPlantWithStateEstimator<double>>(
      std::move(model_ptr), iiwa_instance, wsg_instance, box_instance);
  plant->set_name("plant");

  auto drake_visualizer = builder.AddSystem<systems::DrakeVisualizer>(
      plant->get_plant().get_rigid_body_tree(), &lcm);

  builder.Connect(plant->get_output_port_plant_state(),
                  drake_visualizer->get_input_port(0));

  auto iiwa_trajectory_generator = builder.AddSystem<RobotPlanInterpolator>(
      drake::GetDrakePath() + kIiwaUrdf);
  builder.Connect(plant->get_output_port_iiwa_state(),
                  iiwa_trajectory_generator->get_state_input_port());
  builder.Connect(
      iiwa_trajectory_generator->get_state_output_port(),
      plant->get_input_port_iiwa_state_command());
  builder.Connect(
      iiwa_trajectory_generator->get_acceleration_output_port(),
      plant->get_input_port_iiwa_acceleration_command());

  auto wsg_trajectory_generator =
      builder.AddSystem<SchunkWsgTrajectoryGenerator>(
          plant->get_output_port_wsg_state().size(), 0);
  builder.Connect(plant->get_output_port_wsg_state(),
                  wsg_trajectory_generator->get_state_input_port());
  builder.Connect(wsg_trajectory_generator->get_output_port(0),
                  plant->get_input_port_wsg_command());

  auto wsg_status_sender = builder.AddSystem<SchunkWsgStatusSender>(
      plant->get_output_port_wsg_state().size(), 0, 0);
  builder.Connect(plant->get_output_port_wsg_state(),
                  wsg_status_sender->get_input_port(0));

  const Eigen::Vector3d robot_base(0, 0, kTableTopZInWorld);
  Isometry3<double> iiwa_base = Isometry3<double>::Identity();
  iiwa_base.translation() = robot_base;

  drake::log()->info("3: Place location 0 {}",
                     place_locations[0].translation().transpose());

  auto state_machine =
      builder.template AddSystem<PickAndPlaceStateMachineSystem>(
          drake::GetDrakePath() + kIiwaUrdf, kIiwaEndEffectorName,
          iiwa_base, place_locations);

  builder.Connect(plant->get_output_port_box_robot_state_msg(),
                  state_machine->get_input_port_box_state());
  builder.Connect(wsg_status_sender->get_output_port(0),
                  state_machine->get_input_port_wsg_status());
  builder.Connect(plant->get_output_port_iiwa_robot_state_msg(),
                  state_machine->get_input_port_iiwa_state());
  builder.Connect(state_machine->get_output_port_wsg_command(),
                  wsg_trajectory_generator->get_command_input_port());
  builder.Connect(state_machine->get_output_port_iiwa_plan(),
                  iiwa_trajectory_generator->get_plan_input_port());

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);
  simulator.Initialize();

  auto plan_source_context = sys->GetMutableSubsystemContext(
      simulator.get_mutable_context(), iiwa_trajectory_generator);
  iiwa_trajectory_generator->Initialize(
      plan_source_context->get_time(),
      Eigen::VectorXd::Zero(7),
      plan_source_context->get_mutable_state());
  simulator.StepTo(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::monolithic_pick_and_place::DoMain();
}
