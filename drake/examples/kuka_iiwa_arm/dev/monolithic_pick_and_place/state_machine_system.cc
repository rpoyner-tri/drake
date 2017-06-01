#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/state_machine_system.h"

#include <utility>
#include <vector>

#include "bot_core/robot_state_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/pick_and_place_state_machine.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/world_state.h"

using bot_core::robot_state_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
/* index of iiwastate */
const int kStateIndex = 0;

robotlocomotion::robot_plan_t MakeDefaultIiwaPlan() {
  robotlocomotion::robot_plan_t default_plan{};
  default_plan.utime = 0;
  default_plan.num_states = 0;
  return default_plan;
}

lcmt_schunk_wsg_command MakeDefaultWsgCommand() {
  lcmt_schunk_wsg_command default_command{};
  default_command.utime = 0;
  default_command.target_position_mm = 110;  // maximum aperture
  default_command.force = 0;
  return default_command;
}
}  // namespace

using manipulation::planner::ConstraintRelaxingIk;
using pick_and_place::PickAndPlaceStateMachine;

namespace monolithic_pick_and_place {

struct PickAndPlaceStateMachineSystem::InternalState {
  InternalState(const std::string& iiwa_model_path,
                const std::string& end_effector_name,
                const std::vector<Isometry3<double>>& place_locations)
      : world_state(iiwa_model_path, end_effector_name),
        state_machine(place_locations, false),
        last_iiwa_plan(MakeDefaultIiwaPlan()),
        last_wsg_command(MakeDefaultWsgCommand()) {}

  ~InternalState() {}

  pick_and_place::WorldState world_state;
  PickAndPlaceStateMachine state_machine;
  robotlocomotion::robot_plan_t last_iiwa_plan;
  lcmt_schunk_wsg_command last_wsg_command;
};

PickAndPlaceStateMachineSystem::PickAndPlaceStateMachineSystem(
    const std::string& iiwa_model_path,
    const std::string& end_effector_name,
    const Isometry3<double>& iiwa_base,
    const std::vector<Isometry3<double>>& place_locations,
    const double update_interval)
    : iiwa_model_path_(iiwa_model_path),
      end_effector_name_(end_effector_name),
      iiwa_base_(iiwa_base),
      planner_(std::make_unique<ConstraintRelaxingIk>(
          iiwa_model_path_, end_effector_name_, iiwa_base_)),
      place_locations_(place_locations) {
  input_port_iiwa_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_box_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_wsg_status_ = this->DeclareAbstractInputPort().get_index();

  output_port_iiwa_plan_ = this->DeclareAbstractOutputPort(
      systems::Value<robotlocomotion::robot_plan_t>(
          MakeDefaultIiwaPlan())).get_index();

  output_port_wsg_command_ = this->DeclareAbstractOutputPort(
      systems::Value<lcmt_schunk_wsg_command>(
          MakeDefaultWsgCommand())).get_index();
  this->DeclarePeriodicUnrestrictedUpdate(update_interval, 0);

  const double half_box_height = 0.1;
  for (size_t i = 0; i < place_locations_.size(); i++) {
    place_locations_[i].translation() +=
        Vector3<double>(0, 0, half_box_height);
  }

  // TODO(sam.creasey) Add verification of final object position and
  // termination.
}

std::unique_ptr<systems::AbstractValues>
PickAndPlaceStateMachineSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<InternalState>(
          InternalState(iiwa_model_path_, end_effector_name_,
                        place_locations_))));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

void PickAndPlaceStateMachineSystem::SetDefaultState(
    const systems::Context<double>&,
    systems::State<double>* state) const {
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);
  internal_state = InternalState(iiwa_model_path_, end_effector_name_,
                                 place_locations_);
}

void PickAndPlaceStateMachineSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  /* Call actions based on state machine logic */

  robotlocomotion::robot_plan_t& iiwa_plan =
      output->GetMutableData(output_port_iiwa_plan_)
      ->GetMutableValue<robotlocomotion::robot_plan_t>();

  lcmt_schunk_wsg_command& wsg_command =
      output->GetMutableData(output_port_wsg_command_)
      ->GetMutableValue<lcmt_schunk_wsg_command>();

  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);

  iiwa_plan = internal_state.last_iiwa_plan;
  wsg_command = internal_state.last_wsg_command;
}

void PickAndPlaceStateMachineSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Extract Internal state.
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);

  /* Update world state from inputs. */
  const robot_state_t& iiwa_state =
      this->EvalAbstractInput(context, input_port_iiwa_state_)
          ->GetValue<robot_state_t>();
  const robot_state_t& box_state =
      this->EvalAbstractInput(context, input_port_box_state_)
          ->GetValue<robot_state_t>();
  const lcmt_schunk_wsg_status& wsg_status =
      this->EvalAbstractInput(context, input_port_wsg_status_)
          ->GetValue<lcmt_schunk_wsg_status>();

  internal_state.world_state.HandleIiwaStatus(iiwa_state);
  internal_state.world_state.HandleWsgStatus(wsg_status);
  internal_state.world_state.HandleObjectStatus(box_state);

  PickAndPlaceStateMachine::IiwaPublishCallback iiwa_callback =
      ([&](const robotlocomotion::robot_plan_t* plan) {
        internal_state.last_iiwa_plan = *plan;
      });

  PickAndPlaceStateMachine::WsgPublishCallback wsg_callback =
      ([&](const lcmt_schunk_wsg_command* msg) {
        internal_state.last_wsg_command = *msg;
      });
  internal_state.state_machine.Update(
      internal_state.world_state, iiwa_callback, wsg_callback, planner_.get());
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
