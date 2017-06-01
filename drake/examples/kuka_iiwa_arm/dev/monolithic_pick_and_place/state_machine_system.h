#pragma once

#include <memory>
#include <string>

#include "bot_core/robot_state_t.hpp"

#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/sparsity_matrix.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

/**
 * A class that implements the Finite-State-Machine logic for the
 * Pick-And-Place demo. This system should be used by coupling the outputs with
 * the `IiwaMove` and `GripperAction` systems and the inputs are to be
 * connected to the appropriate output ports of the `IiwaStatusSender`,
 * `SchunkWsgStatusSender` and `OracularStateEstimator` systems.
 */
class PickAndPlaceStateMachineSystem : public systems::LeafSystem<double> {
 public:
  /**
   * Constructor for the PickAndPlaceStateMachineSystem
   * @param iiwa_base, The pose of the base of the IIWA robot system.
   * @param update_interval : The update interval of the unrestricted update of
   * this system. This should be bigger than that of the PlanSource components.
   */
  PickAndPlaceStateMachineSystem(
      const std::string& iiwa_model_path,
      const std::string& end_effector_name,
      const Isometry3<double>& iiwa_base,
      const std::vector<Isometry3<double>>& place_locations,
      const double update_interval = 0.01);

  std::unique_ptr<systems::AbstractValues> AllocateAbstractState()
      const override;

  // This kind of a system is not a direct feedthrough.
  bool DoHasDirectFeedthrough(const systems::SparsityMatrix*,
                              int, int) const final {
    return false;
  }

  void SetDefaultState(const systems::Context<double>& context,
                       systems::State<double>* state) const override;

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  /**
   * Getter for the input port corresponding to the abstract input with iiwa
   * state message (LCM `robot_state_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_iiwa_state()
      const {
    return this->get_input_port(input_port_iiwa_state_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with box
   * state message (LCM `botcore::robot_state_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_box_state() const {
    return this->get_input_port(input_port_box_state_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with the wsg
   * status message (LCM `lcmt_schunk_wsg_status` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_wsg_status()
      const {
    return this->get_input_port(input_port_wsg_status_);
  }

  const systems::OutputPortDescriptor<double>& get_output_port_iiwa_plan()
      const {
    return this->get_output_port(output_port_iiwa_plan_);
  }

  const systems::OutputPortDescriptor<double>& get_output_port_wsg_command()
      const {
    return this->get_output_port(output_port_wsg_command_);
  }

 private:
  struct InternalState;

  RigidBodyTree<double> iiwa_tree_{};
  // Input ports.
  int input_port_iiwa_state_{-1};
  int input_port_box_state_{-1};
  int input_port_wsg_status_{-1};
  // Output ports.
  int output_port_iiwa_plan_{-1};
  int output_port_wsg_command_{-1};

  std::string iiwa_model_path_;
  std::string end_effector_name_;
  const Isometry3<double> iiwa_base_;

  const std::unique_ptr<
    manipulation::planner::ConstraintRelaxingIk> planner_{nullptr};

  std::vector<Isometry3<double>> place_locations_;
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
