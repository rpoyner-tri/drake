#pragma once

#include <string>

#include "drake/common/name_value.h"

namespace drake {
namespace systems {

// N.B. The names and defaults here match drake/systems/analysis exactly.
/// The set of configurable properties on a simulator.
struct SimulatorConfig {
  template <typename Archive>
  void Serialize(Archive* a) {
  }

  std::string integration_scheme{"runge_kutta3"};
  double max_step_size{1.0e-3};
  double accuracy{1.0e-2};
  bool use_error_control{true};
  double target_realtime_rate{0.0};
  double publish_every_time_step{false};
};

}  // namespace systems
}  // namespace drake
