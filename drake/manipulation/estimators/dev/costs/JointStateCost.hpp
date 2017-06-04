#ifndef JOINT_STATE_COST_H
#define JOINT_STATE_COST_H

#include <stdexcept>
#include <iostream>
#include "ManipulationTrackerCost.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <mutex>
#include "yaml-cpp/yaml.h"

#include "lcmtypes/bot_core/joint_state_t.hpp"

class JointStateCost : public ManipulationTrackerCost {
public:
  JointStateCost(std::shared_ptr<const RigidBodyTreed> robot_, std::shared_ptr<lcm::LCM> lcm_, YAML::Node config);
  ~JointStateCost() {};

  bool constructCost(ManipulationTracker * tracker, const Eigen::VectorXd x_old, Eigen::MatrixXd& Q, Eigen::VectorXd& f, double& K);

  void readTreeState(const Eigen::VectorXd& q);
private:
  std::string state_channelname = "";
  double joint_reported_var = INFINITY;
  double timeout_time = 0.5;
  bool verbose = false;
  std::vector<std::string> listen_joints;

  std::shared_ptr<lcm::LCM> lcm;
  lcm::Subscription * state_sub;
  std::shared_ptr<const RigidBodyTreed> robot;
  int nq;

  Eigen::VectorXd q_robot_measured;
  std::vector<bool> q_robot_measured_known;
  std::mutex x_robot_measured_mutex;

  double lastReceivedTime;
};

#endif
