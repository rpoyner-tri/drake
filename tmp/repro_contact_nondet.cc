#include <map>
#include <string>
#include <tuple>

#include <sys/personality.h>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_flags.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/tmp/float_fmt.h"
#include "drake/tmp/frames.h"
#include "drake/tmp/text.h"

namespace drake {
namespace tmp {

DEFINE_string(log_level, "err", "Drake log level.");
DEFINE_double(end_time, 1.0, "Length of simulation (seconds).");
DEFINE_int64(meta_trials, 2, "Number of meta trials.");
DEFINE_int64(sim_trials, 4, "Number of sim trials within a meta trial.");
DEFINE_string(styles, "", "Styles to run, comma-separated list.");
DEFINE_string(setups, "", "Setups to run, comma-separated list.");
DEFINE_bool(visualize, false, "Emit data to drake visualizer.");

using Simulator = systems::Simulator<double>;
using CalcOutput = std::function<std::string (const systems::Context<double>&)>;

template <typename T, typename U>
U find_dammit(const std::map<T, U>& map, const T& key, const std::string& message) {
  auto it = map.find(key);
  if (it != map.end()) {
    return it->second;
  }
  throw std::runtime_error(message);
}

template <typename T> std::string to_string(const T&);

// Enum ResimulateStyle.
enum class ResimulateStyle {
  Reuse = 0, ReuseNewContext = 1, Recreate = 2,
};
template <> std::string to_string(const ResimulateStyle& value) {
  static const std::map<ResimulateStyle, std::string> map = {
    {ResimulateStyle::Reuse, "Reuse"},
    {ResimulateStyle::ReuseNewContext, "ReuseNewContext"},
    {ResimulateStyle::Recreate, "Recreate"},
  };
  return find_dammit(map, value, "unknown ResimulateStyle");
}

// Enum Setup.
enum class SetupEnum {
  Continuous_NoGeometry,
  Discrete_NoGeometry,
  Continuous_WithGeometry_NoGripper,
  Discrete_WithGeometry_NoGripper,
  Discrete_WithGeometry_AnzuWsg,
  Discrete_WithGeometry_AnzuWsgWelded,
  Discrete_WithGeometry_DrakeWsg,
  Discrete_WithGeometry_DrakeWsgWelded,
};
template <> std::string to_string(const SetupEnum& value) {
  static const std::map<SetupEnum, std::string> map = {
    {SetupEnum::Continuous_NoGeometry, "Continuous_NoGeometry"},
    {SetupEnum::Discrete_NoGeometry, "Discrete_NoGeometry"},
    {SetupEnum::Continuous_WithGeometry_NoGripper, "Continuous_WithGeometry_NoGripper"},
    {SetupEnum::Discrete_WithGeometry_NoGripper, "Discrete_WithGeometry_NoGripper"},
    {SetupEnum::Discrete_WithGeometry_AnzuWsg, "Discrete_WithGeometry_AnzuWsg"},
    {SetupEnum::Discrete_WithGeometry_AnzuWsgWelded, "Discrete_WithGeometry_AnzuWsgWelded"},
    {SetupEnum::Discrete_WithGeometry_DrakeWsg, "Discrete_WithGeometry_DrakeWsg"},
    {SetupEnum::Discrete_WithGeometry_DrakeWsgWelded, "Discrete_WithGeometry_DrakeWsgWelded"},
  };
  return find_dammit(map, value, "unknown SetupEnum");
}

struct Setup {
  SetupEnum tag;
  double plant_time_step{};
  bool has_geometry{};
  std::string gripper;
};
using SetupMap = std::map<SetupEnum, Setup>;

const SetupMap& setup_map() {
  static const SetupMap map = {
    {SetupEnum::Continuous_NoGeometry, {SetupEnum::Continuous_NoGeometry, 0., false, ""}},
    {SetupEnum::Discrete_NoGeometry, {SetupEnum::Discrete_NoGeometry, 0.001, false, ""}},
    {SetupEnum::Continuous_WithGeometry_NoGripper, {SetupEnum::Continuous_WithGeometry_NoGripper, 0., true, ""}},
    {SetupEnum::Discrete_WithGeometry_NoGripper, {SetupEnum::Discrete_WithGeometry_NoGripper, 0.001, true, ""}},
    // Grippers.
    {SetupEnum::Discrete_WithGeometry_AnzuWsg,
     {SetupEnum::Discrete_WithGeometry_AnzuWsg, 0., true, "drake/tmp/schunk_wsg_50_anzu.sdf"}},
    {SetupEnum::Discrete_WithGeometry_AnzuWsgWelded,
     {SetupEnum::Discrete_WithGeometry_AnzuWsgWelded, 0.001, true, "drake/tmp/schunk_wsg_50_anzu_welded.sdf"}},
    {SetupEnum::Discrete_WithGeometry_DrakeWsg,
     {SetupEnum::Discrete_WithGeometry_DrakeWsg, 0.001, true,
      "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf"}},
    {SetupEnum::Discrete_WithGeometry_DrakeWsgWelded,
     {SetupEnum::Discrete_WithGeometry_DrakeWsgWelded, 0.001, true, "drake/tmp/schunk_wsg_50_drake_welded.sdf"}},
  };
  return map;
}

math::RollPitchYaw<double> rpy_deg(const Eigen::Vector3d& rpy_rad) {
  return math::RollPitchYaw<double>{rpy_rad * M_PI / 180};
}

void no_control(systems::DiagramBuilder<double>* builder,
                const multibody::MultibodyPlant<double>& plant,
                multibody::ModelInstanceIndex model) {
  auto nu = plant.num_actuated_dofs(model);
  auto constant = builder->AddSystem<systems::ConstantVectorSource>(Eigen::VectorXd::Zero(nu));
  builder->Connect(
      constant->get_output_port(),
      plant.get_actuation_input_port(model));
}

static constexpr char contact_results_format[] = R"XXX(
point_pair_contact_info({}):
  bodyA: {}
  bodyB: {}
  contact_force: {}
  contact_point: {}
  slip_speed: {}
  separation_speed: {}
)XXX";

std::string contact_results_to_str(
    const multibody::MultibodyPlant<double>& plant,
    const multibody::ContactResults<double>& contact_results) {
  std::string out;
  auto count = contact_results.num_point_pair_contacts();
  if (count == 0) {
    return "<no contacts>";
  }
  std::vector<const multibody::PointPairContactInfo<double>*> infos(count);
  for (int k = 0; k < count; k++) {
    infos.at(k) = &contact_results.point_pair_contact_info(k);
  }
  // TODO(eric.cousineau): Are these contact pairs being out of order an issue?
  auto cmp = [](const auto* a, const auto* b) {
    // key=lambda x: (x.bodyA_index(), x.bodyB_index())
    auto key = [](const auto* x) { return std::make_pair(x->bodyA_index(), x->bodyB_index()); };
    return key(a) < key(b);
  };
  std::sort(infos.begin(), infos.end(), cmp);
  for (int i = 0; i < count; i++) {
    auto& info = infos[i];
    auto& bodyA = plant.get_body(multibody::BodyIndex(info->bodyA_index()));
    auto& bodyB = plant.get_body(multibody::BodyIndex(info->bodyB_index()));
    // N.B. Use np.array() always to ensure we use the same formatting for
    // floating-point numbers.
    out += fmt::format(contact_results_format, i, bodyA.name(), bodyB.name(),
                       vec_fmt(info->contact_force()),
                       vec_fmt(info->contact_point()),
                       float_fmt(info->slip_speed()),
                       float_fmt(info->separation_speed()));
  }
  return out;
}

std::tuple<std::unique_ptr<Simulator>, CalcOutput> make_simulator(const Setup& setup) {
  double max_step_size = 0.01;

  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double>* plant{};
  geometry::SceneGraph<double>* scene_graph{};
  if (setup.has_geometry) {
    std::tie(plant, scene_graph) = multibody::AddMultibodyPlantSceneGraph(
        &builder, setup.plant_time_step);
  } else {
    plant = builder.AddSystem<multibody::MultibodyPlant>(setup.plant_time_step);
  }
  multibody::Parser parser(plant, scene_graph);

  std::deque<multibody::ModelInstanceIndex> control_models;
  auto iiwa = parser.AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "iiwa14_spheres_dense_elbow_collision.urdf"));
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"),
                    math::RigidTransform(Eigen::Vector3d{0.6, 0., 0.}));
  control_models.push_back(iiwa);

  std::optional<multibody::ModelInstanceIndex> wsg;
  if (!setup.gripper.empty()) {
    auto [X_7G, body] = std::invoke([&] {
        struct Where { math::RigidTransform<double> wrt; std::string what; };
        if (setup.gripper.find("anzu") != std::string::npos) {
          return Where{{rpy_deg({180., 0., 158.}), Eigen::Vector3d{0, 0, 0.053}}, "gripper"};
        }  else {
          return Where{{rpy_deg({90., 0., 90.}), Eigen::Vector3d{0, 0, 0.114}}, "body"};
        }
      });
    wsg = parser.AddModelFromFile(FindResourceOrThrow(setup.gripper));
    plant->WeldFrames(
        plant->GetFrameByName("iiwa_link_7"),
        plant->GetFrameByName(body),
        X_7G);
    control_models.push_back(*wsg);
  }

  parser.AddModelFromFile(FindResourceOrThrow("drake/tmp/fake_table.sdf"));
  plant->WeldFrames(
      plant->world_frame(), plant->GetFrameByName("fake_table"),
      math::RigidTransform(Eigen::Vector3d{0, 0, -0.5}));
  plant->Finalize();

  no_control(&builder, *plant, iiwa);
  if (wsg) {
    no_control(&builder, *plant, *wsg);
  }

  if (FLAGS_visualize and scene_graph) {
    auto role = geometry::Role::kProximity;
    ConnectDrakeVisualizer(&builder, *scene_graph, nullptr, role);
    ConnectContactResultsToDrakeVisualizer(&builder, *plant);
  }

  auto diagram = builder.Build();

  auto d_context = diagram->CreateDefaultContext();
  auto& context = plant->GetMyMutableContextFromRoot(d_context.get());
  Eigen::Matrix<double, 7, 1> q0_iiwa;
  q0_iiwa << 0.02, 0.085, -0.285, 1.43, 0.284, -1.07, 0.1;
  plant->SetPositions(&context, iiwa, q0_iiwa);
  if (wsg && plant->num_positions(*wsg) > 0) {
    Eigen::Vector2d q0_wsg{.05, 0.05};
    plant->SetPositions(&context, *wsg, q0_wsg);
  }
  auto& contact_results_port = plant->get_contact_results_output_port();

  auto calc_output = [setup, plant, &contact_results_port](const systems::Context<double>& def_context) {
    auto& my_context = plant->GetMyContextFromRoot(def_context);
    auto q = plant->GetPositions(my_context);
    if (setup.tag == SetupEnum::Discrete_NoGeometry) {
      return fmt::format("q: {}", vec_fmt(q));
    } else {
      const auto& contact_results = contact_results_port.Eval<multibody::ContactResults<double>>(my_context);
      auto contact_results_text = contact_results_to_str(*plant, contact_results);
      return fmt::format("q: {}\ncontact_results: {}", vec_fmt(q), indent(contact_results_text, "  "));
    }
  };

  // sanity check.
  plant->GetMyContextFromRoot(*d_context);

  auto simulator = std::make_unique<systems::Simulator<double>>(std::move(diagram), std::move(d_context));
  if (FLAGS_visualize and scene_graph) {
    simulator->set_target_realtime_rate(1.);
  }
  systems::ResetIntegratorFromFlags<double>(simulator.get(), "runge_kutta2", max_step_size);

  // sanity check.
  plant->GetMyContextFromRoot(simulator->get_context());
  return {std::move(simulator), calc_output};
}


class SimulationChecker {
 public:
  void run(Simulator* simulator, CalcOutput calc_output) {
    double dt = 0.001;
    double end_time = FLAGS_end_time;
    const std::string prefix("    ");
    auto& d_context = simulator->get_context();

    Frames frames;
    {
      Frames::Current current(&frames);

      auto output = calc_output(d_context);
      frames.add(d_context.get_time(), output);
      while (d_context.get_time() + dt / 2 < end_time) {
        simulator->AdvanceTo(d_context.get_time() + dt);
        output = calc_output(d_context);
        frames.add(d_context.get_time(), output);
      }
    }

    int prev_count = static_cast<int>(frames_set_.size());
    frames_set_.insert(frames);
    int count = static_cast<int>(frames_set_.size());
    if (!frames_baseline_) {
      fmt::print("{}{}\n", prefix, "good (first)");
      DRAKE_ASSERT(count == 1);
      frames_baseline_ = frames;
    } else if (count == prev_count) {
      fmt::print("{}{}\n", prefix, "good (not unique)");
    } else {
      fmt::print("{}{}\n", prefix, "BAD");
      fmt::print(
          indent(frames_baseline_->text_diff(frames), prefix + "  "));
      frames_set_.insert(frames);
    }
  }

  std::string summary() const {
    int count = static_cast<int>(frames_set_.size());
    DRAKE_ASSERT(count > 0);
    if (count == 1) {
      return "good (num_unique = 1)";
    } else {
      return fmt::format("BAD  (num_unique = {})", count);
    }
  };

 private:
  std::optional<Frames> frames_baseline_;
  std::unordered_set<Frames> frames_set_;
};


std::tuple<ResimulateStyle, std::string> simulate_trials(
    ResimulateStyle style,
    int num_sim_trials,
    const Setup& setup) {
  fmt::print("{}\n", to_string(style));

  SimulationChecker checker;

  switch (style) {
    case ResimulateStyle::Reuse: {
      auto [simulator, calc_output] = make_simulator(setup);
      auto& d_context = simulator->get_mutable_context();
      auto d_context_initial = d_context.Clone();
      for (int k = 0; k < num_sim_trials; k++) {
        fmt::print("  index: {}\n", k);
        d_context.SetTimeStateAndParametersFrom(*d_context_initial);
        checker.run(simulator.get(), calc_output);
      }
      break;
    }
    case ResimulateStyle::ReuseNewContext: {
      auto [simulator, calc_output] = make_simulator(setup);
      for (int k = 0; k < num_sim_trials; k++) {
        fmt::print("  index: {}\n", k);
        simulator->reset_context(simulator->get_system().CreateDefaultContext());
        checker.run(simulator.get(), calc_output);
      }
      break;
    }
    case ResimulateStyle::Recreate: {
      for (int k = 0; k < num_sim_trials; k++) {
        fmt::print("  index: {}\n", k);
        auto [simulator, calc_output] = make_simulator(setup);
        checker.run(simulator.get(), calc_output);
      }
      break;
    }
    default:
      throw std::runtime_error("style not implemented!");
  };

  return {style, checker.summary()};
}


std::string run_simulations(int num_sim_trials, const Setup& setup) {
  const auto known_styles = {
    ResimulateStyle::Reuse,
    ResimulateStyle::ReuseNewContext,
    ResimulateStyle::Recreate,
  };
  auto styles_vec = split(FLAGS_styles, ',');
  std::set<std::string> styles(styles_vec.begin(), styles_vec.end());
  // TODO: column alignment calculated from style string sizes
  std::string result;
  for (const auto& style : known_styles) {
    const auto style_name = to_string(style);
    if (!styles.empty() && !styles.count(style_name)) {
      continue;
    }
    auto [got_style, summary] = simulate_trials(style, num_sim_trials, setup);
    DRAKE_ASSERT(got_style == style);
    result += fmt::format("{:<20}: {}\n", style_name, summary);
  }
  return result;
}



int do_main() {
  personality(ADDR_NO_RANDOMIZE);
  logging::set_log_level(FLAGS_log_level);
  DRAKE_ASSERT(1 <= FLAGS_float_precision && FLAGS_float_precision <= 17);
  int num_meta_trials = FLAGS_meta_trials;
  int num_sim_trials = FLAGS_sim_trials;
  auto setups_vec = split(FLAGS_setups, ',');
  std::set<std::string> setups(setups_vec.begin(), setups_vec.end());
  std::stringstream tally;
  tally << "\n";
  for (const auto& setup_pair : setup_map()) {
    const auto setup_name = to_string(setup_pair.first);
    if (!setups.empty() && !setups.count(setup_name)) {
      continue;
    }
    tally << fmt::format("* setup = {}\n", setup_name);
    const auto setup = setup_pair.second;
    for (int meta_trial = 0; meta_trial < num_meta_trials; meta_trial++) {
      fmt::print(
          "\n\n[ meta_trial = {}, "
          "num_sim_trials = {}, setup = {} ]\n\n",
          meta_trial, num_sim_trials, setup_name);
      auto summary = run_simulations(num_sim_trials, setup);
      tally << fmt::format("  * meta_trial = {}, num_sim_trials = {}\n",
                           meta_trial, num_sim_trials);
      tally << indent(summary, "      ");
    }
    tally << "\n";
  }
  fmt::print(tally.str());
  return 0;
}

}
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::tmp::do_main();
}
