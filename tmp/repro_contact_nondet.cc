#include <deque>
#include <map>
#include <string>
#include <tuple>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "third-party/difflib/src/difflib.h"

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_flags.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace tmp {
// Replacement for python textwrap.indent, sorta..
std::string indent(const std::string& text, const std::string& prefix) {
  std::stringstream ss(text);
  std::string to;
  std::string result;

  while (std::getline(ss, to,'\n')) {
    result += prefix + to;
  }
  return result;
}

std::string text_diff(const std::string& a, const std::string& b) {
  auto to_lines = [](const std::string& text) {
    std::vector<std::string> result;
    std::stringstream ss(text);
    while (std::getline(ss, to,'\n')) {
      result.push_back(to);
    }
    return result;
  };
  auto a_lines = to_lines(a);
  auto b_lines = to_lines(b);
  if (a_lines.size() == b_lines.size()) {
    std::string out;
    for (int k = 0; k < a_lines.size(); k++) {
      out += diff.compare(a_lines[k], b_lines[k]);
    }
  } else {
  }
  return {};
}

template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}

template<typename It> void hash_range(std::size_t& seed, It first, It last) {
  for(; first != last; ++first)
  {
    hash_combine(seed, *first);
  }
}

class Frames {
 public:
  int add(double t, const std::string& value) {
    auto index = size();
    times_.push_back(t);
    values_.push_back(value);
    return index;
  }

  int size() const { return static_cast<int>(times_.size()); }

  std::tuple<double, std::string> get(int i) const {
    if (i < 0) {
      i += size();  // Pythonic indexing.
    }
    DRAKE_ASSERT(i >= 0 && i < size());
    return {times_[i], values_[i]};
  }

  std::optional<int> first_diff(const Frames& other) const {
    // Return index for both items at first difference, or None.
    DRAKE_ASSERT(size() == other.size());
    for (int i = 0; i < size(); i++) {
      auto [t, value] = get(i);
      auto [t_other, value_other] = other.get(i);
      DRAKE_ASSERT(t == t_other);
      if (value != value_other) {
        return i;
      }
    }
    return {};
  }

  bool operator==(const Frames& other) const { return !first_diff(other); }

  size_t hash() const {
    size_t seed = 0;
    hash_range(seed, times_.begin(), times_.end());
    hash_range(seed, values_.begin(), values_.end());
    return seed;
  }

  std::string text_diff(const Frames& other) const {
    auto index = first_diff(other);
    DRAKE_ASSERT(!!index);
    std::stringstream ss;
    auto fmt_diff = [&](int index_actual, int index_label) {
      auto [t, value] = get(index_actual);
      auto [_, value_other] = other.get(index_actual);
      _ = {}; // ignore.
      ss << fmt::format("diff[{}], t: {}\n", index_label, t)
         << indent(tmp::text_diff(value, value_other), "  ") << "\n";
    };
    fmt_diff(*index, 0);
    fmt_diff(-1, -1);
    return ss.str();
  }

 private:
  std::deque<double> times_;
  std::deque<std::string> values_;
};
}
}

namespace std {
template<>
struct hash<drake::tmp::Frames> {
  size_t operator()(const drake::tmp::Frames& x) const {
    return x.hash();
  }
};
}

namespace drake {
namespace tmp {
using Simulator = systems::Simulator<double>;
using CalcOutput = std::function<std::string (const systems::Context<double>&)>;

static constexpr bool VISUALIZE = false;

template <typename T, typename U>
U find_dammit(const std::map<T, U>& map, const T& key, const std::string& message) {
  auto it = map.find(key);
  if (it != map.end()) {
    return it->second;
  }
  throw std::runtime_error(message);
}


// Enum ResimulateStyle.
enum class ResimulateStyle {
  kReuse = 0, kReuseNewContext = 1, kRecreate = 2,
};
using ResimulateStyleToString = std::map<ResimulateStyle, std::string>;

template <typename T> const std::string to_string(const T&);
template <> const std::string to_string(const ResimulateStyle& value) {
  static const ResimulateStyleToString map = {
    {ResimulateStyle::kReuse, "kReuse"},
    {ResimulateStyle::kReuseNewContext, "kReuseNewContext"},
    {ResimulateStyle::kRecreate, "kRecreate"},
  };
  return find_dammit(map, value, "unknown ResimulateStyle");
}

// Enum Setup.
struct Setup {
  std::string name;
  double plant_time_step{};
  bool has_geometry{};
  std::string gripper;
};
using SetupMap = std::map<std::string, Setup>;

const SetupMap& setup_map() {
  static const SetupMap map = {
    {"Continuous_NoGeometry", {"Continuous_NoGeometry", 0., false, ""}},
    {"Discrete_NoGeometry", {"Discrete_NoGeometry", 0.001, false, ""}},
    {"Continuous_WithGeometry_NoGripper", {"Continuous_WithGeometry_NoGripper", 0., true, ""}},
    {"Discrete_WithGeometry_NoGripper", {"Discrete_WithGeometry_NoGripper", 0.001, true, ""}},
    // Grippers.
    {"Discrete_WithGeometry_AnzuWsg",
     {"Discrete_WithGeometry_AnzuWsg", 0., true, "drake/tmp/schunk_wsg_50_anzu.sdf"}},
    {"Discrete_WithGeometry_AnzuWsgWelded",
     {"Discrete_WithGeometry_AnzuWsgWelded", 0.001, true, "drake/tmp/schunk_wsg_50_anzu_welded.sdf"}},
    {"Discrete_WithGeometry_DrakeWsg",
     {"Discrete_WithGeometry_DrakeWsg", 0.001, true,
      "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf"}},
    {"Discrete_WithGeometry_DrakeWsgWelded",
     {"Discrete_WithGeometry_DrakeWsgWelded", 0.001, true, "drake/tmp/schunk_wsg_50_drake_welded.sdf"}},
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
    out += fmt::format(contact_results_format, i, bodyA.name(), bodyB.name(), info->contact_force(),
                       info->contact_point(), info->slip_speed(), info->separation_speed());
  }
  return out;  // .rstrip();
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

  if (VISUALIZE and scene_graph) {
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
    if (setup.
        name == "Discrete_NoGeometry") {
      return fmt::format("q: {}", q);
    } else {
      const auto& contact_results = contact_results_port.Eval<multibody::ContactResults<double>>(my_context);
      auto contact_results_text = contact_results_to_str(*plant, contact_results);
      return fmt::format("q: {}\ncontact_results: {}", q, indent(contact_results_text, "  "));
    }
  };

  // sanity check.
  plant->GetMyContextFromRoot(*d_context);

  auto simulator = std::make_unique<systems::Simulator<double>>(std::move(diagram), std::move(d_context));
  if (VISUALIZE and scene_graph) {
    simulator->set_target_realtime_rate(1.);
  }
  systems::ResetIntegratorFromFlags<double>(simulator.get(), "runge_kutta2", max_step_size);

  // sanity check.
  plant->GetMyContextFromRoot(simulator->get_context());
  fmt::print("{}\n", simulator->get_system().GetGraphvizString());
  return {std::move(simulator), calc_output};
}


class SimulationChecker {
 public:
  void run(Simulator* simulator, CalcOutput calc_output) {
    double dt = 0.001;
    double end_time = 1.;
    const std::string prefix("    ");
    auto& d_context = simulator->get_context();

    Frames frames;

    while (d_context.get_time() + dt / 2 < end_time) {
      simulator->AdvanceTo(d_context.get_time() + dt);
      auto output = calc_output(d_context);
      frames.add(d_context.get_time(), output);
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

  std::string summary() const { return {}; };  // TODO

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
    case ResimulateStyle::kReuse: {
      auto [simulator, calc_output] = make_simulator(setup);
      auto& d_context = simulator->get_mutable_context();
      simulator->Initialize();
      auto d_context_initial = d_context.Clone();
      for (int k = 0; k < num_sim_trials; k++) {
        fmt::print("  index: {}\n", k);
        d_context.SetTimeStateAndParametersFrom(*d_context_initial);
        checker.run(simulator.get(), calc_output);
      }
      break;
    }
    default:
      throw std::runtime_error("style not implemented!");
  };

  return {style,fmt::format("{} {}", num_sim_trials, setup.plant_time_step)};
}


int do_main() {
  auto setup = find_dammit(setup_map(), std::string("Discrete_NoGeometry"), "unknown setup");
  simulate_trials(ResimulateStyle::kReuse, 2, setup);
  return 0;
}

}
  }

int main(int, const char* []) {
  return drake::tmp::do_main();
}
