#include <deque>
#include <map>
#include <string>
#include <tuple>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include <gflags/gflags.h>

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

DEFINE_double(end_time, 1.0, "Length of simulation (seconds).");
DEFINE_int64(meta_trials, 2, "Number of meta trials.");
DEFINE_int64(sim_trials, 4, "Number of sim trials within a meta trial.");
DEFINE_string(styles, "", "Styles to run, comma-separated list.");
DEFINE_string(setups, "", "Setups to run, comma-separated list.");
DEFINE_int64(float_precision, 4,
             "Precision to use when string formatting doubles for comparision");


#define FLOAT_FMT "{:.{}g}"

std::string float_fmt(double x) {
  return fmt::format(FLOAT_FMT, x, FLAGS_float_precision);
}

template <typename T>
std::string vec_fmt(const T& v) {
  std::string result(fmt::format("[" FLOAT_FMT, v(0), FLAGS_float_precision));
  for (int k = 1; k < v.size(); k++) {
    result += fmt::format(" " FLOAT_FMT, v(k), FLAGS_float_precision);
  }
  result += "]";
  return result;
}

template <typename T>
void append(std::vector<T>* dst, const std::vector<T>& src) {
  dst->reserve(dst->size() + src.size());
  for (const auto& x : src) { dst->push_back(x); }
};


// Replacement for parts of difflib.
class Diff {
 public:
  Diff() {}
  std::vector<std::string> compare(
      const std::vector<std::string>& a, const std::vector<std::string>& b) {
    auto cruncher = difflib::MakeSequenceMatcher(a, b);
    std::vector<std::string> results;
    for (const auto& opcode : cruncher.get_opcodes()) {
      auto [tag, alo, ahi, blo, bhi] = opcode;
      if (tag == "replace") {
        append(&results, fancy_replace(a, alo, ahi, b, blo, bhi));
      } else if (tag == "delete") {
        append(&results, dump("-", a, alo, ahi));
      } else if (tag == "insert") {
        append(&results, dump("+", b, blo, bhi));
      } else if (tag == "equal") {
        append(&results, dump(" ", a, alo, ahi));
      } else {
        throw std::runtime_error("unknown compare opcode tag");
      }
    }
    return results;
  }
 private:
  std::vector<std::string> dump(
      const std::string& tag,
      const std::vector<std::string>& lines,
      size_t lo, size_t hi) {
    std::vector<std::string> results;
    for (size_t k = lo;  k < hi; k++) {
      results.push_back(fmt::format("{} {}", tag, lines[k]));
    }
    return results;
  }

  std::vector<std::string> plain_replace(
      const std::vector<std::string>& a, size_t alo, size_t ahi,
      const std::vector<std::string>& b, size_t blo, size_t bhi) {
    DRAKE_ASSERT(alo < ahi && blo < bhi);
    std::vector<std::string> results;
    if (bhi - blo < ahi - alo) {
      append(&results, dump("+", b, blo, bhi));
      append(&results, dump("-", a, alo, ahi));
    } else {
      append(&results, dump("-", a, alo, ahi));
      append(&results, dump("+", b, blo, bhi));
    }
    return results;
  }

  std::vector<std::string> fancy_replace(
      const std::vector<std::string>& a, size_t alo, size_t ahi,
      const std::vector<std::string>& b, size_t blo, size_t bhi) {
    std::vector<std::string> results;
    double best_ratio = 0.74;
    double cutoff = 0.75;
    auto cruncher = difflib::MakeSequenceMatcher(std::string(), std::string());
    std::optional<size_t> eqi, eqj;
    size_t best_i{}, best_j{};
    for (size_t j = blo; j < bhi; j++ ) {
      const auto& bj = b[j];
      cruncher.set_seq2(bj);
      for (size_t i = alo; i < ahi; i++) {
        const auto& ai = a[i];
        if (ai == bj) {
          if (!eqi) {
            eqi = i;
            eqj = j;
          }
          continue;
        }
        cruncher.set_seq1(ai);
        if (cruncher.ratio() > best_ratio) {
          best_ratio = cruncher.ratio();
          best_i = i;
          best_j = j;
        }
      }
    }
    if (best_ratio < cutoff) {
      if (eqi) {
        return plain_replace(a, alo, ahi, b, blo, bhi);
      }
      best_i = *eqi;
      best_j = *eqj;
      best_ratio = 1.0;
    } else {
      eqi = std::nullopt;
    }
    append(&results, fancy_helper(a, alo, best_i, b, blo, best_j));
    auto aelt = a[best_i];
    auto belt = b[best_j];
    if (!eqi) {
      // # pump out a '-', '?', '+', '?' quad for the synched lines
      std::string atags, btags;
      cruncher.set_seq(aelt, belt);
      for (const auto& opcode : cruncher.get_opcodes()) {
        auto [tag, ai1, ai2, bj1, bj2] = opcode;
        size_t la = ai2 - ai1;
        size_t lb = bj2 - bj1;
        if (tag == "replace") {
          atags += std::string(la, '^');
          btags += std::string(lb, '^');
        } else if (tag == "delete") {
          atags += std::string(la, '-');
        } else if (tag == "insert") {
          btags += std::string(lb, '+');
        } else if (tag == "equal") {
          atags += std::string(la, ' ');
          btags += std::string(lb, ' ');
        } else {
          throw std::runtime_error("unknown tag in fancy_replace");
        }
      }
      append(&results, qformat(aelt, belt, atags, btags));
    } else {
      results.push_back(std::string("  ") + aelt);
    }
    append(&results, fancy_helper(a, best_i + 1, ahi, b, best_j + 1, bhi));
    return results;
  }

  std::vector<std::string> fancy_helper(
      const std::vector<std::string>& a, size_t alo, size_t ahi,
      const std::vector<std::string>& b, size_t blo, size_t bhi) {
    std::vector<std::string> results;
    if (alo < ahi) {
      if (blo < bhi) {
        append(&results, fancy_replace(a, alo, ahi, b, blo, bhi));
      } else {
        append(&results, dump("-", a, alo, ahi));
      }
    } else if (blo < bhi) {
      append(&results, dump("+", b, blo, bhi));
    }
    return results;
  }

  std::vector<std::string> qformat(std::string aline, std::string bline,
                                   std::string atags, std::string btags) {
    std::vector<std::string> results;
    auto count_leading = [](const std::string& line, char c) {
      size_t i = 0;
      size_t n = line.size();
      while (i < n && line[i] == c) {
        i++;
      }
      return i;
    };
    auto rtrim = [](const std::string& s) {
      size_t last = s.find_last_not_of(" \t");
      return s.substr(0, (last + 1));
    };
    size_t common = std::min(count_leading(aline, '\t'),
                             count_leading(bline, '\t'));
    common = std::min(common, count_leading(atags.substr(0, common), ' '));
    common = std::min(common, count_leading(btags.substr(0, common), ' '));
    atags = rtrim(atags.substr(common));
    btags = rtrim(btags.substr(common));
    results.push_back(std::string("- ") + aline);
    if (!atags.empty()) {
      results.push_back(fmt::format("? {}{}\n", std::string(common, '\t'), atags));
    }
    results.push_back(std::string("- ") + bline);
    if (!btags.empty()) {
      results.push_back(fmt::format("? {}{}\n", std::string(common, '\t'), btags));
    }
    return results;
  }
};

// Replacement for python textwrap.indent, sorta.
std::vector<std::string> split( const std::string& text, char delim='\n') {
    std::vector<std::string> result;
    std::stringstream ss(text);
    std::string to;
    while (std::getline(ss, to, delim)) {
      result.push_back(to);
    }
    return result;
}

std::string indent(const std::string& text, const std::string& prefix) {
  std::stringstream ss(text);
  std::string to;
  std::string result;

  while (std::getline(ss, to,'\n')) {
    result += prefix + to + "\n";
  }
  return result;
}

std::string text_diff(const std::string& a, const std::string& b) {
  auto a_lines = split(a);
  auto b_lines = split(b);
  Diff diff;
  std::string result;
  if (a_lines.size() == b_lines.size()) {
    for (int k = 0; k < static_cast<int>(a_lines.size()); k++) {
      auto cmp_lines = diff.compare({a_lines[k]}, {b_lines[k]});
      for (const auto& line : cmp_lines) { result += line + "\n"; }
    }
  } else {
    auto cmp_lines = diff.compare(a_lines, b_lines);
    for (const auto& line : cmp_lines) { result += line + "\n"; }
  }
  return result;
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
      ss << fmt::format("diff[{}], t: {}\n", index_label, float_fmt(t))
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
    if (setup.tag == SetupEnum::Discrete_NoGeometry) {
      // fmt::print("q: {}", vec_fmt(q)); //XXX
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
  if (VISUALIZE and scene_graph) {
    simulator->set_target_realtime_rate(1.);
  }
  systems::ResetIntegratorFromFlags<double>(simulator.get(), "runge_kutta2", max_step_size);

  // sanity check.
  plant->GetMyContextFromRoot(simulator->get_context());
  // fmt::print("{}\n", simulator->get_system().GetGraphvizString());
  return {std::move(simulator), calc_output};
}


class SimulationChecker {
 public:
  void run(Simulator* simulator, CalcOutput calc_output) {
    double dt = 0.001;
    double end_time = FLAGS_end_time;
    const std::string prefix("    ");
    simulator->Initialize();  // Proposed patch for Reuse.
    auto& d_context = simulator->get_context();

    Frames frames;

    auto output = calc_output(d_context);
    frames.add(d_context.get_time(), output);
    while (d_context.get_time() + dt / 2 < end_time) {
      simulator->AdvanceTo(d_context.get_time() + dt);
      output = calc_output(d_context);
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
      simulator->Initialize();
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
  logging::set_log_level("err");
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
