#include <benchmark/benchmark.h>
#include <fmt/format.h>

#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace geometry {
namespace internal {

std::string ComplianceType(int arg) {
  switch (arg) {
    case 0: return "undefined";
    case 1: return "compliant";
    case 2: return "rigid";
  };
  DRAKE_UNREACHABLE();
}

class HydroelasticateBenchmark : public benchmark::Fixture {
 public:
  void SetupScene(const benchmark::State& state) {
    DefaultProximityProperties dpp;
    SceneGraphConfig scene_graph_config;
    scene_graph_config.default_proximity_properties.compliance_type =
        ComplianceType(state.range(0));
    ProximityProperties props;
    if (scene_graph_config.default_proximity_properties.compliance_type ==
        "unknown") {
      // Simulate a manually annotated model.
      props.UpdateProperty(kHydroGroup, kComplianceType,
                           HydroelasticType::kSoft);
      props.UpdateProperty(
          kHydroGroup, kElastic,
          scene_graph_config.default_proximity_properties.hydroelastic_modulus);
      props.UpdateProperty(
          kHydroGroup, kRezHint,
          scene_graph_config.default_proximity_properties.mesh_resolution_hint);
    }
    int num_geoms = state.range(1);
    scene_graph_ = std::make_unique<SceneGraph<double>>(scene_graph_config);

    auto source_id = scene_graph_->RegisterSource("benchmark");
    auto frame_id =
        scene_graph_->RegisterFrame(source_id, GeometryFrame{"frame0"});
    for (int k = 0; k < num_geoms; ++k) {
      auto geom_id = scene_graph_->RegisterGeometry(
          source_id, frame_id,
          std::make_unique<GeometryInstance>(math::RigidTransformd(),
                                             Box(1.0, 1.0, 1.0),
                                             fmt::format("box{}", k)));
      scene_graph_->AssignRole(source_id, geom_id, props);
    }
  }

  std::unique_ptr<SceneGraph<double>> scene_graph_;
};

BENCHMARK_DEFINE_F(HydroelasticateBenchmark, CreateDefaultContext)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupScene(state);
  for (auto _ : state) {
    scene_graph_->CreateDefaultContext();
  }
}

BENCHMARK_REGISTER_F(HydroelasticateBenchmark, CreateDefaultContext)
    ->Unit(benchmark::kMillisecond)
    ->Args({0, 1})
    ->Args({1, 1})
    ->Args({2, 1})
    ->Args({0, 100})
    ->Args({1, 100})
    ->Args({2, 100});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
