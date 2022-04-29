#pragma once

#include <map>
#include <set>
#include <string>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/geometry_set.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace internal {

class CollisionFilterGroupResolver {
 public:
  CollisionFilterGroupResolver(MultibodyPlant<double>* plant);
  void AddGroup(const std::string& group_name,
                std::set<std::string> body_names,
                std::optional<ModelInstanceIndex> model_instance);
  void AddPair(const std::string& group_name_a,
               const std::string& group_name_b,
               std::optional<ModelInstanceIndex> model_instance);
  void Resolve();

 private:
  std::string FullyQualify(
      const std::string& name,
      std::optional<ModelInstanceIndex> model_instance) const;
  bool IsGroupDefined(const std::string& group_name) const;
  MultibodyPlant<double>* plant_;
  std::map<std::string, geometry::GeometrySet> groups_;
  std::set<SortedPair<std::string>> pairs_;
};

}  // namespace internal
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
