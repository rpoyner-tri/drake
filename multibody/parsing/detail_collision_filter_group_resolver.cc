#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"

#include "drake/multibody/parsing/scoped_names.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace internal {

CollisionFilterGroupResolver::CollisionFilterGroupResolver(
    MultibodyPlant<double>* plant) : plant_(plant) {}

void CollisionFilterGroupResolver::AddGroup(
    const std::string& group_name,
    std::set<std::string> body_names,
    std::optional<ModelInstanceIndex> model_instance) {
  DRAKE_DEMAND(!group_name.empty());
  if (model_instance) {
    DRAKE_DEMAND(*model_instance < plant_->num_model_instances());
  }

  geometry::GeometrySet geometry_set;
  for (const auto& body_name : body_names) {
    DRAKE_DEMAND(!body_name.empty());
    std::optional<ModelInstanceIndex> body_model;
    ScopedName scoped_name = ParseScopedName(body_name);

    if (scoped_name.instance_name.empty()) {
      if (model_instance) {
        // Local and unprefixed; look within this model.
        body_model = *model_instance;
      } else {
        // Global and unprefixed.
        // Nothing to do.
      }
    } else {
      // Within local scopes, scoped names are not supported.
      // XXX TODO diagnostics?
      DRAKE_THROW_UNLESS(!model_instance);
      body_model = plant_->GetModelInstanceByName(scoped_name.instance_name);
    }

    const auto& body = body_model ?
                       plant_->GetBodyByName(body_name.c_str(), *body_model) :
                       plant_->GetBodyByName(body_name.c_str());
    geometry_set.Add(plant_->GetBodyFrameIdOrThrow(body.index()));
  }
  groups_.insert({FullyQualify(group_name, model_instance), geometry_set});
}

void CollisionFilterGroupResolver::AddPair(
    const std::string& group_name_a,
    const std::string& group_name_b,
    std::optional<ModelInstanceIndex> model_instance) {
  DRAKE_DEMAND(!group_name_a.empty());
  DRAKE_DEMAND(!group_name_b.empty());
  if (model_instance) {
    DRAKE_DEMAND(*model_instance < plant_->num_model_instances());
  }

  // Store group pairs by fully qualified name. The groups don't need to
  // actually be defined until Resolve() time.
  const std::string name_a = FullyQualify(group_name_a, model_instance);
  const std::string name_b = FullyQualify(group_name_b, model_instance);

  // These two group names are allowed to be identical, which means the
  // bodies inside this collision filter group should be collision excluded
  // among each other.
  pairs_.insert({name_a.c_str(), name_b.c_str()});
}

void CollisionFilterGroupResolver::Resolve() {
  for (const auto& [name_a, name_b] : pairs_) {
    // XXX TODO diagnostics?
    DRAKE_THROW_UNLESS(IsGroupDefined(name_a));
    // XXX TODO diagnostics?
    DRAKE_THROW_UNLESS(IsGroupDefined(name_b));
    plant_->ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
        {name_a, groups_.find(name_a)->second},
        {name_b, groups_.find(name_b)->second});
  }
}

std::string CollisionFilterGroupResolver::FullyQualify(
    const std::string& name,
    std::optional<ModelInstanceIndex> model_instance) const {
  if (!model_instance) {
    // Names found in global scope are just themselves.
    return name;
  }

  // Names found in local scope are not allowed to be scoped names, since we
  // don't support outbound references from a local model scope to somewhere
  // else.
  ScopedName scoped_name = ParseScopedName(name);
  // XXX TODO diagnostics?
  DRAKE_THROW_UNLESS(scoped_name.instance_name.empty());

  return PrefixName(GetInstanceScopeName(*plant_, *model_instance), name);
}

bool CollisionFilterGroupResolver::IsGroupDefined(
    const std::string& group_name) const {
    return groups_.count(group_name) > 0;
}

}  // namespace internal
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
