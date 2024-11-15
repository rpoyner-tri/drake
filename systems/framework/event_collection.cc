#include "drake/systems/framework/event_collection.h"

#define DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_EVENTS(SomeType)	\
  template SomeType<::drake::systems::PublishEvent<double>>;		\
  template SomeType<::drake::systems::DiscreteUpdateEvent<double>>;	\
  template SomeType<::drake::systems::UnrestrictedUpdateEvent<double>>;	\
  template SomeType<::drake::systems::PublishEvent<::drake::AutoDiffXd>>; \
  template SomeType<::drake::systems::DiscreteUpdateEvent<::drake::AutoDiffXd>>; \
  template SomeType<::drake::systems::UnrestrictedUpdateEvent<::drake::AutoDiffXd>>; \
  template SomeType<::drake::systems::PublishEvent<::drake::symbolic::Expression>>; \
  template SomeType<::drake::systems::DiscreteUpdateEvent<::drake::symbolic::Expression>>; \
  template SomeType<::drake::systems::UnrestrictedUpdateEvent<::drake::symbolic::Expression>>;

namespace drake {
namespace systems {
template <typename Event>
EventCollection<Event>::~EventCollection() {}
template <typename Event>
LeafEventCollection<Event>::~LeafEventCollection() {}
template <typename Event>
DiagramEventCollection<Event>::~DiagramEventCollection() {}
  
template <typename T>
CompositeEventCollection<T>::~CompositeEventCollection() {}
template <typename T>
LeafCompositeEventCollection<T>::~LeafCompositeEventCollection() {}
  template <typename T>  
DiagramCompositeEventCollection<T>::~DiagramCompositeEventCollection() {}
}  // namespace systems
}  // namespace drake

DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_EVENTS(
    class ::drake::systems::EventCollection);
DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_EVENTS(
    class ::drake::systems::LeafEventCollection);
DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_EVENTS(
    class ::drake::systems::DiagramEventCollection);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::CompositeEventCollection);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafCompositeEventCollection);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramCompositeEventCollection);

// Due to a circular dependency (the various event types depend on the
// collection types and vice versa) it's not possible to instantiate the event
// types without the collections available.  For that reason, we create the
// instances for both in this file.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::WitnessTriggeredEventData);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Event);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::PublishEvent);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteUpdateEvent);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::UnrestrictedUpdateEvent);
