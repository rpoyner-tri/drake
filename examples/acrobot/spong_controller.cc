#include "drake/examples/acrobot/spong_controller.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace acrobot {

template <typename T>
AcrobotSpongController<T>::~AcrobotSpongController() {}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::acrobot::AcrobotSpongController);
