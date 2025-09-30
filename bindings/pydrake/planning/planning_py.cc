#include "drake/bindings/pydrake/planning/planning_py.h"

namespace drake {
namespace pydrake {

NB_MODULE(planning, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = R"""(
A collection of motion planning algorithms for finding configurations
and/or trajectories of dynamical systems.
)""";

  py::module_::import_("pydrake.geometry");
  py::module_::import_("pydrake.multibody.parsing");
  py::module_::import_("pydrake.multibody.plant");
  py::module_::import_("pydrake.multibody.rational");
  py::module_::import_("pydrake.solvers");
  py::module_::import_("pydrake.symbolic");
  py::module_::import_("pydrake.systems.framework");
  py::module_::import_("pydrake.systems.primitives");
  py::module_::import_("pydrake.trajectories");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefinePlanningRobotDiagram(m);
  internal::DefinePlanningCollisionCheckerInterfaceTypes(m);
  internal::DefinePlanningCollisionChecker(m);
  internal::DefinePlanningDofMask(m);
  internal::DefinePlanningJointLimits(m);
  internal::DefinePlanningGraphAlgorithms(m);
  internal::DefinePlanningTrajectoryOptimization(m);
  internal::DefinePlanningVisibilityGraph(m);
  internal::DefinePlanningIrisCommon(m);
  internal::DefinePlanningIrisNp2(m);
  internal::DefinePlanningIrisZo(m);
  internal::DefinePlanningIrisFromCliqueCover(m);
  internal::DefinePlanningZmpPlanner(m);
}

}  // namespace pydrake
}  // namespace drake
