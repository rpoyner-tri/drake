# -*- coding: utf-8 -*-

import pydrake.systems.framework

import copy
import gc
from textwrap import dedent

import unittest
import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.common import RandomGenerator
from pydrake.common.test_utilities import numpy_compare
from pydrake.common.value import AbstractValue, Value
from pydrake.examples import PendulumPlant, RimlessWheel
from pydrake.symbolic import Expression
from pydrake.systems.analysis import (
    GetIntegrationSchemes,
    IntegratorBase, IntegratorBase_,
    PrintSimulatorStatistics,
    ResetIntegratorFromFlags,
    RungeKutta2Integrator,
    Simulator, Simulator_,
)
from pydrake.systems.framework import (
    BasicVector, BasicVector_,
    CacheEntry,
    ContextBase,
    Context, Context_,
    ContinuousState, ContinuousState_,
    Diagram, Diagram_,
    DiagramBuilder, DiagramBuilder_,
    DiscreteUpdateEvent, DiscreteUpdateEvent_,
    DiscreteValues, DiscreteValues_,
    Event, Event_,
    EventStatus,
    InputPort, InputPort_,
    InputPortIndex,
    kUseDefaultName,
    LeafContext, LeafContext_,
    LeafSystem, LeafSystem_,
    OutputPort, OutputPort_,
    OutputPortIndex,
    Parameters, Parameters_,
    PeriodicEventData,
    PublishEvent, PublishEvent_,
    State, State_,
    Subvector, Subvector_,
    Supervector, Supervector_,
    System, System_,
    SystemVisitor,
    SystemBase,
    SystemOutput, SystemOutput_,
    SystemScalarConverter,
    VectorBase, VectorBase_,
    TriggerType,
    VectorSystem, VectorSystem_,
    _ExternalSystemConstraint,
)
from pydrake.systems.primitives import (
    Adder, Adder_,
    ConstantValueSource,
    ConstantVectorSource, ConstantVectorSource_,
    Integrator,
    LinearSystem,
    PassThrough, PassThrough_,
    ZeroOrderHold,
)
from pydrake.systems.test.test_util import (
    MyVector2,
    DummySystemA,
    MakeDummySystem,
)
from pydrake.systems.test_utilities import framework_test_util

# TODO(eric.cousineau): The scope of this test file and `custom_test.py`
# is poor. Move these tests into `framework_test` and `analysis_test`, and
# ensure that the tests reflect this, even if there is some coupling.


class TestGeneral(unittest.TestCase):
    def _check_instantiations(
            self, template, default_cls, supports_symbolic=True):
        self.assertIs(template[None], default_cls)
        self.assertIs(template[float], default_cls)
        self.assertIsNot(template[AutoDiffXd], default_cls)
        if supports_symbolic:
            self.assertIsNot(template[Expression], default_cls)

    def _compare_system_instances(self, lhs, rhs):
        # Compares two different scalar type instantiation instances of a
        # system.
        self.assertEqual(lhs.num_input_ports(), rhs.num_input_ports())
        self.assertEqual(
            lhs.num_output_ports(), rhs.num_output_ports())
        for i in range(lhs.num_input_ports()):
            lhs_port = lhs.get_input_port(i, warn_deprecated=False)
            rhs_port = rhs.get_input_port(i, warn_deprecated=False)
            self.assertEqual(lhs_port.size(), rhs_port.size())
        for i in range(lhs.num_output_ports()):
            lhs_port = lhs.get_output_port(i, warn_deprecated=False)
            rhs_port = rhs.get_output_port(i, warn_deprecated=False)
            self.assertEqual(lhs_port.size(), rhs_port.size())

    def test_system_base_api(self):
        # Test a system with a different number of inputs from outputs.
        system = Adder(3, 10)
        self.assertIsInstance(system, SystemBase)
        self.assertEqual(
            system.GetSystemType(),
            "drake::systems::Adder<double>")
        system.set_name(name="adder")
        self.assertEqual(system.get_name(), "adder")
        self.assertEqual(system.GetSystemName(), "adder")
        self.assertEqual(system.GetSystemPathname(), "::adder")
        self.assertEqual(system.num_input_ports(), 3)
        self.assertEqual(system.num_output_ports(), 1)
        self.assertEqual(system.num_continuous_states(), 0)
        self.assertEqual(system.num_discrete_state_groups(), 0)
        self.assertEqual(system.num_abstract_states(), 0)
        self.assertEqual(system.implicit_time_derivatives_residual_size(), 0)
        self.assertTrue(system.HasInputPort("u1"))
        u1 = system.GetInputPort("u1")
        self.assertEqual(u1.get_name(), "u1")
        self.assertIn("u1", u1.GetFullDescription())
        self.assertEqual(u1.get_index(), 1)
        self.assertEqual(u1.size(), 10)
        self.assertIsNotNone(u1.ticket())
        self.assertIsInstance(u1.Allocate(), Value[BasicVector])
        self.assertIs(u1.get_system(), system)
        self.assertTrue(system.HasOutputPort("sum"))
        y = system.GetOutputPort("sum")
        self.assertEqual(y.get_name(), "sum")
        self.assertEqual(y.get_index(), 0)
        self.assertIsInstance(y.Allocate(), Value[BasicVector])
        self.assertIs(y.get_system(), system)
        cache_entry = y.cache_entry()
        self.assertFalse(cache_entry.is_disabled_by_default())
        y.disable_caching_by_default()
        self.assertTrue(cache_entry.is_disabled_by_default())
        self.assertEqual(y, system.get_output_port())
        # TODO(eric.cousineau): Consolidate the main API tests for `System`
        # to this test point.

    def test_context_base_api(self):
        system = Adder(3, 10)
        context = system.AllocateContext()
        self.assertIsInstance(context, ContextBase)
        self.assertEqual(context.num_input_ports(), 3)
        self.assertEqual(context.num_output_ports(), 1)
        context.DisableCaching()
        context.EnableCaching()
        context.SetAllCacheEntriesOutOfDate()
        context.FreezeCache()
        self.assertTrue(context.is_cache_frozen())
        context.UnfreezeCache()
        self.assertFalse(context.is_cache_frozen())

    def test_context_api(self):
        system = Adder(3, 10)
        context = system.AllocateContext()
        self.assertIsInstance(
            context.get_continuous_state(), ContinuousState)
        self.assertIsInstance(
            context.get_mutable_continuous_state(), ContinuousState)
        self.assertIsInstance(
            context.get_continuous_state_vector(), VectorBase)
        self.assertIsInstance(
            context.get_mutable_continuous_state_vector(), VectorBase)
        system.SetDefaultContext(context=context)

        # Check random context method.
        system.SetRandomContext(context=context, generator=RandomGenerator())

        context = system.CreateDefaultContext()
        self.assertIsInstance(
            context.get_continuous_state(), ContinuousState)
        self.assertIsInstance(
            context.get_mutable_continuous_state(), ContinuousState)
        self.assertIsInstance(
            context.get_continuous_state_vector(), VectorBase)
        self.assertIsInstance(
            context.get_mutable_continuous_state_vector(), VectorBase)
        self.assertTrue(context.is_stateless())
        self.assertFalse(context.has_only_continuous_state())
        self.assertFalse(context.has_only_discrete_state())
        self.assertEqual(context.num_total_states(), 0)
        # TODO(eric.cousineau): Consolidate main API tests for `Context` here.

        # Test methods with two scalar types.
        for T in [float, AutoDiffXd, Expression]:
            systemT = Adder_[T](3, 10)
            contextT = systemT.CreateDefaultContext()
            for U in [float, AutoDiffXd, Expression]:
                systemU = Adder_[U](3, 10)
                contextU = systemU.CreateDefaultContext()
                contextU.SetTime(0.5)
                contextT.SetStateAndParametersFrom(contextU)
                contextT.SetTimeStateAndParametersFrom(contextU)
                if T == float:
                    self.assertEqual(contextT.get_time(), 0.5)
                elif T == AutoDiffXd:
                    self.assertEqual(contextT.get_time().value(), 0.5)
                else:
                    self.assertEqual(contextT.get_time().Evaluate(), 0.5)

        pendulum = PendulumPlant()
        context = pendulum.CreateDefaultContext()
        self.assertEqual(context.num_numeric_parameter_groups(), 1)
        self.assertEqual(pendulum.num_numeric_parameter_groups(), 1)
        self.assertTrue(
            context.get_parameters().get_numeric_parameter(0) is
            context.get_numeric_parameter(index=0))
        self.assertTrue(
            context.get_mutable_parameters().get_mutable_numeric_parameter(
                0) is context.get_mutable_numeric_parameter(index=0))
        self.assertEqual(context.num_abstract_parameters(), 0)
        self.assertEqual(pendulum.num_numeric_parameter_groups(), 1)
        # TODO(russt): Bind _Declare*Parameter or find an example with an
        # abstract parameter to actually call this method.
        self.assertTrue(hasattr(context, "get_abstract_parameter"))
        self.assertTrue(hasattr(context, "get_mutable_abstract_parameter"))
        x = np.array([0.1, 0.2])
        context.SetContinuousState(x)
        np.testing.assert_equal(
            context.get_continuous_state().CopyToVector(), x)
        np.testing.assert_equal(
            context.get_continuous_state_vector().CopyToVector(), x)
        context.SetTimeAndContinuousState(0.3, 2*x)
        np.testing.assert_equal(context.get_time(), 0.3)
        np.testing.assert_equal(
            context.get_continuous_state_vector().CopyToVector(), 2*x)
        self.assertNotEqual(pendulum.EvalPotentialEnergy(context=context), 0)
        self.assertNotEqual(pendulum.EvalKineticEnergy(context=context), 0)

        # RimlessWheel has a single discrete variable and a bool abstract
        # variable.
        rimless = RimlessWheel()
        context = rimless.CreateDefaultContext()
        x = np.array([1.125])
        context.SetDiscreteState(xd=2 * x)
        np.testing.assert_equal(
            context.get_discrete_state_vector().CopyToVector(), 2 * x)
        context.SetDiscreteState(group_index=0, xd=3 * x)
        np.testing.assert_equal(
            context.get_discrete_state_vector().CopyToVector(), 3 * x)
        # Just verify that the third overload is present.
        context.SetDiscreteState(context.get_discrete_state())

        def check_abstract_value_zero(context, expected_value):
            # Check through Context, State, and AbstractValues APIs.
            self.assertEqual(context.get_abstract_state(index=0).get_value(),
                             expected_value)
            self.assertEqual(context.get_abstract_state().get_value(
                index=0).get_value(), expected_value)
            self.assertEqual(context.get_state().get_abstract_state()
                             .get_value(index=0).get_value(), expected_value)

        context.SetAbstractState(index=0, value=True)
        check_abstract_value_zero(context, True)
        context.SetAbstractState(index=0, value=False)
        check_abstract_value_zero(context, False)
        value = context.get_mutable_state().get_mutable_abstract_state()\
            .get_mutable_value(index=0)
        value.set_value(True)
        check_abstract_value_zero(context, True)

    def test_event_api(self):
        # TriggerType - existence check.
        TriggerType.kUnknown
        TriggerType.kInitialization
        TriggerType.kForced
        TriggerType.kTimed
        TriggerType.kPeriodic
        TriggerType.kPerStep
        TriggerType.kWitness

        # PublishEvent.
        # TODO(eric.cousineau): Test other event types when it is useful to
        # expose them.

        def callback(context, event): pass

        event = PublishEvent(callback=callback)
        self.assertIsInstance(event, Event)
        event = PublishEvent(
            trigger_type=TriggerType.kInitialization, callback=callback)
        self.assertIsInstance(event, Event)
        self.assertEqual(event.get_trigger_type(), TriggerType.kInitialization)

        def system_callback(system, context, event): pass

        event = PublishEvent(system_callback=system_callback)
        self.assertIsInstance(event, Event)
        event = PublishEvent(
            trigger_type=TriggerType.kInitialization,
            system_callback=system_callback)
        self.assertIsInstance(event, Event)
        self.assertEqual(event.get_trigger_type(), TriggerType.kInitialization)

        # Simple discrete-time system.
        system1 = LinearSystem(A=[1], B=[1], C=[1], D=[1], time_period=0.1)
        periodic_data = system1.GetUniquePeriodicDiscreteUpdateAttribute()
        self.assertIsInstance(periodic_data, PeriodicEventData)
        periodic_data.period_sec()
        periodic_data.offset_sec()
        self.assertTrue(copy.copy(periodic_data) is not periodic_data)
        is_diff_eq, period = system1.IsDifferenceEquationSystem()
        self.assertTrue(is_diff_eq)
        self.assertFalse(system1.IsDifferentialEquationSystem())
        self.assertEqual(period, periodic_data.period_sec())
        context = system1.CreateDefaultContext()
        system1.get_input_port(0).FixValue(context, 0.0)
        updated_discrete = system1.EvalUniquePeriodicDiscreteUpdate(context)
        self.assertEqual(updated_discrete.num_groups(),
                         context.get_discrete_state().num_groups())

        # Simple continuous-time system.
        system2 = LinearSystem(A=[1], B=[1], C=[1], D=[1], time_period=0.0)
        periodic_data = system2.GetUniquePeriodicDiscreteUpdateAttribute()
        self.assertIsNone(periodic_data)
        is_diff_eq, period = system2.IsDifferenceEquationSystem()
        self.assertFalse(is_diff_eq)
        self.assertTrue(system2.IsDifferentialEquationSystem())

    def test_continuous_state_api(self):
        self.assertEqual(ContinuousState().size(), 0)
        custom_vector = MyVector2(np.ones(2))
        state = ContinuousState(state=custom_vector)
        self.assertIsInstance(state.get_vector(), MyVector2)
        self.assertEqual(state.size(), 2)
        self.assertEqual(state.num_q(), 0)
        self.assertEqual(state.num_v(), 0)
        self.assertEqual(state.num_z(), 2)
        state = ContinuousState(state=custom_vector, num_q=1, num_v=1, num_z=0)
        self.assertIsInstance(state.get_vector(), MyVector2)
        self.assertEqual(state.size(), 2)
        self.assertEqual(state.num_q(), 1)
        self.assertEqual(state.num_v(), 1)
        self.assertEqual(state.num_z(), 0)
        state = ContinuousState(state=BasicVector(2))
        self.assertEqual(state.size(), 2)
        state = ContinuousState(state=BasicVector(np.arange(6)), num_q=3,
                                num_v=2, num_z=1)
        state_clone = state.Clone()
        self.assertTrue(state_clone is not state)
        self.assertEqual(state.size(), 6)
        self.assertEqual(state.num_q(), 3)
        self.assertEqual(state.num_v(), 2)
        self.assertEqual(state.num_z(), 1)
        self.assertEqual(state[1], 1.0)
        state[1] = 11.
        self.assertEqual(state[1], 11.)
        self.assertEqual(state.get_vector().size(), 6)
        self.assertEqual(state.get_mutable_vector().size(), 6)
        self.assertEqual(state.get_generalized_position().size(), 3)
        self.assertEqual(state.get_mutable_generalized_position().size(), 3)
        self.assertEqual(state.get_generalized_velocity().size(), 2)
        self.assertEqual(state.get_mutable_generalized_velocity().size(), 2)
        self.assertEqual(state.get_misc_continuous_state().size(), 1)
        self.assertEqual(state.get_mutable_misc_continuous_state().size(), 1)
        state.SetFrom(ContinuousState(BasicVector(6), 3, 2, 1))
        state.SetFromVector(value=3*np.arange(6))
        self.assertEqual(len(state.CopyToVector()), 6)

    @numpy_compare.check_all_types
    def test_discrete_value_api(self, T):
        DiscreteValues = DiscreteValues_[T]
        BasicVector = BasicVector_[T]
        cast = np.vectorize(T)

        self.assertEqual(DiscreteValues().num_groups(), 0)
        discrete_values = DiscreteValues(data=[BasicVector(1), BasicVector(2)])
        self.assertEqual(discrete_values.num_groups(), 2)
        x = cast(np.array([1.23, 4.56]))
        discrete_values.set_value(1, x)
        numpy_compare.assert_equal(discrete_values.get_value(index=1), x)
        if T == float:
            numpy_compare.assert_equal(
                discrete_values.get_mutable_value(index=1), x)
        else:
            with self.assertRaises(RuntimeError):
                discrete_values.get_mutable_value(index=1)

        discrete_values = DiscreteValues(datum=BasicVector(np.arange(3)))
        self.assertEqual(discrete_values.size(), 3)
        discrete_values_clone = discrete_values.Clone()
        self.assertTrue(discrete_values_clone is not discrete_values)
        self.assertEqual(len(discrete_values.get_data()), 1)
        self.assertEqual(discrete_values.get_vector(index=0).size(), 3)
        self.assertEqual(discrete_values.get_mutable_vector(index=0).size(), 3)
        x = cast(np.array([1., 3., 4.]))
        discrete_values.set_value(x)
        numpy_compare.assert_equal(discrete_values.value(index=0), x)
        numpy_compare.assert_equal(discrete_values.get_value(), x)
        if T == float:
            numpy_compare.assert_equal(
                discrete_values.get_mutable_value(), x)
        discrete_values[1] = 5.
        numpy_compare.assert_equal(discrete_values[1], T(5.))
        if T == float:
            vector = discrete_values.get_mutable_value()
            vector[0] = 2.3
            self.assertEqual(discrete_values[0], 2.3)
        discrete_values.SetFrom(DiscreteValues_[float](BasicVector_[float](3)))

    def test_instantiations(self):
        # Quick check of instantiations for given types.
        # N.B. These checks are ordered according to their binding definitions
        # in the corresponding source file.
        # `analysis_py.cc`
        self._check_instantiations(IntegratorBase_, IntegratorBase, False)
        self._check_instantiations(Simulator_, Simulator, False)
        # `framework_py_semantics.cc`
        self._check_instantiations(Context_, Context)
        for T in [float, AutoDiffXd, Expression]:
            self.assertTrue(issubclass(Context_[T], ContextBase), repr(T))
        self._check_instantiations(LeafContext_, LeafContext)
        self._check_instantiations(Event_, Event)
        self._check_instantiations(PublishEvent_, PublishEvent)
        self._check_instantiations(DiscreteUpdateEvent_, DiscreteUpdateEvent)
        self._check_instantiations(DiagramBuilder_, DiagramBuilder)
        self._check_instantiations(OutputPort_, OutputPort)
        self._check_instantiations(SystemOutput_, SystemOutput)
        self._check_instantiations(InputPort_, InputPort)
        self._check_instantiations(Parameters_, Parameters)
        self._check_instantiations(State_, State)
        self._check_instantiations(ContinuousState_, ContinuousState)
        self._check_instantiations(DiscreteValues_, DiscreteValues)
        # `framework_py_systems.cc`
        self._check_instantiations(System_, System)
        self._check_instantiations(LeafSystem_, LeafSystem)
        self._check_instantiations(Diagram_, Diagram)
        self._check_instantiations(VectorSystem_, VectorSystem)
        # `framework_py_values.cc`
        self._check_instantiations(VectorBase_, VectorBase)
        self._check_instantiations(BasicVector_, BasicVector)
        self._check_instantiations(Supervector_, Supervector)
        self._check_instantiations(Subvector_, Subvector)

    def test_scalar_type_conversion(self):
        float_system = Adder(1, 1)
        float_context = float_system.CreateDefaultContext()
        float_system.get_input_port(0).FixValue(float_context, 1.)
        for T in [float, AutoDiffXd, Expression]:
            system = Adder_[T](1, 1)
            self.assertIsInstance(system.get_system_scalar_converter(),
                                  SystemScalarConverter)
            # N.B. Current scalar conversion does not permit conversion to and
            # from the same type.
            if T != float:
                methods = [Adder_[T].ToScalarType[float],
                           Adder_[T].ToScalarTypeMaybe[float]]
                for method in methods:
                    system_float = method(system)
                    self.assertIsInstance(system_float, System_[float])
                    self._compare_system_instances(system, system_float)
            if T != AutoDiffXd:
                methods = [Adder_[T].ToAutoDiffXd, Adder_[T].ToAutoDiffXdMaybe,
                           Adder_[T].ToScalarType[AutoDiffXd],
                           Adder_[T].ToScalarTypeMaybe[AutoDiffXd]]
                for method in methods:
                    system_ad = method(system)
                    self.assertIsInstance(system_ad, System_[AutoDiffXd])
                    self._compare_system_instances(system, system_ad)
            if T != Expression:
                methods = [Adder_[T].ToSymbolic, Adder_[T].ToSymbolicMaybe,
                           Adder_[T].ToScalarType[Expression],
                           Adder_[T].ToScalarTypeMaybe[Expression]]
                for method in methods:
                    system_sym = method(system)
                    self.assertIsInstance(system_sym, System_[Expression])
                    self._compare_system_instances(system, system_sym)
            context = system.CreateDefaultContext()
            system.FixInputPortsFrom(other_system=float_system,
                                     other_context=float_context,
                                     target_context=context)
            u = system.get_input_port(0).Eval(context)
            self.assertEqual(len(u), 1)
            if T == float:
                self.assertEqual(u[0], 1.)
            elif T == AutoDiffXd:
                self.assertEqual(u[0].value(), 1.)
            else:
                self.assertEqual(u[0].Evaluate(), 1.)

    @numpy_compare.check_all_types
    def test_port_output(self, T):
        # TODO(eric.cousineau): Find better location for this testing.
        system = ConstantVectorSource_[T]([1.])
        context = system.CreateDefaultContext()
        # Check number of output ports and value for a given context.
        output = system.AllocateOutput()
        self.assertEqual(output.num_ports(), 1)
        system.CalcOutput(context=context, outputs=output)
        if T == float:
            value = output.get_vector_data(0).get_value()
            self.assertTrue(np.allclose([1], value))
        elif T == AutoDiffXd:
            value = output.get_vector_data(0)._get_value_copy()
            # TODO(eric.cousineau): Define `isfinite` ufunc, if
            # possible, to use for `np.allclose`.
            self.assertEqual(value.shape, (1,))
            self.assertEqual(value[0], AutoDiffXd(1.))

    def test_copy(self):
        # Copy a context using `deepcopy` or `clone`.
        system = ConstantVectorSource([1])
        context = system.CreateDefaultContext()
        context_copies = [
            copy.copy(context),
            copy.deepcopy(context),
            context.Clone(),
        ]
        # TODO(eric.cousineau): Compare copies.
        for context_copy in context_copies:
            self.assertTrue(context_copy is not context)

    def test_str(self):
        """
        Tests str() methods. See ./value_test.py for testing str() and
        repr() specific to BasicVector.
        """
        # Context.
        integrator = Integrator(3)
        integrator.set_name("integrator")
        context = integrator.CreateDefaultContext()
        # N.B. This is only to show behavior of C++ string formatting in
        # Python. It is OK to update this when the upstream C++ code changes.
        self.assertEqual(
            str(context),
            dedent("""\
            ::integrator Context
            ---------------------
            Time: 0
            States:
              3 continuous states
                0 0 0

            """),
        )
        # TODO(eric.cousineau): Add more.

    def test_diagram_simulation(self):
        # TODO(eric.cousineau): Move this to `analysis_test.py`.
        # Similar to: //systems/framework:diagram_test, ExampleDiagram
        size = 3

        builder = DiagramBuilder()
        self.assertTrue(builder.empty())
        adder0 = builder.AddSystem(Adder(2, size))
        adder0.set_name("adder0")
        self.assertFalse(builder.empty())

        adder1 = builder.AddSystem(Adder(2, size))
        adder1.set_name("adder1")

        integrator = builder.AddSystem(Integrator(size))
        integrator.set_name("integrator")

        self.assertEqual(
            builder.GetSystems(),
            [adder0, adder1, integrator])
        self.assertEqual(
            builder.GetMutableSystems(),
            [adder0, adder1, integrator])

        builder.Connect(adder0.get_output_port(0), adder1.get_input_port(0))
        builder.Connect(adder1.get_output_port(0),
                        integrator.get_input_port(0))

        # Exercise naming variants.
        builder.ExportInput(adder0.get_input_port(0))
        builder.ExportInput(adder0.get_input_port(1), kUseDefaultName)
        builder.ExportInput(adder1.get_input_port(1), "third_input")
        builder.ExportOutput(integrator.get_output_port(0), "result")

        diagram = builder.Build()
        self.assertEqual(adder0.get_name(), "adder0")
        self.assertTrue(diagram.HasSubsystemNamed("adder0"))
        self.assertEqual(diagram.GetSubsystemByName("adder0"), adder0)
        self.assertEqual(
            diagram.GetSystems(),
            [adder0, adder1, integrator])
        # TODO(eric.cousineau): Figure out unicode handling if needed.
        # See //systems/framework/test/diagram_test.cc:349 (sha: bc84e73)
        # for an example name.
        diagram.set_name("test_diagram")

        simulator = Simulator(diagram)
        context = simulator.get_mutable_context()

        # Create and attach inputs.
        # TODO(eric.cousineau): Not seeing any assertions being printed if no
        # inputs are connected. Need to check this behavior.
        input0 = np.array([0.1, 0.2, 0.3])
        diagram.get_input_port(0).FixValue(context, input0)
        input1 = np.array([0.02, 0.03, 0.04])
        diagram.get_input_port(1).FixValue(context, input1)
        # Test the BasicVector overload.
        input2 = BasicVector([0.003, 0.004, 0.005])
        diagram.get_input_port(2).FixValue(context, input2)

        # Initialize integrator states.
        integrator_xc = (
            diagram.GetMutableSubsystemState(integrator, context)
                   .get_mutable_continuous_state().get_vector())
        integrator_xc.SetFromVector([0, 1, 2])

        simulator.Initialize()

        # Simulate briefly, and take full-context snapshots at intermediate
        # points.
        n = 6
        times = np.linspace(0, 1, n)
        context_log = []
        for t in times:
            simulator.AdvanceTo(t)
            # Record snapshot of *entire* context.
            context_log.append(context.Clone())

        # Test binding for PrintSimulatorStatistics
        PrintSimulatorStatistics(simulator)

        xc_initial = np.array([0, 1, 2])
        xc_final = np.array([0.123, 1.234, 2.345])

        for i, context_i in enumerate(context_log):
            t = times[i]
            self.assertEqual(context_i.get_time(), t)
            xc = context_i.get_continuous_state_vector().CopyToVector()
            xc_expected = (float(i) / (n - 1) * (xc_final - xc_initial)
                           + xc_initial)
            self.assertTrue(np.allclose(xc, xc_expected))

    def test_simulator_context_manipulation(self):
        # TODO(eric.cousineau): Move this to `analysis_test.py`.
        system = ConstantVectorSource([1])
        # Use default-constructed context.
        simulator = Simulator(system)
        self.assertTrue(simulator.has_context())
        context_default = simulator.get_mutable_context()
        self.assertIsInstance(context_default, Context)
        context = system.CreateDefaultContext()
        simulator.reset_context(context)
        # The python bindings cause the context to always use Python reference
        # counting, so the evicted context referred to by `context_default`
        # will still be alive.
        self.assertTrue(context_default.is_stateless())
        self.assertIs(context, simulator.get_mutable_context())
        simulator.reset_context(None)
        # Similar to the case above, the evicted context referred to by
        # `context` is still alive.
        self.assertTrue(context.is_stateless())
        self.assertFalse(simulator.has_context())

    def test_simulator_flags(self):
        # TODO(eric.cousineau): Move this to `analysis_test.py`.
        system = ConstantVectorSource([1])
        simulator = Simulator(system)

        ResetIntegratorFromFlags(simulator, "runge_kutta2", 0.00123)
        integrator = simulator.get_integrator()
        self.assertEqual(type(integrator), RungeKutta2Integrator)
        self.assertEqual(integrator.get_maximum_step_size(), 0.00123)

        self.assertGreater(len(GetIntegrationSchemes()), 5)

    def test_abstract_output_port_eval(self):
        model_value = Value("Hello World")
        source = ConstantValueSource(copy.copy(model_value))
        context = source.CreateDefaultContext()
        output_port = source.get_output_port(0)

        value = output_port.Eval(context)
        self.assertEqual(type(value), type(model_value.get_value()))
        self.assertEqual(value, model_value.get_value())

        value_abs = output_port.EvalAbstract(context)
        self.assertEqual(type(value_abs), type(model_value))
        self.assertEqual(value_abs.get_value(), model_value.get_value())

    def test_vector_output_port_eval(self):
        np_value = np.array([1., 2., 3.])
        model_value = Value(BasicVector(np_value))
        source = ConstantVectorSource(np_value)
        context = source.CreateDefaultContext()
        output_port = source.get_output_port(0)

        value = output_port.Eval(context)
        self.assertEqual(type(value), np.ndarray)
        np.testing.assert_equal(value, np_value)

        value_abs = output_port.EvalAbstract(context)
        self.assertEqual(type(value_abs), type(model_value))
        self.assertEqual(type(value_abs.get_value().get_value()), np.ndarray)
        np.testing.assert_equal(value_abs.get_value().get_value(), np_value)

        basic = output_port.EvalBasicVector(context)
        self.assertEqual(type(basic), BasicVector)
        self.assertEqual(type(basic.get_value()), np.ndarray)
        np.testing.assert_equal(basic.get_value(), np_value)

    def test_abstract_input_port_eval(self):
        model_value = Value("Hello World")
        system = PassThrough(copy.copy(model_value))
        context = system.CreateDefaultContext()
        fixed = system.get_input_port(0).FixValue(context,
                                                  copy.copy(model_value))
        self.assertIsInstance(fixed.GetMutableData(), AbstractValue)
        input_port = system.get_input_port(0)

        self.assertTrue(input_port.HasValue(context))
        value = input_port.Eval(context)
        self.assertEqual(type(value), type(model_value.get_value()))
        self.assertEqual(value, model_value.get_value())

        value_abs = input_port.EvalAbstract(context)
        self.assertEqual(type(value_abs), type(model_value))
        self.assertEqual(value_abs.get_value(), model_value.get_value())

    def test_vector_input_port_eval(self):
        np_value = np.array([1., 2., 3.])
        model_value = Value(BasicVector(np_value))
        system = PassThrough(len(np_value))
        context = system.CreateDefaultContext()
        system.get_input_port(0).FixValue(context, np_value)
        input_port = system.get_input_port(0)

        self.assertTrue(input_port.HasValue(context))
        value = input_port.Eval(context)
        self.assertEqual(type(value), np.ndarray)
        np.testing.assert_equal(value, np_value)

        value_abs = input_port.EvalAbstract(context)
        self.assertEqual(type(value_abs), type(model_value))
        self.assertEqual(type(value_abs.get_value().get_value()), np.ndarray)
        np.testing.assert_equal(value_abs.get_value().get_value(), np_value)

        basic = input_port.EvalBasicVector(context)
        self.assertEqual(type(basic), BasicVector)
        self.assertEqual(type(basic.get_value()), np.ndarray)
        np.testing.assert_equal(basic.get_value(), np_value)

    def test_abstract_input_port_fix_string(self):
        model_value = Value("")
        system = PassThrough(copy.copy(model_value))
        context = system.CreateDefaultContext()
        input_port = system.get_input_port(0)

        # Fix to a literal.
        input_port.FixValue(context, "Alpha")
        value = input_port.Eval(context)
        self.assertEqual(type(value), type(model_value.get_value()))
        self.assertEqual(value, "Alpha")

        # Fix to a type-erased string.
        input_port.FixValue(context, Value("Bravo"))
        value = input_port.Eval(context)
        self.assertEqual(type(value), type(model_value.get_value()))
        self.assertEqual(value, "Bravo")

        # Fix to a non-string.
        with self.assertRaises(RuntimeError):
            # A RuntimeError occurs when the Context detects that the
            # type-erased Value objects are incompatible.
            input_port.FixValue(context, Value(1))
        with self.assertRaises(TypeError):
            # A TypeError occurs when pybind Value.set_value cannot match any
            # overload for how to assign the argument into the erased storage.
            input_port.FixValue(context, 1)
        with self.assertRaises(TypeError):
            input_port.FixValue(context, np.array([2.]))

    def test_abstract_input_port_fix_object(self):
        # The port type is py::object, not any specific C++ type.
        model_value = Value(object())
        system = PassThrough(copy.copy(model_value))
        context = system.CreateDefaultContext()
        input_port = system.get_input_port(0)

        # Fix to a type-erased py::object.
        input_port.FixValue(context, Value(object()))

        # Fix to an int.
        input_port.FixValue(context, 1)
        value = input_port.Eval(context)
        self.assertEqual(type(value), int)
        self.assertEqual(value, 1)

        # Fixing to an explicitly-typed Value instantiation is an error ...
        with self.assertRaises(RuntimeError):
            input_port.FixValue(context, Value("string"))
        # ... but implicit typing works just fine.
        input_port.FixValue(context, "string")
        value = input_port.Eval(context)
        self.assertEqual(type(value), str)
        self.assertEqual(value, "string")

    @numpy_compare.check_all_types
    def test_vector_input_port_fix(self, T):
        np_zeros = np.array([0.])
        system = PassThrough_[T](len(np_zeros))
        context = system.CreateDefaultContext()
        input_port = system.get_input_port(0)

        # Fix to a scalar.
        input_port.FixValue(context, T(1.))
        value = input_port.Eval(context)
        self.assertEqual(type(value), np.ndarray)
        numpy_compare.assert_equal(value, np.array([T(1.)]))

        # Fix to an ndarray.
        input_port.FixValue(context, np.array([T(2.)]))
        value = input_port.Eval(context)
        self.assertEqual(type(value), np.ndarray)
        numpy_compare.assert_equal(value, np.array([T(2.)]))

        # Fix to a BasicVector.
        input_port.FixValue(context, BasicVector_[T]([3.]))
        value = input_port.Eval(context)
        self.assertEqual(type(value), np.ndarray)
        numpy_compare.assert_equal(value, np.array([T(3.)]))

        # Fix to a type-erased BasicVector.
        input_port.FixValue(context, Value(BasicVector_[T]([4.])))
        value = input_port.Eval(context)
        self.assertEqual(type(value), np.ndarray)
        numpy_compare.assert_equal(value, np.array([T(4.)]))

        # Fix to wrong-sized vector.
        with self.assertRaises(RuntimeError):
            input_port.FixValue(context, np.array([0., 1.]))
        with self.assertRaises(RuntimeError):
            input_port.FixValue(context, Value(BasicVector_[T]([0., 1.])))

        # Fix to a non-vector.
        with self.assertRaises(TypeError):
            # A TypeError occurs when pybind Value.set_value cannot match any
            # overload for how to assign the argument into the erased storage.
            input_port.FixValue(context, "string")
        with self.assertRaises(RuntimeError):
            # A RuntimeError occurs when the Context detects that the
            # type-erased Value objects are incompatible.
            input_port.FixValue(context, Value("string"))

    @numpy_compare.check_all_types
    def test_allocate_input_vector(self, T):
        system = PassThrough_[T](1)
        value = system.AllocateInputVector(system.get_input_port())
        self.assertIsInstance(value, BasicVector_[T])

    @numpy_compare.check_all_types
    def test_allocate_input_abstract(self, T):
        system = PassThrough_[T](Value("a"))
        value = system.AllocateInputAbstract(system.get_input_port())
        self.assertIsInstance(value, Value[str])

    @numpy_compare.check_all_types
    def test_constaints_api(self, T):
        system = PassThrough_[T](1)
        self.assertEqual(system.num_constraints(), 0)
        system._AddExternalConstraint(constraint=_ExternalSystemConstraint())
        self.assertEqual(system.num_constraints(), 1)
        cloned = system.Clone()
        self.assertEqual(cloned.num_constraints(), 1)

    def test_event_status(self):
        system = ZeroOrderHold(period_sec=0.1, vector_size=1)
        # Existence check.
        EventStatus.Severity.kDidNothing
        EventStatus.Severity.kSucceeded
        EventStatus.Severity.kReachedTermination
        EventStatus.Severity.kFailed

        self.assertIsInstance(EventStatus.DidNothing(), EventStatus)
        self.assertIsInstance(EventStatus.Succeeded(), EventStatus)
        status = EventStatus.ReachedTermination(system=system, message="done")
        # Check API.
        self.assertIsInstance(status, EventStatus)
        self.assertEqual(
            status.severity(), EventStatus.Severity.kReachedTermination)
        self.assertIs(status.system(), system)
        self.assertEqual(status.message(), "done")
        self.assertIsInstance(
            status.KeepMoreSevere(candidate=status), EventStatus)
        status = EventStatus.Failed(system=system, message="failed")
        self.assertIsInstance(status, EventStatus)

    def test_diagram_builder_remove(self):
        builder = DiagramBuilder()
        source = builder.AddSystem(ConstantVectorSource([0.0]))
        adder = builder.AddSystem(Adder(1, 1))
        builder.ExportOutput(source.get_output_port())
        builder.RemoveSystem(adder)  # N.B. Deletes 'adder'; don't use after!
        diagram = builder.Build()
        self.assertEqual(diagram.num_input_ports(), 0)
        self.assertEqual(diagram.num_output_ports(), 1)

    def test_diagram_fan_out(self):
        builder = DiagramBuilder()
        adder = builder.AddSystem(Adder(7, 1))
        adder.set_name("adder")
        builder.ExportOutput(adder.get_output_port())
        in0_index = builder.ExportInput(adder.get_input_port(0), "in0")
        in1_index = builder.ExportInput(adder.get_input_port(1), "in1")

        # Exercise ConnectInput overload bindings, with and without argument
        # names.
        builder.ConnectInput(in0_index, adder.get_input_port(2))
        builder.ConnectInput("in1", adder.get_input_port(3))
        builder.ConnectInput(diagram_port_name="in0",
                             input=adder.get_input_port(4))
        builder.ConnectInput(diagram_port_index=in1_index,
                             input=adder.get_input_port(5))
        builder.ConnectToSame(exemplar=adder.get_input_port(2),
                              dest=adder.get_input_port(6))
        diagram = builder.Build()

        # Check the desired input topology is in the graph.
        graph = diagram.GetGraphvizString()
        self.assertRegex(graph, ":0:e -> .*:u0:w")
        self.assertRegex(graph, ":1:e -> .*:u1:w")
        self.assertRegex(graph, ":0:e -> .*:u2:w")
        self.assertRegex(graph, ":1:e -> .*:u3:w")
        self.assertRegex(graph, ":0:e -> .*:u4:w")
        self.assertRegex(graph, ":1:e -> .*:u5:w")
        self.assertRegex(graph, ":0:e -> .*:u6:w")

        # Check that fragment struct is a subset of the whole graph.
        fragment_struct = diagram.GetGraphvizFragment()
        self.assertEqual(len(fragment_struct.input_ports), 2)
        self.assertEqual(len(fragment_struct.output_ports), 1)
        fragment_text = "".join(fragment_struct.fragments)
        self.assertGreater(len(fragment_text), 0)
        self.assertIn(fragment_text, graph)

        # Check that max_depth has an effect.
        small_graph = diagram.GetGraphvizString(max_depth=0)
        self.assertLess(len(small_graph), len(graph))

    def test_diagram_api(self):
        def make_diagram():
            builder = DiagramBuilder()
            self.assertTrue(builder.empty())
            self.assertFalse(builder.already_built())
            adder1 = builder.AddNamedSystem("adder1", Adder(2, 2))
            adder2 = builder.AddNamedSystem("adder2", Adder(1, 2))
            builder.Connect(adder1.get_output_port(), adder2.get_input_port())
            self.assertTrue(
                builder.IsConnectedOrExported(port=adder2.get_input_port()))
            builder.Connect(adder2.get_output_port(), adder1.get_input_port(0))
            builder.Disconnect(source=adder2.get_output_port(),
                               dest=adder1.get_input_port(0))
            builder.ExportInput(adder1.get_input_port(0), "in0")
            builder.ExportInput(adder1.get_input_port(1), "in1")
            builder.ExportOutput(adder2.get_output_port(), "out")
            self.assertEqual(builder.num_input_ports(), 2)
            self.assertEqual(builder.num_output_ports(), 1)
            self.assertTrue(builder.HasSubsystemNamed("adder1"))
            builder.GetSubsystemByName(name="adder1")
            builder.GetMutableSubsystemByName(name="adder2")
            self.assertEqual(len(builder.connection_map()), 1)
            diagram = builder.Build()
            return adder1, adder2, diagram

        adder1, adder2, diagram = make_diagram()
        connections = diagram.connection_map()
        self.assertIn((adder2, InputPortIndex(0)), connections)
        self.assertEqual(connections[(adder2, InputPortIndex(0))],
                         (adder1, OutputPortIndex(0)))
        del adder1, adder2, diagram  # To test keep-alive logic
        gc.collect()
        self.assertEqual(list(connections.keys())[0][0].get_name(), "adder2")

        adder1, adder2, diagram = make_diagram()
        in0_locators = diagram.GetInputPortLocators(
            port_index=InputPortIndex(0))
        in1_locators = diagram.GetInputPortLocators(
            port_index=InputPortIndex(1))
        self.assertEqual(in0_locators, [(adder1, InputPortIndex(0))])
        self.assertEqual(in1_locators, [(adder1, InputPortIndex(1))])
        del adder1, adder2, diagram  # To test keep-alive logic
        gc.collect()
        self.assertEqual(in0_locators[0][0].get_name(), "adder1")

        adder1, adder2, diagram = make_diagram()
        out_locators = diagram.get_output_port_locator(
            port_index=OutputPortIndex(0))
        self.assertEqual(out_locators, (adder2, OutputPortIndex(0)))
        del adder1, adder2, diagram  # To test keep-alive logic
        gc.collect()
        self.assertEqual(out_locators[0].get_name(), "adder2")

    def test_add_named_system(self):
        builder = DiagramBuilder()
        adder1 = builder.AddNamedSystem("adder1", Adder(2, 3))
        self.assertEqual(adder1.get_name(), "adder1")
        adder2 = builder.AddNamedSystem(name="adder2", system=Adder(5, 8))
        self.assertEqual(adder2.get_name(), "adder2")

    def test_module_constants(self):
        self.assertEqual(repr(kUseDefaultName), "kUseDefaultName")

    def test_system_visitor(self):
        builder = DiagramBuilder()
        builder.AddNamedSystem("adder1", Adder(2, 2))
        builder.AddNamedSystem("adder2", Adder(2, 2))
        system = builder.Build()
        system.set_name("diagram")

        visited_systems = []
        visited_diagrams = []

        class MyVisitor(SystemVisitor):
            def VisitSystem(self, system):
                visited_systems.append(system.get_name())

            def VisitDiagram(self, diagram):
                visited_diagrams.append(diagram.get_name())
                for sys in diagram.GetSystems():
                    sys.Accept(self)

        visitor = MyVisitor()
        system.Accept(v=visitor)
        self.assertEqual(visited_systems, ["adder1", "adder2"])
        self.assertEqual(visited_diagrams, ["diagram"])

    def test_ports_lifetime_hazard(self):
        # To ensure the test will fail if only one suitable annotation is
        # missing, we must only test one API per returned port reference
        # identity at a time.
        api_kinds = ["default", "index", "name"]

        def make_hazard_system(api_kind):
            model_value = Value("Hello World")
            dut = PassThrough(copy.copy(model_value))

            # Try to provoke a lifetime hazard like #22515.
            ports = []
            if api_kind == "default":
                ports.append(dut.get_input_port())
                ports.append(dut.get_output_port())
            elif api_kind == "index":
                ports.append(dut.get_input_port(0))
                ports.append(dut.get_output_port(0))
            else:
                assert api_kind == "name"
                ports.append(dut.GetInputPort("u"))
                ports.append(dut.GetOutputPort("y"))
            return dut, ports

        for api_kind in api_kinds:
            dut, ports = make_hazard_system(api_kind)
            dut = [dut]  # The helper function requires passing dut via a list.
            framework_test_util.check_ports_lifetime_hazard(self, dut, ports)

    def test_system_builder_add_system(self):
        # This test ensures that `DiagramBuilder.AddSystem` and
        # `DiagramBuilder.AddNamedSystem` are bound with the correct return
        # value policy, otherwise it may lead to premature deletion of the
        # system. See issue #22880 for details.
        builder = DiagramBuilder()

        # `system1` has type `pydrake.systems.test.test_util.DummySystemA`.
        system1 = MakeDummySystem()
        # The return value of `builder.AddSystem(system1)` has type `pydrake.
        # systems.framework.System`, and its reference count is zero.
        builder.AddSystem(system1)
        # The c++ code for `AddSystem` returns a raw pointer. If pybind11
        # treats it as `take_ownership`, and since the reference count is
        # zero, pybind11 calls delete on the raw pointer, leading to a
        # use-after-free hazard.

        system2 = MakeDummySystem()
        builder.AddNamedSystem("system2", system2)

        builder.Build()
