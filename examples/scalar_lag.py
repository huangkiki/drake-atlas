"""Original scalar-convertible native System; source/AST only, never imported.

Input u, state x and output y are meters; fixed tau is seconds. No Simulator,
step, renderer, optimizer or automatic instance construction is included.
Importing would still register the TemplateSystem decorator; this phase did not.
"""

import math

from pydrake.autodiffutils import (
    ExtractGradient,
    ExtractValue,
    InitializeAutoDiff,
)
from pydrake.systems.framework import LeafSystem_
from pydrake.systems.scalar_conversion import TemplateSystem


@TemplateSystem.define("FirstOrderLag_")
def FirstOrderLag_(T):
    class Impl(LeafSystem_[T]):
        def _construct(self, tau=0.05, converter=None):
            LeafSystem_[T].__init__(self, converter=converter)
            if not math.isfinite(tau) or tau <= 0:
                raise ValueError("tau must be finite, positive seconds")
            self._tau = tau  # Fixed configuration, not an AD parameter.
            self._input = self.DeclareVectorInputPort("u_m", 1)
            self.DeclareContinuousState(1)
            self.DeclareVectorOutputPort(
                "y_m", 1, self._output,
                prerequisites_of_calc={self.all_state_ticket()},
            )

        def _construct_copy(self, other, converter=None):
            Impl._construct(self, other._tau, converter=converter)

        def _output(self, context, output):
            output.SetAtIndex(
                0, context.get_continuous_state_vector().GetAtIndex(0)
            )

        def DoCalcTimeDerivatives(self, context, derivatives):
            x = context.get_continuous_state_vector().GetAtIndex(0)
            u = self._input.Eval(context)[0]
            derivatives.get_mutable_vector().SetAtIndex(0, (u - x) / self._tau)

    return Impl


FirstOrderLag = FirstOrderLag_[None]


def inspect_local_state_derivative():
    """Unexecuted reading function: xdot and its local derivative with respect to x.

    The analytic prediction is xdot=6 m/s and d(xdot)/dx=-20 1/s. These are
    mathematical predictions, not recorded outputs from a native evaluation.
    Neither u nor tau is seeded as an independent variable in this example.
    """
    system = FirstOrderLag(tau=0.05).ToAutoDiffXd()
    context = system.CreateDefaultContext()
    context.SetContinuousState(InitializeAutoDiff([0.2]))
    system.get_input_port().FixValue(context, [0.5])
    derivative = system.EvalTimeDerivatives(context).CopyToVector()
    return ExtractValue(derivative), ExtractGradient(derivative)
