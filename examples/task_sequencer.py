"""Original native LeafSystem task-state example; never executed for E2.

Input: [approach_error_m, aperture_m, normal_force_N, slip_speed_mps,
        sample_time_s, abort_0_or_1]. Output: [width_target_m, phase_code].
All thresholds are teaching placeholders, not a grasp score or safety policy.
A Fault preserves the last reference; a real application must define fallback.
"""

from enum import IntEnum

import numpy as np
from pydrake.all import EventStatus, LeafSystem


class Phase(IntEnum):
    APPROACH = 0
    CLOSE = 1
    HOLD = 2
    RELEASE = 3
    DONE = 4
    FAULT = 5


class TaskSequencer(LeafSystem):
    """Keep task progress in Context State, not mutable Python instance data."""

    def __init__(self):
        super().__init__()
        self._observation = self.DeclareVectorInputPort("observation", 6)
        # phase, phase-entry time, stable-since time, width reference.
        # Negative time sentinels are initialized at the first actual update.
        self.DeclareDiscreteState([Phase.APPROACH, -1.0, -1.0, 0.08])
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=0.01, offset_sec=0.0, update=self._update
        )
        self.DeclareVectorOutputPort(
            "command", 2, self._output,
            prerequisites_of_calc={self.all_state_ticket()},
        )

    def _update(self, context, discrete_state):
        state = context.get_discrete_state_vector().CopyToVector()
        phase = Phase(int(state[0]))
        entered, stable_since, width = state[1:]
        if phase in (Phase.DONE, Phase.FAULT):
            discrete_state.set_value(state)
            return EventStatus.Succeeded()

        now = context.get_time()
        if entered < 0:
            entered = now
        obs = self._observation.Eval(context)
        error, aperture, force, slip, sample_time, abort = obs
        valid = (
            np.isfinite(obs).all()
            and error >= 0 and aperture >= 0 and force >= 0
            and abort in (0, 1)
            and 0 <= now - sample_time <= 0.02
        )
        if not valid or abort == 1 or force > 20.0 or now - entered > 3.0:
            discrete_state.set_value([Phase.FAULT, now, -1.0, width])
            # Framework update succeeded; task outcome is Fault, not success.
            return EventStatus.Succeeded()

        next_phase = phase
        if phase == Phase.APPROACH and error <= 0.005:
            next_phase, width = Phase.CLOSE, 0.03
        elif phase == Phase.CLOSE and force >= 2.0 and aperture > 0.001:
            next_phase = Phase.HOLD
        elif phase == Phase.HOLD:
            if force >= 2.0 and abs(slip) <= 0.001:
                if stable_since < 0:
                    stable_since = now
                elif now - stable_since >= 1.0:
                    next_phase, width = Phase.RELEASE, 0.08
            else:
                stable_since = -1.0
        elif phase == Phase.RELEASE and aperture >= 0.07 and force <= 0.1:
            next_phase = Phase.DONE

        if next_phase != phase:
            entered, stable_since = now, -1.0
        discrete_state.set_value([next_phase, entered, stable_since, width])
        return EventStatus.Succeeded()

    def _output(self, context, output):
        state = context.get_discrete_state_vector().get_value()
        output.SetFromVector([state[3], state[0]])
