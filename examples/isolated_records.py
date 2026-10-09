"""Native E5 reading example; AST/source reviewed, never imported or executed.

Two independent contexts share a read-only Diagram with a RandomSource and a
periodic-only VectorLogSink. No physics model, renderer, learning wrapper or
native stepping is performed by this module. The functions are not called here.
"""

from pydrake.common import RandomDistribution, RandomGenerator
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, TriggerType
from pydrake.systems.primitives import LogVectorOutput, RandomSource


def make_independent_records():
    """Describe two independently seeded, uninitialized native simulators.

    The source emits two dimensionless standard Gaussian values, held for
    10 ms. The logger requests periodic-only samples every 20 ms. Construction
    produces no log samples; a future authorized caller would
    initialize and advance events. Initial state values can be read before
    that, but an initial draw is not an executed periodic sampling event.
    """
    builder = DiagramBuilder()
    source = builder.AddSystem(
        RandomSource(RandomDistribution.kGaussian, 2, 0.01)
    )
    logger = LogVectorOutput(
        source.get_output_port(), builder, {TriggerType.kPeriodic}, 0.02
    )
    builder.ExportOutput(source.get_output_port(), "dimensionless_draws")
    diagram = builder.Build()

    simulators = []
    for seed in (11, 22):
        context = diagram.CreateDefaultContext()
        context.SetTime(0.0)
        diagram.SetRandomContext(context, RandomGenerator(seed))
        simulator = Simulator(diagram, context)
        simulator.set_target_realtime_rate(0.0)
        simulators.append(simulator)
    return diagram, logger, tuple(simulators)


def replace_with_fresh_episode(diagram, simulator, seed):
    """Replace with a fresh matching Context; leave Initialize to the caller.

    This specific Diagram has no exported input to re-fix and no physics or
    external resources to reset. A general robot task needs those additional
    contracts. A fresh logger cache starts empty; cloning old history would not.
    """
    context = diagram.CreateDefaultContext()
    context.SetTime(0.0)
    diagram.SetRandomContext(context, RandomGenerator(seed))
    simulator.reset_context(context)


def copy_episode_log(logger, simulator):
    """Return independent host arrays: time[M] and dimensionless values[2,M]."""
    log = logger.FindLog(simulator.get_context())
    return log.sample_times().copy(), log.data().copy()
