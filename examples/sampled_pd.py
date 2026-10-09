"""Native PID -> saturation -> hold -> actuation wiring, not executed for E2.

The gains and periods are teaching choices, not tuned or validated settings.
No Simulator is created and no control loop runs. Ki is zero; this does not
redefine PidController as a discrete-time controller.
"""

from pydrake.all import ConstantVectorSource, PidController, Saturation, ZeroOrderHold

from arm_kinematics import make_arm_builder


def main():
    builder, plant, arm = make_arm_builder(time_step=0.001)
    # Direct wiring below is valid only for this asset and this explicit mapping.
    # Other models need the actual state/actuator projection, not these assumptions.
    for index, name in enumerate(("shoulder", "elbow")):
        joint = plant.GetJointByName(name, arm)
        actuator = plant.GetJointActuatorByName(name + "_motor", arm)
        mapping = (
            joint.position_start(), joint.velocity_start(), actuator.input_start()
        )
        if mapping != (index, index, index):
            raise ValueError("This teaching wiring requires matching q, v and u order")

    reference = builder.AddSystem(ConstantVectorSource([0.4, -0.6, 0.0, 0.0]))
    controller = builder.AddSystem(
        PidController(kp=[20.0, 20.0], ki=[0.0, 0.0], kd=[4.0, 4.0])
    )
    # Command budget is tighter than this asset's 20 N m actuator limits.
    saturation = builder.AddSystem(
        Saturation(min_value=[-5.0, -5.0], max_value=[5.0, 5.0])
    )
    hold = builder.AddSystem(ZeroOrderHold(period_sec=0.005, vector_size=2))
    builder.Connect(
        plant.get_state_output_port(arm), controller.get_input_port_estimated_state()
    )
    builder.Connect(
        reference.get_output_port(), controller.get_input_port_desired_state()
    )
    builder.Connect(controller.get_output_port_control(), saturation.get_input_port())
    builder.Connect(saturation.get_output_port(), hold.get_input_port())
    builder.Connect(hold.get_output_port(), plant.get_actuation_input_port(arm))
    diagram = builder.Build()
    root = diagram.CreateDefaultContext()
    hold_context = hold.GetMyMutableContextFromRoot(root)
    hold.SetVectorState(hold_context, [0.0, 0.0])
    # The initial hold state is zero. Wiring alone has not sampled the PID output.
    print("Native control diagram constructed; no updates or simulation performed.")


if __name__ == "__main__":
    main()
