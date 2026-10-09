"""Original two-link arm: native model, FK/Jacobian and an unsolved IK problem.

Source reviewed against Drake 1.57.0; syntax checked, never executed for E2.
There is no Simulator, AdvanceTo or Solve. No collision geometries are provided.
"""

import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    InverseKinematics,
    JacobianWrtVariable,
    Parser,
)

# Two uniform 1 x 0.1 x 0.1 m links, 1 kg each. Centers of mass are at x=0.5.
# Inertia tensors below are central inertias in link-aligned inertial frames.
ARM_URDF = """\
<robot name="e2_arm">
  <link name="base"/>
  <link name="link1">
    <inertial>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.0016666666666666668" ixy="0" ixz="0"
               iyy="0.08416666666666667" iyz="0" izz="0.08416666666666667"/>
    </inertial>
  </link>
  <link name="link2">
    <inertial>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.0016666666666666668" ixy="0" ixz="0"
               iyy="0.08416666666666667" iyz="0" izz="0.08416666666666667"/>
    </inertial>
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-2.8" upper="2.8" effort="20" velocity="2"/>
  </joint>
  <joint name="elbow" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-2.8" upper="2.8" effort="20" velocity="2"/>
  </joint>
</robot>
"""


def make_arm_builder(time_step):
    """Build this specific teaching asset; leave Diagram wiring to the caller."""
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    parser = Parser(plant)
    parser.SetStrictParsing()
    (arm,) = parser.AddModelsFromString(ARM_URDF, "urdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", arm))
    # URDF contains no transmission; actuators are added explicitly here.
    for name in ("shoulder", "elbow"):
        plant.AddJointActuator(
            name + "_motor", plant.GetJointByName(name, arm), 20.0
        )
    plant.Finalize()
    return builder, plant, arm


def main():
    builder, plant, arm = make_arm_builder(time_step=0.001)
    diagram = builder.Build()
    root = diagram.CreateDefaultContext()
    context = plant.GetMyMutableContextFromRoot(root)
    plant.GetJointByName("shoulder", arm).set_angle(context, np.pi / 4)
    plant.GetJointByName("elbow", arm).set_angle(context, -np.pi / 2)
    link2 = plant.GetFrameByName("link2", arm)
    world = plant.world_frame()
    p_BoT_B = np.array([1.0, 0.0, 0.0])
    p_WT = plant.CalcPointsPositions(context, link2, p_BoT_B.reshape(3, 1), world)
    J_WT = plant.CalcJacobianSpatialVelocity(
        context, JacobianWrtVariable.kV, link2, p_BoT_B, world, world
    )
    print("TCP position [m]:", p_WT)
    print("Spatial Jacobian (angular rows first):", J_WT)

    # IK can mutate its supplied Context. Keep a separate ROOT clone alive.
    planning_root = root.Clone()
    planning_context = plant.GetMyMutableContextFromRoot(planning_root)
    ik = InverseKinematics(plant, planning_context, with_joint_limits=True)
    goal = np.array([1.2, 0.5, 0.0])
    tolerance = 1e-4  # Teaching position box [m]; not experimental acceptance.
    ik.AddPositionConstraint(
        link2, p_BoT_B, world, goal - tolerance, goal + tolerance
    )
    seed = plant.GetPositions(planning_context).copy()
    ik.prog().AddQuadraticErrorCost(np.eye(plant.num_positions()), seed, ik.q())
    ik.prog().SetInitialGuess(ik.q(), seed)
    # Deliberately no Solve: no feasibility, collision or execution claim.
    print("IK decision variable count:", len(ik.q()))


if __name__ == "__main__":
    main()
