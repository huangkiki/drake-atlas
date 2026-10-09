"""Read native modeling and Context APIs against Drake 1.57.0.

Status: source reviewed and syntax checked; NOT executed by this course delivery.
This original asset has no external files. There is no Simulator or time stepping.
If run later, this prints model/state observations, not validated physical results.
"""

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Parser,
    RigidTransform,
    SpatialVelocity,
)

# Uniform box: mass 2 kg, full lengths 0.2 / 0.4 / 0.6 m.
# Body, inertial, visual and collision origins coincide at the center of mass.
# Ixx = m * (ly**2 + lz**2) / 12; analogous formulas give Iyy and Izz.
BOX_URDF = """\
<robot name="e1_box">
  <link name="box">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="0.08666666666666667" ixy="0" ixz="0"
               iyy="0.06666666666666667" iyz="0" izz="0.03333333333333333"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.2 0.4 0.6"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.2 0.4 0.6"/></geometry>
    </collision>
  </link>
</robot>
"""


def main():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    parser = Parser(plant)
    parser.SetStrictParsing()
    (model_instance,) = parser.AddModelsFromString(BOX_URDF, "urdf")
    # No weld is added: Finalize's default base joint is quaternion floating.
    plant.Finalize()
    diagram = builder.Build()
    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    body = plant.GetUniqueFloatingBaseBodyOrThrow(model_instance)

    # This API explicitly means the floating base body's pose in world.
    plant.SetFloatingBaseBodyPoseInWorldFrame(
        plant_context, body, RigidTransform([0.0, 0.0, 0.5])
    )
    # This body's default floating base frames are world and the body frame.
    # SpatialVelocity stores angular velocity first, then translational velocity.
    plant.SetFreeBodySpatialVelocity(
        plant_context, body, SpatialVelocity(w=[0.0, 0.0, 2.0], v=[0.0, 0.0, 0.0])
    )
    saved_q = plant.GetPositions(plant_context).copy()
    saved_v = plant.GetVelocities(plant_context).copy()
    snapshot = root_context.Clone()

    # Edit the original Context; the snapshot has its own Context state.
    plant.SetFloatingBaseBodyPoseInWorldFrame(
        plant_context, body, RigidTransform([0.0, 0.0, 0.8])
    )
    snapshot_plant = plant.GetMyContextFromRoot(snapshot)
    print("nq, nv:", plant.num_positions(), plant.num_velocities())
    print("saved q (wxyz, xyz):", saved_q)
    print("saved v (angular, linear):", saved_v)
    print("snapshot q:", plant.GetPositions(snapshot_plant).copy())
    print("edited q:", plant.GetPositions(plant_context).copy())

    # This edits time; it does not integrate motion. Time belongs to the root.
    root_context.SetTime(0.25)
    # In this version SetDefaultContext resets parameters/state, not time/inputs.
    diagram.SetDefaultContext(root_context)
    print("time after default state/parameter reset:", root_context.get_time())
    print(
        "q after default state/parameter reset:",
        plant.GetPositions(plant_context).copy(),
    )
    # No contact output is interpreted here; sampled dynamics has not been run.
    # Keep diagram, plant, scene_graph and their contexts alive for these queries.


if __name__ == "__main__":
    main()
