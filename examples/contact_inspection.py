"""Original sphere/ground contact configuration and field reader for Drake 1.57.0.

Source and syntax reviewed only. This file was never executed for E3.
No Simulator, forced event, collision query or solver is invoked by main.
The reader is separate and is not called by main; its Eval would read the
configured output, whose sampled/live contract must be understood first.
"""

from pydrake.geometry import (
    AddCompliantHydroelasticProperties,
    AddContactMaterial,
    AddRigidHydroelasticProperties,
    HalfSpace,
    ProximityProperties,
    Sphere,
)
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    ContactModel,
    CoulombFriction,
    DiscreteContactApproximation,
)
from pydrake.systems.framework import DiagramBuilder

# Uniform solid sphere: m=1 kg, r=0.05 m; Ixx=Iyy=Izz=2*m*r**2/5.
# Geometry is deliberately registered through native APIs below, not the URDF.
BALL_URDF = """\
<robot name="e3_ball">
  <link name="ball">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
</robot>
"""


def make_contact_diagram():
    """Configure this specific teaching model, without stepping or querying it."""
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    plant.set_contact_model(ContactModel.kHydroelasticWithFallback)
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kLagged)
    plant.set_stiction_tolerance(1e-4)  # m/s; not a Newton tolerance.
    plant.SetUseSampledOutputPorts(True)
    parser = Parser(plant)
    parser.SetStrictParsing()
    (instance,) = parser.AddModelsFromString(BALL_URDF, "urdf")
    ball = plant.GetBodyByName("ball", instance)

    ball_properties = ProximityProperties()
    AddContactMaterial(
        properties=ball_properties, dissipation=20.0, point_stiffness=2e5,
        friction=CoulombFriction(static_friction=0.8, dynamic_friction=0.6),
    )
    AddCompliantHydroelasticProperties(
        resolution_hint=0.02, hydroelastic_modulus=1e6,
        properties=ball_properties,
    )
    # Seconds, used by kSap approximation; ignored by this kLagged choice.
    ball_properties.AddProperty("material", "relaxation_time", 0.01)
    ball_geometry = plant.RegisterCollisionGeometry(
        ball, RigidTransform(), Sphere(0.05), "ball_contact", ball_properties
    )

    ground_properties = ProximityProperties()
    AddContactMaterial(
        properties=ground_properties, dissipation=10.0, point_stiffness=2e5,
        friction=CoulombFriction(static_friction=0.7, dynamic_friction=0.5),
    )
    AddRigidHydroelasticProperties(properties=ground_properties)
    ground_properties.AddProperty("material", "relaxation_time", 0.02)
    ground_geometry = plant.RegisterCollisionGeometry(
        plant.world_body(), RigidTransform(), HalfSpace(),
        "ground_contact", ground_properties,
    )
    plant.Finalize()
    diagram = builder.Build()
    root = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root)
    plant.SetFloatingBaseBodyPoseInWorldFrame(
        plant_context, ball, RigidTransform([0.0, 0.0, 0.06])
    )
    return diagram, root, plant, ball_geometry, ground_geometry


def read_contact_fields(plant, plant_context):
    """Read native fields without conflating A/B sides or inventing sample time.

    These records are explanatory data, not a grasp score. In sampled mode an
    initial empty list does not prove geometry separation. The function is not
    called by this example's main and was not executed during E3.
    """
    results = plant.get_contact_results_output_port().Eval(plant_context)
    inspector = plant.EvalSceneGraphInspector(plant_context)
    points = []
    for i in range(results.num_point_pair_contacts()):
        info = results.point_pair_contact_info(i)
        pair = info.point_pair()
        force_B_W = info.contact_force().copy()
        normal_BA_W = pair.nhat_BA_W.copy()
        points.append({
            "body_A": info.bodyA_index(), "body_B": info.bodyB_index(),
            "geometry_A": pair.id_A, "geometry_B": pair.id_B,
            "point_W_m": info.contact_point().copy(),
            "force_on_B_W_N": force_B_W,
            "normal_BA_W": normal_BA_W,
            "signed_compression_N": -normal_BA_W @ force_B_W,
            "slip_speed_mps": info.slip_speed(),
            "separation_speed_mps": info.separation_speed(),
        })
    surfaces = []
    for i in range(results.num_hydroelastic_contacts()):
        info = results.hydroelastic_contact_info(i)
        surface = info.contact_surface()
        body_A = plant.GetBodyFromFrameId(inspector.GetFrameId(surface.id_M()))
        body_B = plant.GetBodyFromFrameId(inspector.GetFrameId(surface.id_N()))
        surfaces.append({
            "body_A": body_A.index(), "body_B": body_B.index(),
            "geometry_M": surface.id_M(), "geometry_N": surface.id_N(),
            "centroid_W_m": surface.centroid().copy(),
            # Rotational [N m] first, translational [N] last. Force is on A.
            "wrench_on_A_at_centroid_W": info.F_Ac_W().get_coeffs().copy(),
        })
    return {
        "query_time_s": plant_context.get_time(),  # Not a physical sample time.
        "point_contacts": points,
        "hydroelastic_contacts": surfaces,
    }


if __name__ == "__main__":
    diagram, root, plant, ball_id, ground_id = make_contact_diagram()
    print("Configured native contact model; no physics update or contact readback.")
