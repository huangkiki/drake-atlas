"""Read-only teaching example: a depth camera with explicit sample-and-hold.

Source-reviewed against Drake v1.57.0; AST checked, never imported or run here.
Calling build_sampled_camera would create a native renderer and Diagram. It does
not advance time, initialize events, render an image, or launch Meshcat. A later
authorized run must process sampling events before treating outputs as captured
frames. Initial ZOH values are placeholders, including the zero time vector.
"""

from pydrake.common.value import AbstractValue
from pydrake.geometry import (
    ClippingRange,
    DepthRange,
    DepthRenderCamera,
    GeometryInstance,
    MakeRenderEngineVtk,
    PerceptionProperties,
    RenderCameraCore,
    RenderEngineVtkParams,
    RenderLabel,
    Rgba,
    SceneGraph,
    Sphere,
)
from pydrake.math import RigidTransform
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ZeroOrderHold
from pydrake.systems.sensors import CameraInfo, ImageDepth32F, RgbdSensor


def build_sampled_camera():
    """Return a Diagram; no Context, Simulator or rendering Eval is performed."""
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())
    scene_graph.set_name("scene_graph")

    # Perception geometry only: an original anchored sphere, with no physics.
    source_id = scene_graph.RegisterSource("camera_lesson")
    sphere = GeometryInstance(
        RigidTransform([0.0, 0.0, 2.0]), Sphere(0.2), "target_sphere"
    )
    perception = PerceptionProperties()
    perception.AddProperty("phong", "diffuse", Rgba(0.2, 0.6, 0.8, 1.0))
    perception.AddProperty("label", "id", RenderLabel(7))
    sphere.set_perception_properties(perception)
    scene_graph.RegisterAnchoredGeometry(source_id, sphere)

    renderer_name = "lesson_renderer"
    # No backend availability or image fidelity was tested in this phase.
    scene_graph.AddRenderer(
        renderer_name, MakeRenderEngineVtk(RenderEngineVtkParams())
    )
    width, height = 64, 48
    intrinsics = CameraInfo(width, height, 48.0, 48.0, 31.5, 23.5)
    core = RenderCameraCore(
        renderer_name, intrinsics, ClippingRange(0.01, 10.0), RigidTransform()
    )
    depth_camera = DepthRenderCamera(core, DepthRange(0.1, 5.0))
    camera = builder.AddSystem(
        RgbdSensor(
            SceneGraph.world_frame_id(), RigidTransform(), depth_camera, False
        )
    )
    builder.Connect(
        scene_graph.get_query_output_port(), camera.query_object_input_port()
    )

    # Hold only the needed image, with explicit matching pose and timestamp.
    # This deliberately avoids the fixed-version RgbdSensorDiscrete image_time
    # export discrepancy; see docs/sensor-timing.md for the source evidence.
    period, offset = 0.05, 0.01
    depth_hold = builder.AddSystem(
        ZeroOrderHold(
            period, AbstractValue.Make(ImageDepth32F(width, height)), offset
        )
    )
    pose_hold = builder.AddSystem(
        ZeroOrderHold(period, AbstractValue.Make(RigidTransform()), offset)
    )
    time_hold = builder.AddSystem(ZeroOrderHold(period, 1, offset))
    builder.Connect(
        camera.depth_image_32F_output_port(), depth_hold.get_input_port()
    )
    builder.Connect(
        camera.body_pose_in_world_output_port(), pose_hold.get_input_port()
    )
    builder.Connect(camera.image_time_output_port(), time_hold.get_input_port())
    builder.ExportOutput(depth_hold.get_output_port(), "depth_m")
    builder.ExportOutput(pose_hold.get_output_port(), "X_WB_at_capture")
    builder.ExportOutput(time_hold.get_output_port(), "capture_time_s")
    return builder.Build()
