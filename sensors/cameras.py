import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.kit import SimulationApp
from pxr import UsdGeom
from pxr.Usd import Stage

# from src.config import Config
from configs.main_config import MainConfig
import os
from omni.isaac.core_nodes.scripts.utils import set_target_prims

def setup_cameras(cfg: MainConfig, simulation_app: SimulationApp, stage: Stage, cameras: list = None):

    graph_controller = setup_cameras_graph(cfg, simulation_app)
    for camera_name, camera in cameras.items():
        if camera_name == "Camera_3" or camera_name == "Camera_1": # or camera_name == "Camera_1"
            setup_camera_graph(cfg, simulation_app, stage, graph_controller, camera_name, camera)


def setup_cameras_graph(cfg: MainConfig, simulation_app: SimulationApp) -> og.Controller:
    keys = og.Controller.Keys
    controller = og.Controller(graph_id="ros_cameras_graph")

    (graph, _, _, _) = controller.edit(
        {
            "graph_path": cfg.camera_action_graph_stage_path,
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("read_sys_time", "omni.isaac.core_nodes.IsaacReadSystemTime"),
            ]
        },
    )

    simulation_app.update()
    return controller


def setup_camera_graph(
    cfg: MainConfig, simulation_app: SimulationApp, stage: Stage, controller: og.Controller, camera_name: str, camera: str
):
    """Setup the action graph for publishing Images, Depths and CameraInfo to ROS1/2"""

    keys = og.Controller.Keys
    graph = og.get_graph_by_path(cfg.camera_action_graph_stage_path)

    ros_bridge = "omni.isaac.ros_bridge".split("-")[0]
    ros_v = 1

    camera_prim = camera.prim_path
 
    # camera_index = 3  
    camera_index = int(camera_name[-1])

    controller.edit(
        graph,
        {
            keys.CREATE_NODES: [
                (f"create_{camera_name}Viewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                (f"get_{camera_name}RenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                (f"set_{camera_name}Camera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                (f"camera_{camera_name}HelperRgb", f"{ros_bridge}.ROS{ros_v}CameraHelper"),
                (f"camera_{camera_name}HelperInfo", f"{ros_bridge}.ROS{ros_v}CameraHelper"),
                (f"camera_{camera_name}HelperDepth", f"{ros_bridge}.ROS{ros_v}CameraHelper"),
                (f"camera_{camera_name}HelperDepthPCL", f"{ros_bridge}.ROS{ros_v}CameraHelper"),
                (f"set_{camera_name}Resolution", "omni.isaac.core_nodes.IsaacSetViewportResolution"),
                (f"PublishTF_{camera_name}", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", f"create_{camera_name}Viewport.inputs:execIn"),
                (
                    f"create_{camera_name}Viewport.outputs:execOut",
                    f"set_{camera_name}Resolution.inputs:execIn",
                ),
                (
                    f"create_{camera_name}Viewport.outputs:viewport",
                    f"get_{camera_name}RenderProduct.inputs:viewport",
                ),
                (
                    f"create_{camera_name}Viewport.outputs:viewport",
                    f"set_{camera_name}Resolution.inputs:viewport",
                ),
                (
                    f"set_{camera_name}Resolution.outputs:execOut",
                    f"get_{camera_name}RenderProduct.inputs:execIn",
                ),

                (f"get_{camera_name}RenderProduct.outputs:execOut", f"set_{camera_name}Camera.inputs:execIn"),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"set_{camera_name}Camera.inputs:renderProductPath",
                ),
                (f"set_{camera_name}Camera.outputs:execOut", f"camera_{camera_name}HelperRgb.inputs:execIn"),
                (f"set_{camera_name}Camera.outputs:execOut", f"camera_{camera_name}HelperInfo.inputs:execIn"),
                (
                    f"set_{camera_name}Camera.outputs:execOut",
                    f"camera_{camera_name}HelperDepth.inputs:execIn",
                ),
                (
                    f"set_{camera_name}Camera.outputs:execOut",
                    f"camera_{camera_name}HelperDepthPCL.inputs:execIn",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"camera_{camera_name}HelperRgb.inputs:renderProductPath",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"camera_{camera_name}HelperInfo.inputs:renderProductPath",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"camera_{camera_name}HelperDepth.inputs:renderProductPath",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"camera_{camera_name}HelperDepthPCL.inputs:renderProductPath",
                ),
                ("OnTick.outputs:tick", f"PublishTF_{camera_name}"+".inputs:execIn"),
                ("read_sys_time.outputs:systemTime", f"PublishTF_{camera_name}"+".inputs:timeStamp")
            ],
            keys.SET_VALUES: [
                (
                    f"create_{camera_name}Viewport.inputs:viewportId",
                    camera_index,
                ),

                (f"camera_{camera_name}HelperRgb.inputs:frameId", f"{camera_name}"),
                (f"camera_{camera_name}HelperRgb.inputs:topicName", f"/sensor/camera/camera_{camera_index}_isaac/color/image_raw"),
                (f"camera_{camera_name}HelperRgb.inputs:type", "rgb"),
                (f"camera_{camera_name}HelperRgb.inputs:useSystemTime", True),

                (f"camera_{camera_name}HelperInfo.inputs:frameId", f"{camera_name}"),
                (f"camera_{camera_name}HelperInfo.inputs:topicName", f"/sensor/camera/camera_{camera_index}/camera_info"),
                (f"camera_{camera_name}HelperInfo.inputs:type", "camera_info"),
                (f"set_{camera_name}Resolution.inputs:height", 480),
                (f"set_{camera_name}Resolution.inputs:width", 640),
                (f"camera_{camera_name}HelperInfo.inputs:useSystemTime", True),

                (f"camera_{camera_name}HelperDepthPCL.inputs:frameId", f"{camera_name}"),
                (f"camera_{camera_name}HelperDepthPCL.inputs:topicName", f"{camera_name}_depth_pcl"),
                (f"camera_{camera_name}HelperDepthPCL.inputs:type", "depth_pcl"),
                (f"camera_{camera_name}HelperDepthPCL.inputs:useSystemTime", True),

                # (f"camera_{camera_name}HelperDepth.inputs:type", "depth"),   # 重点! depth的topic类型!!!!
                (f"camera_{camera_name}HelperDepth.inputs:frameId", f"{camera_name}"),
                (f"camera_{camera_name}HelperDepth.inputs:topicName", f"{camera_name}_depth"),
                (f"camera_{camera_name}HelperDepth.inputs:type", "depth"),
                (f"camera_{camera_name}HelperDepth.inputs:useSystemTime", True),
                (f"PublishTF_{camera_name}"+".inputs:topicName", f"/tf_camera_{camera_index}"),

            ],
        },
    )

    set_targets(
        prim=stage.GetPrimAtPath(cfg.camera_action_graph_stage_path + f"/set_{camera_name}Camera"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[camera_prim],
    )
# /World/aloha/fl_link6/visuals/cam03_root/cam03_parent/Camera_3
# camera_index
# f"/World/aloha/fl_link6/visuals/cam03_root/cam03_parent/Camera_3"
# /World/aloha/box2_Link/visuals/cam01_root/cam01_parent/Camera_1
    if camera_index == 3:
        set_target_prims(
            primPath="/World/aloha/CAMERA/PublishTF_Camera_3",
            inputName="inputs:targetPrims",
            targetPrimPaths=[f"/World/aloha/fl_link6/visuals/cam03_root/cam03_parent/Camera_3"],
        )
        set_target_prims(
            primPath="/World/aloha/CAMERA/PublishTF_Camera_3",
            inputName="inputs:parentPrim",
            targetPrimPaths=["/World"],
        )
    elif camera_index == 1:
        set_target_prims(
            primPath="/World/aloha/CAMERA/PublishTF_Camera_1",
            inputName="inputs:targetPrims",
            targetPrimPaths=[f"/World/aloha/box2_Link/visuals/cam01_root/cam01_parent/Camera_1"],
        )
        set_target_prims(
            primPath="/World/aloha/CAMERA/PublishTF_Camera_1",
            inputName="inputs:parentPrim",
            targetPrimPaths=["/World"],
        )

    simulation_app.update()

    controller.evaluate_sync(graph)
