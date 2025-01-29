import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.kit import SimulationApp
from pxr.Usd import Stage

# from src.config import Config
from configs.main_config import MainConfig


def setup_lidar_graph(cfg: MainConfig, simulation_app: SimulationApp, stage: Stage):
    """Setup the LiDAR action graph for publishing the point cloud to ROS1/2"""

    ros_bridge = "omni.isaac.ros_bridge".split("-")[0]
    ros_v = 1

    try:
        og.Controller.edit(
            {"graph_path": cfg.lidar.action_graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("createLiRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("lidarHelperMsg", f"{ros_bridge}.ROS{ros_v}RtxLidarHelper"),
                    ("lidarHelperPointcloud", f"{ros_bridge}.ROS{ros_v}RtxLidarHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "createLiRenderProduct.inputs:execIn"),
                    ("createLiRenderProduct.outputs:execOut", "lidarHelperMsg.inputs:execIn"),
                    ("createLiRenderProduct.outputs:execOut", "lidarHelperPointcloud.inputs:execIn"),
                    (
                        "createLiRenderProduct.outputs:renderProductPath",
                        "lidarHelperMsg.inputs:renderProductPath",
                    ),
                    (
                        "createLiRenderProduct.outputs:renderProductPath",
                        "lidarHelperPointcloud.inputs:renderProductPath",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # ? LiDAR
                    ("lidarHelperPointcloud.inputs:topicName", "lidar"),
                    ("lidarHelperPointcloud.inputs:frameId", "rtx_lidar"),
                    # ("lidarHelperPointcloud.inputs:nodeNamespace", "/sensor"),
                    ("lidarHelperPointcloud.inputs:type", "point_cloud"),
                ],
            },
        )
    except Exception as e:
        print(e)

    set_targets(
        prim=stage.GetPrimAtPath(cfg.lidar.action_graph_path + "/createLiRenderProduct"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[cfg.lidar.lidar_stage_path],
    )

    simulation_app.update()
