import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.kit import SimulationApp
from pxr.Usd import Stage

# from src.config import Config

from configs.main_config import MainConfig


def setup_tf_graph(cfg: MainConfig, simulation_app: SimulationApp, stage: Stage):
    """Setup the action graph for publishing Husky and LIDAR tf transforms to ROS1/2"""

    ros_bridge = "omni.isaac.ros_bridge".split("-")[0]
    ros_v = 1

    controller = og.Controller(graph_id="ros_tf_graph")

    (graph, _, _, _) = controller.edit(
        {"graph_path": cfg.tf_action_graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                # * TF Tree
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                # ("PublishClock", f"{ros_bridge}.ROS{ros_v}PublishClock"),
                ("ReadSysTime", "omni.isaac.core_nodes.IsaacReadSystemTime"),
                # aloha
                ("tfPublisher", f"{ros_bridge}.ROS{ros_v}PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                # ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "tfPublisher.inputs:execIn"),
                ("ReadSysTime.outputs:systemTime", "tfPublisher.inputs:timeStamp"),
                # ("ReadSysTime.outputs:systemTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )


    # * TF Tree
    set_targets(
        prim=stage.GetPrimAtPath(cfg.tf_action_graph_path + "/tfPublisher"),
        attribute="inputs:parentPrim",
        target_prim_paths=[cfg.robot_stage_path + "/base_link"],  #  "/World"
    )
    set_targets(
        prim=stage.GetPrimAtPath(cfg.tf_action_graph_path + "/tfPublisher"),
        attribute="inputs:targetPrims",
        target_prim_paths=[cfg.robot_stage_path + "/fl_link6/gripper_center"],
    )


    simulation_app.update()
