import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.kit import SimulationApp
from pxr.Usd import Stage

# from src.config import Config

from configs.main_config import MainConfig


def setup_tf_with_odom(cfg: MainConfig, simulation_app: SimulationApp, stage: Stage):
    """Setup the action graph for publishing Husky and LIDAR tf transforms to ROS1/2"""

    ros_bridge = "omni.isaac.ros_bridge".split("-")[0]
    ros_v = 1

    controller = og.Controller(graph_id="ros_tf_odom_graph")

    (graph, _, _, _) = controller.edit(
        {"graph_path": cfg.tf_odom_action_graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                # * TF Tree
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                # ("PublishClock", f"{ros_bridge}.ROS{ros_v}PublishClock"),
                # ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSystemTime"),
                # aloha tf
                ("tfRawPublisher", f"{ros_bridge}.ROS{ros_v}PublishRawTransformTree"),
                # Odometry Compute
                ("OdometryCompute", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                # Odometry Publish
                ("OdometryPublish", f"{ros_bridge}.ROS{ros_v}PublishOdometry"),
            ],
            og.Controller.Keys.CONNECT: [
                # ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "tfRawPublisher.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "OdometryCompute.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "OdometryPublish.inputs:execIn"),

                # ("ReadSimTime.outputs:simulationTime", "tfRawPublisher.inputs:timeStamp"),
                # ("ReadSimTime.outputs:simulationTime", "OdometryPublish.inputs:timeStamp"),

                ("ReadSimTime.outputs:systemTime", "tfRawPublisher.inputs:timeStamp"),
                ("ReadSimTime.outputs:systemTime", "OdometryPublish.inputs:timeStamp"),
                
                ("OdometryCompute.outputs:execOut", "OdometryPublish.inputs:execIn"),
                ("OdometryCompute.outputs:angularVelocity", "OdometryPublish.inputs:angularVelocity"),
                ("OdometryCompute.outputs:linearVelocity", "OdometryPublish.inputs:linearVelocity"),
                ("OdometryCompute.outputs:orientation", "OdometryPublish.inputs:orientation"),
                ("OdometryCompute.outputs:position", "OdometryPublish.inputs:position"),
                ("OdometryCompute.outputs:orientation", "tfRawPublisher.inputs:rotation"),
                ("OdometryCompute.outputs:position", "tfRawPublisher.inputs:translation"),

                
            ],
            og.Controller.Keys.SET_VALUES: [
                ("OdometryPublish.inputs:topicName", "/robot/base_1/odometry"),
                ("tfRawPublisher.inputs:topicName", "/tf_for_base"),

            ],
        },
    )

    # Odometry Compute
    set_targets(
        prim=stage.GetPrimAtPath(cfg.tf_odom_action_graph_path + "/OdometryCompute"),
        attribute="inputs:chassisPrim",
        target_prim_paths=[cfg.robot_stage_path],
    )

    simulation_app.update()
