import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.kit import SimulationApp
from pxr.Usd import Stage

# from src.config import Config
from configs.main_config import MainConfig


def setup_joint_state(cfg: MainConfig, simulation_app: SimulationApp, stage: Stage):
    """Setup the action graph for publishing Husky IMU measurements to ROS1/2"""
    keys = og.Controller.Keys


    ros_bridge = "omni.isaac.ros_bridge".split("-")[0]
    ros_v = 1

    try:
        og.Controller.edit(
            {
                "graph_path": cfg.joint_action_graph_stage_path,
                "evaluator_name": "execution",
            },
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishJointState", "omni.isaac.ros_bridge.ROS1PublishJointState"),
                    ("ReadSysTime", "omni.isaac.core_nodes.IsaacReadSystemTime"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("ReadSysTime.outputs:systemTime", "PublishJointState.inputs:timeStamp"),
                ],
                keys.SET_VALUES: [
                    # Setting topic names to joint state publisher and subscriber.
                    ("PublishJointState.inputs:topicName", "/joint_states"),
                    # ("PublishJointState.inputs:targetPrim", cfg.robot_stage_path)
                ],
            },
        )
    except Exception as e:
        print(e)

    set_targets(
        prim=stage.GetPrimAtPath(cfg.joint_action_graph_stage_path + "/PublishJointState"),
        attribute="inputs:targetPrim",
        target_prim_paths=[cfg.robot_stage_path],
    )

    simulation_app.update()
