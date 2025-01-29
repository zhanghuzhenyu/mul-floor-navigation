import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.kit import SimulationApp
from pxr.Usd import Stage

from src.config import Config


def setup_imu_graph(cfg: Config, simulation_app: SimulationApp, stage: Stage):
    """Setup the action graph for publishing Husky IMU measurements to ROS1/2"""
    keys = og.Controller.Keys

    ros_bridge = cfg.ros_cfg[cfg.ros].ros_bridge_extension.split("-")[0]
    ros_v = cfg.ros_cfg[cfg.ros].ros_v

    try:
        og.Controller.edit(
            {
                "graph_path": cfg.imu.action_graph_path,
                "evaluator_name": "execution",
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("imuReader", "omni.isaac.sensor.IsaacReadIMU"),
                    ("publishImu", f"{ros_bridge}.ROS{ros_v}PublishImu"),
                    ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "imuReader.inputs:execIn"),
                    ("imuReader.outputs:execOut", "publishImu.inputs:execIn"),
                    ("imuReader.outputs:angVel", "publishImu.inputs:angularVelocity"),
                    ("imuReader.outputs:linAcc", "publishImu.inputs:linearAcceleration"),
                    ("imuReader.outputs:orientation", "publishImu.inputs:orientation"),
                    ("readSimTime.outputs:simulationTime", "publishImu.inputs:timeStamp"),
                ],
                keys.SET_VALUES: [
                    ("publishImu.inputs:topicName", "IMU"),
                    ("publishImu.inputs:nodeNamespace", "/sensor"),
                    ("publishImu.inputs:frameId", "IMU"),
                ],
            },
        )
    except Exception as e:
        print(e)

    set_targets(
        prim=stage.GetPrimAtPath(cfg.imu.action_graph_path + "/imuReader"),
        attribute="inputs:imuPrim",
        target_prim_paths=[cfg.imu.imu_stage_path],
    )

    simulation_app.update()
