from dataclasses import asdict, dataclass
from typing import Any, DefaultDict, Dict, Optional, Tuple, Union
import numpy as np

@dataclass
class MainConfig: 


    robot_usd_path: str =  '/home/zhang/asset/usd_4_0.usd'
    robot_stage_path: str = '/World/aloha'    
    
    # setup task params: 
    log_camera_data: bool = True
    log_lidar_data: bool = False

    # # 
    ur5_relative_pose: Tuple[float, ...] = (0.3312, 0, 0.257)
    ur5_init_pose: Tuple[float, ...] = (0, -0.5, -2, 0, 1.570, 0)
    joints_default_positions: Tuple[float, ...] = (3.1415927, -2.871760, 2.799204, -3.072348, -1.581982, -0.000120)
    end_effector_initial_height: float = 0.65

    # scene setup 
    env_usd_path: str = '/home/zhang/.local/share/ov/pkg/isaac_sim-2022.2.1/Nikita/Isaac_Sim_Rearrangement_task-main/aloha/warehouse_9.usd' #'/isaac-sim/standalone_examples/aloha/scenes/RoboLab2024-kitchen/kitchen_1.usd' #'/isaac-sim/standalone_examples/aloha/scenes/warehouse_9.usd'
    env_name: str = 'Warehouse_0'
    env_obj_name: str = 'Warehouse_0'
    env_prim_path: str = '/World/' + env_name

    # object setup
    object_usd_path: str =  '/home/zhang/.local/share/ov/pkg/isaac_sim-2022.2.1/Nikita/Isaac_Sim_Rearrangement_task-main/aloha/sber_kitchen_bowl3.usd' #'/isaac-sim/standalone_examples/aloha/assets/objects/sber_kitchen_bowl1.usd' # for import custom usd model
    object_name: str = 'waitingbench'
    object_prim_path: str = '/World/env' + object_name
    object_init_position: Tuple[float, ...] = (2.3, 2.124, 0.39) #(4.02, -4.11, 0.6) - шкаф #(4.42, -0.11, 0.6)#(3.1, 2.124, 0.39) #(2.3, 2.124, 0.39)
    # object_scale: Tuple[float, ...] = (0.00004, 0.00004, 0.00004)#(0.0004, 0.0004, 0.0004) - bowl #(0.006, 0.006, 0.006) #(0.008, 0.008, 0.008)
    object_scale: Tuple[float, ...] = (0.0004, 0.0004, 0.0004)
    
    # robot setup
    robot_init_pose: Tuple[float, ...] = (1, -1.5, 0)  
    # robot_init_ori: Tuple[float, ...] = (0, 0, 0, 1) 
    robot_init_ori: Tuple[float, ...] = (0, 0, 180) # 这里用角度 
    target_position: Tuple[float, ...] = (4, 0.1, 0)

    # controllers_setup:
    wheel_radius: float = 0.3
    wheel_base: float =  0.5

    lateral_velocity: float = 1.3 # 1.3 works
    yaw_velocity: float = 1.2 # 1.2 works
    position_tol: float = 0 # 0 works
    events_dt: Tuple[float, ...] = (0.004, 0.005, 0.05) # 0.003, 0.02, 0.05
    # end_effector_offset: Tuple[float, ...] = (+0.00, +0.013, +0.17) 
    end_effector_offset: Tuple[float, ...] = (+0.00, +0.00, +0.00) 
    #gripper_joint_closed_positions: Tuple[float, ...] = (1, 1)
    joint_opened_positions: Tuple[float, ...] = (0.048, 0.048) #(0.041, 0.041) # max value = 0.047
    joint_closed_positions: Tuple[float, ...] = (0.01, 0.01)
    end_effector_orientation: Tuple[float, ...] = (0, np.pi, 0)

    trans_point_relative_path: str = 'fl_link4' #'fl_base_link' #'fl_link8' #'box2_Link'#'fl_link6'

    cameras: Tuple[str, ...] = ('fr_link6/visuals/cam02_root/cam02_parent/Camera_2', 'fl_link6/visuals/cam03_root/cam03_parent/Camera_3', 'box2_Link/visuals/cam01_root/cam01_parent/Camera_1')

    depth_img_size: Tuple[float, ...] = (128, 128)
    img_size: Tuple[float, ...] = (128, 128, 3)

    # lidar setup - so heavy to render

    # relative to husky prim path
    
    lidar_relative_path: str = '/fence_link/fence_link_small/VLP_16/vlp16/lidar'

    # lidar configs, for more data check ./exts/omni.isaac.sensor/data/lidar_configs/ or https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_based_lidar.html#rtx-lidar-config-library
    lidar_pos: Tuple[float, ...] = (0., 0., 0.0147)
    min_range: float = 0.1
    max_range: float = 100.0
    draw_points: bool = True
    draw_lines: bool = False
    horizontal_fov: float = 360.0
    vertical_fov: float = 60.0
    horizontal_resolution: float = 0.4
    vertical_resolution: float = 0.4
    rotation_rate: float = 0.0
    high_lod: bool = True
    yaw_offset: float = 0.0
    enable_semantics: bool = True

    # logging setup 
    log_folder_path: str = '/home/zhang/.local/share/ov/pkg/isaac_sim-2022.2.1/Nikita/Isaac_Sim_Rearrangement_task-main/logs/'
    log_name: str = 'aloha_control'

    #Sensors info

    use_sensors: bool = True

    tf_action_graph_path: str = "/World/aloha/TF"

    tf_odom_action_graph_path: str = "/World/aloha/TF_ODOM"

    camera_action_graph_stage_path: str = "/World/aloha/CAMERA"

    joint_action_graph_stage_path: str = "/World/aloha/JOINT"