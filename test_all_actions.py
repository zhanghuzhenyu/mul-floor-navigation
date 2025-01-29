from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros_bridge")
from omni.isaac.core import World
import omni.isaac.core.tasks as tasks
import os
from omni.isaac.dynamic_control import _dynamic_control
from tasks.pick_place import PickPlace
from configs.main_config import MainConfig
from controllers.robot_controller import MobileManipulatorController
import asyncio
import numpy as np
from omni.isaac.core import SimulationContext
from pxr import PhysxSchema, UsdPhysics
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from pxr import UsdGeom, Gf, Usd
import math
import omni.kit.commands
import omni.usd
from omni.isaac.core.articulations import Articulation
from pxr import UsdGeom, Sdf, Gf, Vt, PhysicsSchemaTools
import omni.physxdemos as demo
from omni.physx import get_physx_scene_query_interface
import omni.kit.viewport.utility as vp_utils
from omni.physx.scripts.utils import CameraTransformHelper
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from omni.usd import get_world_transform_matrix
import rospy
from std_msgs.msg import String
import json
import sys
from elevator_door.msg import ElevatorDoorState

####### Config ############
config = MainConfig()
###########################

def hide_prim(prim_path: str):
    """Hide a prim

    Args:
        prim_path (str, required): The prim path of the prim to hide
    """
    set_prim_visibility_attribute(prim_path, "invisible")


def show_prim(prim_path: str):
    set_prim_visibility_attribute(prim_path, "inherited")

def set_prim_visibility_attribute(prim_path: str, value: str):
    prop_path = f"{prim_path}.visibility"
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(prop_path), value=value, prev=None
    )

def smooth_interpolate(t_current, t_start, t_end, min_position, max_position):
    if t_current < t_start:
        return min_position
    elif t_current > t_end:
        return max_position

    t_normalized = (t_current - t_start) / (t_end - t_start)
    smooth_position = min_position + (max_position - min_position) * (0.5 - 0.5 * math.cos(math.pi * t_normalized))

    return smooth_position

def set_trans(prim, axis, value):

    if not prim.IsValid():
        print(f"Prim don't exist")
        return -1

    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        print(f"Prim is not Xformable")
        return -1

    if axis not in {"x", "y", "z"}:
        print(f"pls use 'x', 'y' or 'z'")
        return -1

    xform_ops = xformable.GetOrderedXformOps()

    translate_op = next((op for op in xform_ops if op.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)

    if translate_op:
        original_translate = translate_op.Get()
    else:
        original_translate = Gf.Vec3d(0, 0, 0)

    new_translate = Gf.Vec3d(
        value if axis == "x" else original_translate[0],
        value if axis == "y" else original_translate[1],
        value if axis == "z" else original_translate[2]
    )

    if translate_op:
        translate_op.Set(new_translate)
    else:
        xformable.AddTranslateOp().Set(new_translate)

    return 0


def control_elevator_doors(t_current, actions):

    left_door_min = 0.0
    left_door_max = -82.43
    right_door_min = 0.0
    right_door_max = 82.43

    left_door_position = left_door_min
    right_door_position = right_door_min

    for action in actions:
        t_start = action['start']
        t_end = action['end']
        direction = action['direction']

        if t_current >= t_start and t_current <= t_end:
            if direction == 'open':
                left_door_position = smooth_interpolate(t_current, t_start, t_end, left_door_min, left_door_max)
                right_door_position = smooth_interpolate(t_current, t_start, t_end, right_door_min, right_door_max)
            elif direction == 'close':
                left_door_position = smooth_interpolate(t_current, t_start, t_end, left_door_max, left_door_min)
                right_door_position = smooth_interpolate(t_current, t_start, t_end, right_door_max, right_door_min)
            else:
                raise ValueError("direction must be 'open' or 'close'")
            
            if action[floor_idx] == 1:
                floor = "root" # root--> f1
            elif action[floor_idx] == 2:
                floor = "root_01"
            elif action[floor_idx] ==3:
                floor = "root_02"

            left_outdoor_prim_path = f"/World/env/_223/{floor}/GLTF_SceneRootNode/Sketchfab_model_0/root_1/GLTF_SceneRootNode_2/RightOutsideDoor_1_5"
            right_outdoor_prim_path = f"/World/env/_223/{floor}/GLTF_SceneRootNode/Sketchfab_model_0/root_1/GLTF_SceneRootNode_2/LeftOutsideDoor_0_3"

            left_door_prim = stage.GetPrimAtPath(left_outdoor_prim_path)
            right_door_prim = stage.GetPrimAtPath(right_outdoor_prim_path)

            set_trans(left_door_prim, "z", left_door_position)
            set_trans(right_door_prim, "z", right_door_position)

    return left_door_position, right_door_position


def control_elevator_height(t_current, actions, with_robot):

    aloha_prim_path = "/World/aloha"
    elevator_cage_prim_path = "/World/env/_223/root/GLTF_SceneRootNode/Sketchfab_model_0/root_1/GLTF_SceneRootNode_2/ElevatorCage_11_11"

    aloha_prim = stage.GetPrimAtPath(aloha_prim_path)
    elevator_cage_prim = stage.GetPrimAtPath(elevator_cage_prim_path)
    
    for action in actions:
        t_start = action['start']
        t_end = action['end']
        current_floor = action["current_floor"]
        target_floor = action["target_floor"]

        current_elevator_heigth = 100 + (current_floor-1)*350
        target_elevator_height = 100 + (target_floor-1)*350
        elevator_height = current_elevator_heigth

        current_aloha_height = 0 + (current_floor-1)*3.5
        target_aloha_height = 0 + (target_floor-1)*3.5
        aloha_height = current_aloha_height
        if t_current >= t_start and t_current <= t_end:
            elevator_height = smooth_interpolate(t_current, t_start, t_end, current_elevator_heigth, target_elevator_height)
            aloha_height = smooth_interpolate(t_current, t_start, t_end, current_aloha_height, target_aloha_height)
            
            set_trans(elevator_cage_prim, "y", elevator_height)
            if with_robot:
                set_trans(aloha_prim, "z", aloha_height)

    return elevator_height, aloha_height

def get_position(input_string):
    """
    Get the 3D position corresponding to the input string using dictionary mapping.

    Parameters:
        input_string (str): The input string for which the 3D position is required.

    Returns:
        tuple: A tuple representing the 3D position (x, y, z), or None if not found.
    """
    position_mapping = {
        "1F cabinet": (2.41869, -2.68171, 0),
        "1F trash bin": (0.36542, -2.67103, 0),
        "1F sofa": (3.17938, 2.40993, 0),
        "1F refrigerator": (6.10793, 2.58895, 0),
        "1F painting": (7.9272, 1.32871, 0),
        "1F blind": (7.90247, -1.32727, 0),
        "1F plant": (0.5, 3.5347, 0),
        "2F blind": (7.90247, -1.32727, 0),
        "1F elevator button panel": (0.1727, -1.335, 1.3),

        "2F trash bin": (0.39659, -2.27471, 3.5),
        "2F cabinet": (2.85098, -2.55979, 3.5),
        "2F sofa chair": (5.41918, -2.39878, 3.5),
        "2F table": (4.40159, 0, 3.5),
        "2F wardrobe": (5.83614, 2.60251, 3.5),
        "2F chair": (2.10173, 2.32465, 3.5),
        "2F desk": (0.30492, 2.09116, 3.5),
        "2F blind": (7.90247, -1.32727, 3.5),
        "2F elevator button panel": (0.1727, -1.335, 1.3 + 3.5),

        "3F trash bin": (1.24433, -2.30942, 7),
        "3F bed": (4.71966, 1.81299, 7),
        "3F table": (6.27138, -2.68833, 7),
        "3F blind": (7.90247, -1.32727, 7),
        "3F elevator button panel": (0.1727, -1.335, 1.3 + 7),
    }
    
    return position_mapping.get(input_string, None)

def report_hit(hit):

    return True

action_list = [
            'Move to', 
            'Press',
            'Enter',
            'Press',
            'Exit',
            'Move to',

            'Move to',
            'Press',
            'Enter',
            'Press',
            'Exit',
            'Move to',
            ] 
target_list = [
            '2F elevator button panel', 
            '2F up button', 
            '2F elevator',
            '2F button', 
            '3F elevator',
            '3F blind',

            '3F elevator button panel',
            '3F down button', 
            '3F elevator',
            '1F button', 
            '1F elevator',
            '1F plant'
            ] 
floor_list = [
            '2', 
            '2',
            '2',
            '2', 
            '3', 
            '3',

            '3', 
            '3',
            '3',
            '3', 
            '1', 
            '1',
            ]

############################ 3D Scene Graph query (works) #######################

from conceptgraph.clip_object_query import CLIPObjectQuery

clip_query = CLIPObjectQuery()
result_SG_list = []

for i in range(len(action_list)):
    result_SG_path_F1 = "/media/zhang/scene/Isaac/temp1/exps/r_mapping_stride16/pcd_r_mapping_stride16.pkl.gz"  
    result_SG_path_F2 = "/media/zhang/scene/Isaac/temp2/exps/r_mapping_stride16/pcd_r_mapping_stride16.pkl.gz"  
    result_SG_path_F3 = "/media/zhang/scene/Isaac/temp3/exps/r_mapping_stride16/pcd_r_mapping_stride16.pkl.gz"  
    result_SG_path_cage = "/media/zhang/scene/Isaac/tempe/exps/r_mapping_stride16/pcd_r_mapping_stride16.pkl.gz" 

    text_query = target_list[i] 
    current_floor = text_query[0]

    if target_list[i][-8:] == 'F button':
        result_SG = clip_query.query(result_SG_path_cage, text_query)
    elif current_floor == '1':
        result_SG = clip_query.query(result_SG_path_F1, text_query)
    elif current_floor == '2':
        result_SG = clip_query.query(result_SG_path_F2, text_query)
    elif current_floor == '3':
        result_SG = clip_query.query(result_SG_path_F3, text_query)

    result_SG_list.append(result_SG)
    print(f"Most probable object is at index {result_SG['index']} with class name '{result_SG['class_name']}' in the {current_floor} floor")
    print(f"location xyz: {result_SG['location_xyz']}")

############################ Navigation Graph (works) ###########################
from nav_graph.nav_graph_dij import NavGraph  
import cv2

graph_path_F1 = "/home/zhang/HOV-SG/data/nav_graph/isaac_1_22_temp1/graph/nav_graph/nav_graph.json"  
graph_path_F2 = "/home/zhang/HOV-SG/data/nav_graph/isaac_1_22_temp2/graph/nav_graph/nav_graph.json"  
graph_path_F3 = "/home/zhang/HOV-SG/data/nav_graph/isaac_1_22_temp3/graph/nav_graph/nav_graph.json"  
save_traj_path_dir = "/home/zhang/exp_log/nav_graph"

start_point = config.robot_init_pose
end_point = result_SG_list[0]['location_xyz']

nav_graph_F1 = NavGraph(graph_path_F1, z_value=0)
nav_graph_F2 = NavGraph(graph_path_F2, z_value=3.5)
nav_graph_F3 = NavGraph(graph_path_F3, z_value=7)

############################ ISAAC INIT #################################

tasks = []
robot_controllers = []
object_names = []
objects = []

my_world = World(stage_units_in_meters=1.0, device="cpu")  # backend="torch", device="cuda"
my_world.scene.add_default_ground_plane()

my_world._physics_context.enable_gpu_dynamics(True)
my_world.initialize_physics()

############################ SETUP SCENE #############################
'''
    Add multiple tasks
'''
num_of_tasks = 1

for i in range(num_of_tasks):
    print(i)
    my_world.add_task(PickPlace(name="task_" + str(i), world = my_world, config = config, offset=np.array([0, (i * 30), 0])))
    task = my_world.get_task(name="task_" + str(i))
    print(task)
    tasks.append(task)

my_world.reset()

############################ SETUP TASKS #############################
'''
    Add info about tasks
'''
for i in range(num_of_tasks):
    controller = MobileManipulatorController(task, my_world, simulation_app, config = config)
    robot_controllers.append(controller)    

my_world.reset()

############################ CHANGE THE LIGHT #################################
stage = get_current_stage()

mesh1 = "/World/env/_223/root_01/GLTF_SceneRootNode/Sketchfab_model_0/root_1/GLTF_SceneRootNode_2/ElevatorCallingButtons_12_33/Object_34_34/Object_56/Object_18"
mesh2 = "/World/env/_223/root_01/GLTF_SceneRootNode/Sketchfab_model_0/root_1/GLTF_SceneRootNode_2/ElevatorCallingButtons_12_33/Object_35_35/Object_58/Object_19"
button_mesh_prim1 = stage.GetPrimAtPath(mesh1)
button_mesh_prim2 = stage.GetPrimAtPath(mesh2)
api1 = UsdPhysics.CollisionAPI.Apply(button_mesh_prim1)
api1.GetCollisionEnabledAttr().Set(True)
api2 = UsdPhysics.CollisionAPI.Apply(button_mesh_prim2)
api2.GetCollisionEnabledAttr().Set(True)

############################ INIT #################################
camera_prim= stage.GetPrimAtPath("/World/aloha/box2_Link/visuals/cam01_root/cam01_parent/Camera_1")
focal_length = camera_prim.GetAttribute("focalLength")
horizontal_aperture = camera_prim.GetAttribute("horizontalAperture")

print("focal length:", focal_length.Get())
print("horizontal aperture:", horizontal_aperture.Get())
horizontal_aperture.Set(70)  # RealSense D435 ->87

hide_prim("/World/defaultGroundPlane/Environment/Geometry") 

current_target_idx = 0 
press_finish = False
first_hit = True
t_step = 0
elevator_height_set = False
elevator_door_set = False
floor_idx = 0
first_into_action_loop = True
get_button_pos_ROS = False
elevator_door_is_open = False
button_radius = 0.04
# button_center = np.array([0, 0, 0])
press_success = False
move_success = False
min_distance_inter_point = 0.05
min_distance_final_point = 0.7 # 0.35
rotate_not_finish = True
should_subscribe_button_detect = True
use_VORONOI = True
use_DOOR_STATUS_DETECT = False

# --------------- ROS ----------------
rospy.init_node('button_info_listener', anonymous=True)

button_positions = {
    '1': None,
    '2': None,
    '3': None,
    '(': None,
    ')': None
}
# ------------------------------------

for i in range(50):
    my_world.step(render=True)
    print(i)
    try:
        button_info = rospy.wait_for_message('/button_info', String, timeout=0.1)
        msg_data = json.loads(button_info.data)
        all_button_position = msg_data
        print("all_button_position:", all_button_position)
        print(f"[{i}] Received button info: {button_info.data}")
    except rospy.ROSException as e:
        print(f"[{i}] Timeout while waiting for message: {e}")


############################ START SIMULATION ############################ 
trajectory_points_entry = [np.array([0.5,0,0]), np.array([-1.05, 0.65, 0])]
trajectory_points_exit = [np.array([-0.5, 0.0, 0.0]), np.array([1.3, 0.0, 0.0])]
outside_pre_press_position = np.array([0.70, -1.06, 0])


#### Set Task Tracker ###
from task_tracker.task_tracker import TaskTracker
tracker = TaskTracker(action_list, target_list, save_dir="/home/zhang/exp_log", config=config)
#### Set Task Tracker ###

while simulation_app.is_running() and not simulation_app.is_exiting() and not tracker.is_complete():

    my_world.step(render=True)      
    print(f"t_step: {t_step}")
    t_step = t_step + 1
    i = t_step
    
    current_observations = tasks[0].get_observations()

    current_task = tracker.current_task()
    print(f"Current Task: Action = '{current_task['action']}', Target = <{current_task['target']}>")
    ################## Move to ############
    if current_task['action'] == 'Move to':
        # tracker.mark_task_complete()
        if first_into_action_loop:
            should_subscribe_button_detect = False
            current_observations = tasks[0].get_observations()
            start_point = current_observations[tasks[0]._robot.name]['robot_position']
            if current_task['target'].split()[-1] == 'panel': 
                end_point = outside_pre_press_position
                min_distance_final_point = 0.05
            else:
                end_point = result_SG_list[current_task['index']]['location_xyz']
                min_distance_final_point = 0.5
            os.makedirs(save_traj_path_dir, exist_ok=True)
            save_traj_path =  os.path.join(save_traj_path_dir, f"move_to_{current_task['target']}_{current_task['index']}.png")            
            if use_VORONOI:
                if target_list[current_task['index']][0] == "1":
                    trajectory_points = nav_graph_F1.compute_trajectory(start_point, end_point)
                    nav_graph_F1.visualize_trajectory(trajectory_points, save_path=save_traj_path)
                elif target_list[current_task['index']][0] == "2":
                    trajectory_points = nav_graph_F2.compute_trajectory(start_point, end_point)
                    nav_graph_F2.visualize_trajectory(trajectory_points, save_path=save_traj_path)
                elif target_list[current_task['index']][0] == "3":
                    trajectory_points = nav_graph_F3.compute_trajectory(start_point, end_point)    
                    nav_graph_F3.visualize_trajectory(trajectory_points, save_path=save_traj_path)      
            else:
                if target_list[current_task['index']][0] == "1":
                    path_world = A_graph_F1.plan(start_point[[0,1]], end_point[[0,1]])
                    A_graph_F1.visualize_path(path_world, "/home/zhang/exp_log/nav_graph_A/path_planning_result.png")
                elif target_list[current_task['index']][0] == "2":
                    path_world = A_graph_F2.plan(start_point[[0,1]], end_point[[0,1]])
                    A_graph_F2.visualize_path(path_world, "/home/zhang/exp_log/nav_graph_A/path_planning_result.png")
                elif target_list[current_task['index']][0] == "3":
                    path_world = A_graph_F3.plan(start_point[[0,1]], end_point[[0,1]])
                    A_graph_F3.visualize_path(path_world, "/home/zhang/exp_log/nav_graph_A/path_planning_result.png")    
                print("Simplified path in world coordinates:", path_world)
                path_world_expanded = [np.append(point, (int(floor_list[current_task['index']])-1)*3.5) for point in path_world]
                trajectory_points = path_world_expanded      
            first_into_action_loop = False
            target_object_GT_position = get_position(current_task['target'])

        if current_target_idx + 1 <= len(trajectory_points):
            if current_target_idx + 1 < len(trajectory_points):
                min_distance = min_distance_inter_point
            elif current_target_idx + 1 == len(trajectory_points): # Last traj point
                min_distance = min_distance_final_point
            goal_position = trajectory_points[current_target_idx]
            # if navigation is completed, move_completed = False
            move_completed = robot_controllers[0].move_to_location(goal_position=goal_position, min_distance=min_distance)
            if not move_completed:
                tracker.add_trajectory_point(current_observations[tasks[0]._robot.name]['robot_position'])
                current_target_idx += 1
                robot_controllers[0].move_first_step = True
                print(f"Moving to next target point {current_target_idx}")
        else:
            if current_task['target'].split()[-1] == 'panel' and rotate_not_finish: 
                end_rotation_angle = np.array([0, 0, 180]) # * np.pi / 180
                rotate_not_finish = robot_controllers[0].rotate_to_location(goal_rotation=end_rotation_angle, min_rotation_error= 0.1)
            else:        
                print('DONE MOVING!!! Trajectory list alreay finish~ ')
                current_observations = tasks[0].get_observations()
                current_robot_position = current_observations[tasks[0]._robot.name]['robot_position']
                current_robot_position_2d = current_robot_position[[0,1]]
                # TODO
                target_object_GT_position_2d = np.array(target_object_GT_position)[[0,1]] # x-y
                robot_move_to_goal_distance = np.linalg.norm(current_robot_position_2d - target_object_GT_position_2d)
                print("robot_move_to_goal_distance: ", robot_move_to_goal_distance)
                if robot_move_to_goal_distance < 1.5:
                    move_success = True
                else:
                    move_success = False
                robot_controllers[0].move_first_step = True
                current_target_idx = 0
                robot_controllers[0]._pick_place_controller = None
                robot_controllers[0].press_first_step = True
                first_into_action_loop = True
                tracker.mark_task_complete(success=move_success, timestamps=t_step, dis_to_goal=robot_move_to_goal_distance)
                move_success = False
                rotate_not_finish = True
                should_subscribe_button_detect = True

    ################## Press ############
    elif current_task['action'] == 'Press':  
        
        if first_into_action_loop:
            should_subscribe_button_detect = True
            if floor_list[current_task['index']] == '1':
                floor_path = 'root'
            elif floor_list[current_task['index']] == '2':
                floor_path = 'root_01'
            elif floor_list[current_task['index']] == '3':
                floor_path = 'root_02'
            if "up" in current_task['target'] or "down" in current_task['target']:
                inside = False
                init_numHits = 2  
                mesh_path = f"/World/env/_223/{floor_path}/GLTF_SceneRootNode/Sketchfab_model_0/root_1/GLTF_SceneRootNode_2/ElevatorCallingButtons_12_33/Object_34_34/Object_56/Object_18"
                target_button_type_up_down = current_task['target'].split()[1]
                if target_button_type_up_down == "up":
                    target_button_type = '('
                    GT_button_prim_path = f"/World/env/_223/{floor_path}/GLTF_SceneRootNode/Sketchfab_model_0/root_1/GLTF_SceneRootNode_2/ElevatorCallingButtons_12_33/GT_pos_up"
                    GT_button_prim = stage.GetPrimAtPath(GT_button_prim_path)
                    GT_button_world_transform = get_world_transform_matrix(GT_button_prim)
                    GT_button_position = Gf.Vec3d(GT_button_world_transform.ExtractTranslation())
                elif target_button_type_up_down == "down":
                    target_button_type = ')'
                    GT_button_prim_path = f"/World/env/_223/{floor_path}/GLTF_SceneRootNode/Sketchfab_model_0/root_1/GLTF_SceneRootNode_2/ElevatorCallingButtons_12_33/GT_pos_down"
                    GT_button_prim = stage.GetPrimAtPath(GT_button_prim_path)
                    GT_button_world_transform = get_world_transform_matrix(GT_button_prim)
                    GT_button_position = Gf.Vec3d(GT_button_world_transform.ExtractTranslation())
            else:
                inside = True
                init_numHits = 2 
                mesh_path = f"/World/env/_223/root/GLTF_SceneRootNode/Sketchfab_model_0/root_1/GLTF_SceneRootNode_2/ElevatorCage_11_11/ElevatorInteriorButtons_4_14/Object_16_16/Object_27/Object_7"
                target_button_type = current_task['target'][0]
                # GT_pos_button_F1 || GT_pos_button_F2 || GT_pos_button_F3
                GT_button_prim_path = f"/World/env/_223/root/GLTF_SceneRootNode/Sketchfab_model_0/root_1/GLTF_SceneRootNode_2/ElevatorCage_11_11/ElevatorInteriorButtons_4_14/GT_pos_button_F{target_button_type}"
                GT_button_prim = stage.GetPrimAtPath(GT_button_prim_path)
                GT_button_world_transform = get_world_transform_matrix(GT_button_prim)
                GT_button_position = Gf.Vec3d(GT_button_world_transform.ExtractTranslation())
            first_into_action_loop = False
        goal_position = current_observations[tasks[0]._robot.name]['robot_position']
        move_completed = robot_controllers[0].move_to_location(goal_position=goal_position, min_distance=0.1, just_stop=True)

        if not first_hit:
            # button_center = result_SG_list[current_task['index']]['location_xyz']            
            robot_controllers[0].pickup_object(press_detect = True, inside = inside, button_position = button_center)        
        else:
            # ---------- ROS --------------
            if not get_button_pos_ROS:
                for button_text, position in all_button_position.items():
                    if target_button_type == button_text:
                        button_center = position
                        get_button_pos_ROS = True
                        should_subscribe_button_detect = False
                        break

            ee_world_pos = current_observations[tasks[0]._robot.manipulator.name]['end_effector_world_position']
            print("ee_world_pos: ", ee_world_pos)
            robot_controllers[0].pickup_object(press_detect = False, inside = inside, button_position = button_center)

        path_tuple = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(mesh_path))
        numHits = get_physx_scene_query_interface().overlap_shape(path_tuple[0], path_tuple[1], report_hit, False)
        print(f"numHits: {numHits}")
        if numHits > init_numHits and first_hit:
            first_hit = False
            hit_ee_world_pos = current_observations[tasks[0]._robot.manipulator.name]['end_effector_world_position']
            print(f"hit_ee_position: {hit_ee_world_pos}")
            print('PRESS DETECTED!!! ')
            # /World/aloha/fl_link6/gripper_center
            gripper_center = stage.GetPrimAtPath("/World/aloha/fl_link6/gripper_center")
            world_transform = get_world_transform_matrix(gripper_center)
            gripper_center_position_first_detected = Gf.Vec3d(world_transform.ExtractTranslation())
            print("GT_button_position: ", GT_button_position)
            print("gripper_center_position_first_detected: ", gripper_center_position_first_detected)
            if not inside:
                GT_2d = np.array(GT_button_position)[1:] 
                gripper_2d = np.array(gripper_center_position_first_detected)[1:]  
            else:
                GT_2d = np.array(GT_button_position)[[0,2]] 
                gripper_2d = np.array(gripper_center_position_first_detected)[[0,2]]   
            press_button_distance = np.linalg.norm(GT_2d - gripper_2d)
                
            print("press_button_distance: ", press_button_distance)
            if press_button_distance < button_radius:
                press_success = True
            else:
                press_success = False

            if inside:
                start_elevator = t_step
                elevator_height_actions = [ 
                    {'start': start_elevator+30, 'end': start_elevator + 60, 'current_floor': 1, 'target_floor': int(floor_list[current_task['index']+1])},  # 在10到30之间上升
                ]
                elevator_height_set = True
                start_door = t_step
                elevator_door_actions = [ 
                    {'start': start_door+70, 'end': start_door + 100, 'direction': 'open', floor_idx:int(floor_list[current_task['index']+1])}, 
                ]
                elevator_door_set = True
            else: 
                start_door = t_step
                elevator_door_actions = [ 
                    {'start': start_door+10, 'end': start_door + 30, 'direction': 'open', floor_idx:int(floor_list[current_task['index']])},  # 在10到30之间开门
                ]
                elevator_door_set = True
        if robot_controllers[0]._pick_place_controller.pick_done():
            print('AREADY BACK TO INIT JOINT STATE!')
            first_hit = True
            numHits = 0
            robot_controllers[0]._pick_place_controller = None
            first_into_action_loop = True
            get_button_pos_ROS = False
            tracker.mark_task_complete(success=press_success, timestamps=t_step)
            press_success = False
    ################## Enter ############
    elif current_task['action'] == 'Enter':   
        should_subscribe_button_detect = False
        trajectory_points = trajectory_points_entry + np.array([0, 0, (int(floor_list[current_task['index']])-1)*3.5])
        if current_target_idx + 1 <= len(trajectory_points):
            
            min_distance = 0.05
            goal_position = trajectory_points[current_target_idx]
            # if navigation is completed, move_completed = False
            move_completed = robot_controllers[0].move_to_location(goal_position=goal_position, min_distance=min_distance)
            if not move_completed:
                current_target_idx += 1
                tracker.add_trajectory_point(current_observations[tasks[0]._robot.name]['robot_position'])
        else: 
            end_rotation_angle = np.array([0, 0, 90]) # * np.pi / 180
            rotate_not_finish = robot_controllers[0].rotate_to_location(goal_rotation=end_rotation_angle, min_rotation_error= 0.1)
            if not rotate_not_finish:
                print('DONE MOVING!!! Trajectory list alreay finish~ ')
                robot_controllers[0].move_first_step = True
                current_target_idx = 0
                robot_controllers[0]._pick_place_controller = None
                robot_controllers[0].press_first_step = True
                first_into_action_loop = True
                elevator_door_is_open = False
                tracker.mark_task_complete(timestamps=t_step)
                rotate_not_finish = True

    ################## Exit ############
    elif current_task['action'] == 'Exit':   
        trajectory_points = trajectory_points_exit + np.array([0, 0, (int(floor_list[current_task['index']])-1)*3.5])
        if current_target_idx + 1 <= len(trajectory_points):
            
            min_distance = 0.05
            goal_position = trajectory_points[current_target_idx]
            # if navigation is completed, move_completed = False
            move_completed = robot_controllers[0].move_to_location(goal_position=goal_position, min_distance=min_distance)

            if not move_completed:
                current_target_idx += 1
                tracker.add_trajectory_point(current_observations[tasks[0]._robot.name]['robot_position'])
        else:
            robot_controllers[0].move_first_step = True
            print(f"Moving to next target point {current_target_idx}")
            current_target_idx = 0
            robot_controllers[0]._pick_place_controller = None
            robot_controllers[0].press_first_step = True
            first_into_action_loop = True
            tracker.mark_task_complete(timestamps=t_step)

    ################## Elevator Control ###########
    if elevator_door_set:
        left_door_position, right_door_position = control_elevator_doors(t_step, elevator_door_actions)
    else:
        elevator_door_actions = [ 
            {'start': 0, 'end': 1, 'direction': 'close', floor_idx:int(floor_list[0])}, 
        ]
        left_door_position, right_door_position = control_elevator_doors(t_step, elevator_door_actions)

    if elevator_height_set:
        elevator_height, aloha_height = control_elevator_height(t_step, elevator_height_actions, with_robot=True)
    else: # DEFAULT HEIGHT
        elevator_height_actions = [
            {'start': 0, 'end': 1, 'current_floor': int(floor_list[0]), 'target_floor': int(floor_list[0])}, 
        ]
        elevator_height, aloha_height = control_elevator_height(t_step, elevator_height_actions, with_robot=False)

    try:
        button_info = rospy.wait_for_message('/button_info', String, timeout=0.1)
        msg_data = json.loads(button_info.data)
        all_button_position = msg_data

    except rospy.ROSException as e:
        print(f"[{i}] Timeout while waiting for message: {e}")

    simulation_app.update()
simulation_app.close() 
