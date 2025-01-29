import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.kit import SimulationApp

from omni.isaac.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController

# from omni.isaac.motion_generation import WheelBasePoseController
# from omni.isaac.wheeled_robots.controllers.differential_controller import (
#     DifferentialController,
# )
from omni.usd import get_world_transform_matrix

# from src.actions.base_server import ActionServer
# from src.config import Config
# from src.controllers.pick_place import PickPlaceController
# from src.sensors.cameras import setup_cameras
# from src.sensors.imu import setup_imu_graph
# from src.sensors.lidar import setup_lidar_graph
# from src.sensors.tf import setup_tf_graph
# from src.tasks.pick_place import PickPlace

from controllers.pick_place import PickPlaceController
from configs.main_config import MainConfig
from tasks.pick_place import PickPlace
from typing import List, Optional, Tuple
import asyncio

from sensors.tf_odom import setup_tf_with_odom
from sensors.tf import setup_tf_graph

from sensors.lidar import setup_lidar_graph
from sensors.cameras import setup_cameras
from sensors.state import setup_joint_state
from omni.isaac.core.utils.rotations import quat_to_euler_angles

class MobileManipulatorController:
    def __init__(self, task:PickPlace, world: World, simulation_app: SimulationApp, config: MainConfig) -> None:
        """
        A class that initializes and controls the Mobile Manipulator robot in the simulated environment.

        Args:
            cfg (Config): The configuration object for the simulation.
            world (World): The Isaac simulation world object.
            simulation_app (SimulationApp): The Isaac simulation application object.
        """
        self._task = task
        self._world = world
        self._simulation_app = simulation_app
        self._prim_trans_point = self._task._robots["trans_point"]

        self._config = config

        self._robot_controller = WheelBasePoseController(
            name="wheel_base_controller",
            open_loop_wheel_controller=DifferentialController(
                name="simple_control", wheel_radius=self._task._config.wheel_radius, wheel_base=self._task._config.wheel_base
            ),
            is_holonomic=False,
        )
        self._pick_place_controller = None
        self._articulation_controller = self._task._robot.manipulator.get_articulation_controller()

        # Set up sensors if enabled
        if self._config.use_sensors:
            self._setup_sensors()
        
        self.move_first_step = True
        self.press_first_step = True
        self.first_use_ocr = True
            
    def _setup_sensors(self):
        """
        Sets up the sensors for the simulation.
        """
        our_stage = get_current_stage()

        self._simulation_app.update()

        setup_cameras(self._config, self._simulation_app, our_stage, self._task.cameras)  # 'fl_link6/visuals/cam03_root/cam03_parent/Camera_3'

        # setup_tf_graph(self._config, self._simulation_app, our_stage)

        # setup_tf_with_odom(self._config, self._simulation_app, our_stage)

        # setup_joint_state(self._config, self._simulation_app, our_stage)

        
        # setup_lidar_graph(self._config, self._simulation_app, our_stage)
        # setup_imu_graph(self._cfg, self.simulation_app, our_stage)

    def move_to_location(self, goal_position: Tuple[float, ...], min_distance: float = 1.6, just_stop: Optional[bool] = False) -> bool:
        """
        Moves the robot to a specified location.

        Args:
            task: The task object containing the location to move to.
            action_server: The action server object for the move to location action.
        """
        if just_stop == True:
            print("Just stop!~")
            end_position, end_orientation = self._task._robot.get_world_pose()

            wheel_actions = self._robot_controller.forward(start_position=end_position,
                                                    start_orientation=end_orientation,
                                                    current_position=end_position,
                                                    current_orientation=end_orientation,
                                                    goal_position=end_position,
                                                    lateral_velocity=self._task._config.lateral_velocity,
                                                    yaw_velocity=self._task._config.yaw_velocity,
                                                    position_tol=self._task._config.position_tol,
                                                    control_type="stop")
            wheel_actions.joint_velocities = wheel_actions.joint_velocities
            self._task._robot.apply_wheel_actions(wheel_actions)
        else:
            distance = 100
            # goal_position = list(goal_position)
            goal_position = np.array(goal_position)
            if self.move_first_step:
                self.start_position, self.start_orientation = self._task._robot.get_world_pose()
                self.move_first_step = False
            current_position, current_orientation = self._task._robot.get_world_pose()
            print("current_position: ", current_position)
            # position = np.array(self._task._config.robot_init_pose)
            # orientation = euler_angles_to_quat(self._task._config.robot_init_ori)
            wheel_actions = self._robot_controller.forward(start_position=self.start_position,
                                                    start_orientation=self.start_orientation,
                                                    current_position=current_position,
                                                    current_orientation=current_orientation,
                                                    goal_position=goal_position,
                                                    lateral_velocity=self._task._config.lateral_velocity,
                                                    yaw_velocity=self._task._config.yaw_velocity,
                                                    position_tol=self._task._config.position_tol,
                                                    control_type="point-to-point"
                                                    )
            wheel_actions.joint_velocities = wheel_actions.joint_velocities
            self._task._robot.apply_wheel_actions(wheel_actions) 
            distance = np.sum((self._task._robot.get_world_pose()[0][:2] - goal_position[:2]) ** 2)  # Compute distance between robot and target
            print(f"distance to target: {distance} || min_distance: {min_distance}")
            if distance <= min_distance: # if done moving 
                print('DONE MOVING!!!')
                # self._task._task_event += 1
                self.move_first_step = True

                return False

            return True
    
    def rotate_to_location(self, goal_rotation: np.array, min_rotation_error: float = 3) -> bool:
        def normalize_angle(angle: float) -> float:
            return (angle + np.pi) % (2 * np.pi) - np.pi
        def normalize_angle_degree(angle: float) -> float:
            return (angle + 180) % (2 * 180) - 180

        position, orientation = self._task._robot.get_world_pose()
        _, local_orientation = self._task._robot.get_local_pose() # same with world pose
        current_angle = quat_to_euler_angles(orientation, degrees=True)

        # yaw_error = normalize_angle(current_angle[2] - goal_rotation[2])
        yaw_error = normalize_angle_degree(current_angle[2] - goal_rotation[2])
        if abs(yaw_error) > min_rotation_error:
            print("yaw_error:", yaw_error)
            wheel_actions = self._robot_controller.forward(
                                                    lateral_velocity=self._task._config.lateral_velocity,
                                                    control_type="only-rotate",
                                                    yaw_errror_for_ratate=yaw_error
                                                    )
            wheel_actions.joint_velocities = wheel_actions.joint_velocities
            self._task._robot.apply_wheel_actions(wheel_actions) 


        if abs(yaw_error) <= min_rotation_error: # if done rotating 
            print('FINISH ROATION IN CIRCLE')

            return False

        return True
    
    def pickup_object(self, press_detect: Optional[bool] = False, inside: Optional[bool] = False, button_position:Optional[np.array] = None) -> bool:
        """
        Picks up an object with the UR5 manipulator.

        Args:
            task: The task object containing the object to pick up.
            action_server: The action server, responsible for this task types.
                            Needed to send feedback and result back to planner.
        """
        stage = get_current_stage()
        #position, orientation = self._task._husky.get_world_pose()
        joints_state = self._task._robot.manipulator.get_joints_state()
        #end_effector_position, _ = husky.manipulator.end_effector.get_local_pose()
        joint_positions = joints_state.positions


        if self._pick_place_controller is None: 
            self._pick_place_controller = PickPlaceController(
                name="manipulator_controller",
                robot_articulation=self._task._robot.manipulator,
                gripper=self._task._robot.manipulator.gripper,
                config=self._task._config,
            )

            #self._pick_place_controller._cspace_controller.reset()
            print(self._pick_place_controller.__dict__)
        
        # TODO
        if self._pick_place_controller.pick_done() and not press_detect:
        #     self._task._task_event += 1
        #     self._pick_place_controller.pause()
            print("Press fail!!")
        #     self.press_first_step = True
            return False
        
        if self.press_first_step:
            self.start_position, self.start_orientation = self._task._robot.get_world_pose()
            self.press_first_step = False

            observations = self._task.get_observations()
            # picking_position = observations[self._task._object.name]['position']
            # target_position = observations[self._task._object.name]['target_position']

            ee_position = observations[self._task._robot.manipulator.name]['end_effector_position']
            self.init_ee_position_world = observations[self._task._robot.manipulator.name]['end_effector_world_position']
            print("Current ee pos:", self.init_ee_position_world)

            self.botton_position = button_position
            if inside:
                self.end_effector_orientation = euler_angles_to_quat(np.array([0, 0, np.pi/2]))
            else:
                self.end_effector_orientation = euler_angles_to_quat(np.array([0, 0, np.pi]))

            
        actions = self._pick_place_controller.forward(
            botton_position=self.botton_position,
            # init_position=ee_posotion, 
            init_position=self.init_ee_position_world,
            current_joint_positions=joint_positions,
            end_effector_offset=list(self._task._config.end_effector_offset),
            end_effector_orientation=self.end_effector_orientation, 
            prim_trans_point=self._prim_trans_point,
            press_detect=press_detect,
            inside=inside
        )

        self._articulation_controller.apply_action(actions)
        return True
    
    def put_object(self, object_position) -> bool:
        
        self.object_position = object_position

        stage = get_current_stage()


        if self._pick_place_controller.is_paused:
            #print(f'IS PAUSED: {self._pick_place_controller.is_paused()}')
            # end_effector_initial_height
            #old_h1 = self._pick_place_controller._h1


            self._pick_place_controller.resume()
            self._pick_place_controller._cspace_controller.reset()
            #self._pick_place_controller.reset()
            
            self._pick_place_controller.is_paused = False
            #self._pick_place_controller._h1 = old_h1
            print('RESUMED _pick_place_controller')

        joints_state = self._task._robot.manipulator.get_joints_state()
        joint_positions = joints_state.positions

        observations = self._task.get_observations()
        picking_position = observations[self._task._object.name]['position']
        target_position = observations[self._task._object.name]['target_position']
        end_effector_position = observations[self._task._robot.manipulator.name]['end_effector_position']

        # end_effector_orientation = (
        #     euler_angles_to_quat(np.array([0, np.pi, 0.5 * np.pi]))
        #     if obj_loc_str == "table"
        #     else euler_angles_to_quat(np.array([0, np.pi, 0.8 * np.pi]))
        # )
        end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi/2, 0]))
        actions = self._pick_place_controller.forward(
            picking_position=picking_position,
            placing_position=target_position,
            current_joint_positions=joint_positions,
            end_effector_offset=np.array(self._task._config.end_effector_offset),
            end_effector_orientation=end_effector_orientation,
            prim_trans_point=self._prim_trans_point,
            object_position = object_position,
        )
        self._articulation_controller.apply_action(actions)
        #print(self._pick_place_controller._event)
        if self._pick_place_controller.is_done():

            self._task._task_event += 1
            self._pick_place_controller.reset()

            print("PLACING_DONE!")
            return False

        return True