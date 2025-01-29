from configs.main_config import MainConfig
#from configs.pickplace_config import PickPlaceConfig
# from omni.isaac.robot_composer import RobotComposer
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from omni.isaac.core.utils.prims import get_prim_at_path, is_prim_path_valid
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.utils.prims import get_prim_at_path, define_prim
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.types import ArticulationAction


import numpy as np

from typing import List

# enable_extension('omni.isaac.robot_composer')

import os

class MobileManipulatorRobot(Robot):
    """[summary]

    Args:
        prim_path (str): [description]
        robot_path (str): relative path from prim path to the robot.
        name (str): [description]
        wheel_dof_names ([str, str]): name of the wheels, [left,right].
        wheel_dof_indices: ([int, int]): indices of the wheels, [left, right]
        usd_path (str, optional): [description]
        create_robot (bool): create robot at prim_path if no robot exist at said path. Defaults to False
        position (Optional[np.ndarray], optional): [description]. Defaults to None.
        orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self, 
        config: MainConfig, 
        robot_name: str, 
        manipulator_name: str, 
        robot_init_pos: List,
        robot_init_ori: List,
    ) -> None:

        self._config = config
        self.robot_init_pos = robot_init_pos

        # for parallels simulations  
        self.robot_prim_path = find_unique_string_name(
            initial_name=self._config.robot_stage_path, 
            is_unique_fn=lambda x: not is_prim_path_valid(x))

        usd_path = os.path.abspath(self._config.robot_usd_path)
        prim = get_prim_at_path(self.robot_prim_path)
        
        if not prim.IsValid():
            prim = define_prim(self.robot_prim_path, "Xform")
            if usd_path:
                prim.GetReferences().AddReference(usd_path)
            else:
                carb.log_error("no valid usd path defined to create new robot")

        super().__init__(
            prim_path=self.robot_prim_path, name=robot_name, position=robot_init_pos, 
            orientation=robot_init_ori,
            articulation_controller=None
        )
        self._wheel_dof_names = ["left_wheel", "right_wheel"]
        self._wheel_dof_indices = None

        # manipulator

        add_reference_to_stage(usd_path=usd_path, prim_path="/World/aloha")

        self.gripper = ParallelGripper(
            end_effector_prim_path=self.robot_prim_path + "/fl_link6",
            joint_prim_names=["fl_joint7", "fl_joint8"],
            joint_opened_positions=np.array(self._config.joint_opened_positions),
            joint_closed_positions=np.array(self._config.joint_closed_positions),
            #action_deltas=np.array([0.02, 0.02]),
        )

        #    joint_opened_positions=np.array([-1,-1]),
        #    joint_closed_positions=np.array([1, 1]),
        #    action_deltas=np.array([0.35, 0.35]),

        self.manipulator = SingleManipulator(
                        prim_path=self.robot_prim_path, 
                        name="aloha_manipulator",
                        end_effector_prim_name="fl_link6",
                        gripper=self.gripper)

    @property
    def wheel_dof_indices(self):
        """[summary]

        Returns:
            int: [description]
        """
        return self._wheel_dof_indices

    def get_wheel_positions(self):
        """[summary]

        Returns:
            Tuple[float, float]: [description]
        """
        full_dofs_positions = self.get_joint_positions()
        wheel_joint_positions = [full_dofs_positions[i] for i in self._wheeled_dof_indices]
        return wheel_joint_positions

    def set_wheel_positions(self, positions) -> None:
        """[summary]

        Args:
            positions (Tuple[float, float]): [description]
        """
        full_dofs_positions = [None] * self.num_dof
        for i in range(self._num_wheel_dof):
            full_dofs_positions[self._wheel_dof_indices[i]] = positions[i]
        self.set_joint_positions(positions=np.array(full_dofs_positions))
        return

    def get_wheel_velocities(self):
        """[summary]

        Returns:
            Tuple[np.ndarray, np.ndarray]: [description]
        """

        full_dofs_velocities = self.get_joint_velocities()
        wheel_dof_velocities = [full_dofs_velocities[i] for i in self._wheel_dof_indicies]
        return wheel_dof_velocities

    def set_wheel_velocities(self, velocities) -> None:
        """[summary]

        Args:
            velocities (Tuple[float, float]): [description]
        """
        full_dofs_velocities = [None] * self.num_dof
        for i in range(self._num_wheel_dof):
            full_dofs_velocities[self._wheel_dof_indices[i]] = velocities[i]
        self.set_joint_velocities(velocities=np.array(full_dofs_velocities))
        return

    def apply_wheel_actions(self, actions: ArticulationAction) -> None:
        """applying action to the wheels to move the robot

        Args:
            actions (ArticulationAction): [description]
        """
        actions_length = actions.get_length()
        if actions_length is not None and actions_length != self._num_wheel_dof:
            raise Exception("ArticulationAction passed should be the same length as the number of wheels")
        joint_actions = ArticulationAction()
        if actions.joint_positions is not None:
            joint_actions.joint_positions = np.zeros(self.num_dof)  # for all dofs of the robot
            for i in range(self._num_wheel_dof):  # set only the ones that are the wheels
                joint_actions.joint_positions[self._wheel_dof_indices[i]] = actions.joint_positions[i]
        if actions.joint_velocities is not None:
            joint_actions.joint_velocities = np.zeros(self.num_dof)
            for i in range(self._num_wheel_dof):
                joint_actions.joint_velocities[self._wheel_dof_indices[i]] = actions.joint_velocities[i]
        if actions.joint_efforts is not None:
            joint_actions.joint_efforts = np.zeros(self.num_dof)
            for i in range(self._num_wheel_dof):
                joint_actions.joint_efforts[self._wheel_dof_indices[i]] = actions.joint_efforts[i]
        self.apply_action(control_actions=joint_actions)
        return

    def apply_joint_actions(self, actions: ArticulationAction) -> None:
        actions_length = actions.get_length()
        if actions_length is not None and actions_length != self._num_wheel_dof:
            raise Exception("ArticulationAction passed should be the same length as the number of wheels")
        joint_actions = ArticulationAction()
        if actions.joint_positions is not None:
            joint_actions.joint_positions = np.zeros(self.num_dof)  # for all dofs of the robot
            for i in range(self._num_wheel_dof):  # set only the ones that are the wheels
                joint_actions.joint_positions[self._wheel_dof_indices[i]] = actions.joint_positions[i]
        if actions.joint_velocities is not None:
            joint_actions.joint_velocities = np.zeros(self.num_dof)
            for i in range(self._num_wheel_dof):
                joint_actions.joint_velocities[self._wheel_dof_indices[i]] = actions.joint_velocities[i]
        if actions.joint_efforts is not None:
            joint_actions.joint_efforts = np.zeros(self.num_dof)
            for i in range(self._num_wheel_dof):
                joint_actions.joint_efforts[self._wheel_dof_indices[i]] = actions.joint_efforts[i]
        self.apply_action(control_actions=joint_actions)
        return

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]"""
        super().initialize(physics_sim_view=physics_sim_view)
        if self._wheel_dof_names is not None:
            self._wheel_dof_indices = [
                self.get_dof_index(self._wheel_dof_names[i]) for i in range(len(self._wheel_dof_names))
            ]
            
        elif self._wheel_dof_indices is None:
            carb.log_error("need to have either wheel names or wheel indices")

        self._num_wheel_dof = len(self._wheel_dof_indices)

        return

    def post_reset(self) -> None:
        """[summary]"""
        super().post_reset()
        # self._articulation_controller.switch_control_mode(mode="velocity")

        self._articulation_controller.switch_dof_control_mode(dof_index=self._wheel_dof_indices[0], mode="velocity")
        self._articulation_controller.switch_dof_control_mode(dof_index=self._wheel_dof_indices[1], mode="velocity")

        return

    def get_articulation_controller_properties(self):
        return self._wheel_dof_names, self._wheel_dof_indices


