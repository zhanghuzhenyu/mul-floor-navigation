# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import typing

import numpy as np
from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.manipulators.grippers.gripper import Gripper

try:
    from omni.usd.utils import get_world_transform_matrix
except:  # noqa E722
    from omni.usd import get_world_transform_matrix

from configs.main_config import MainConfig

class MPickPlaceController:
    """
    A simple pick and place state machine for tutorials

    Each phase runs for 1 second, which is the internal time of the state machine

    Dt of each phase/ event step is defined

    - Phase 0: Move end_effector above the cube center at the 'end_effector_initial_height'.
    - Phase 0.5: open gripper
    - Phase 1: Lower end_effector down to encircle the target cube
    - Phase 2: Wait for Robot's inertia to settle.
    - Phase 3: close grip.
    - Phase 4: Move end_effector up again, keeping the grip tight (lifting the block).

    - Phase 5: Smoothly move the end_effector toward the goal xy, keeping the height constant.
    - Phase 6: Move end_effector vertically toward goal height at the 'end_effector_initial_height'.
    - Phase 7: loosen the grip.
    - Phase 8: Move end_effector vertically up again at the 'end_effector_initial_height'
    - Phase 9: Move end_effector towards the old xy position.

    Args:
        name (str): Name id of the controller
        cspace_controller (BaseController): a cartesian space controller
        that returns an ArticulationAction type
        gripper (Gripper): a gripper controller for open/ close actions.
        end_effector_initial_height (typing.Optional[float], optional):
        end effector initial picking height to start from (more info in phases above).
        If not defined, set to 0.3 meters. Defaults to None.
        events_dt (typing.Optional[typing.List[float]], optional):
        Dt of each phase/ event step. 10 phases dt has to be defined. Defaults to None.

    Raises:
        Exception: events dt need to be list or numpy array
        Exception: events dt need have length of 10
    """

    def __init__(
        self,
        name: str,
        cspace_controller: BaseController,
        gripper: Gripper,
        end_effector_initial_height: typing.Optional[float] = None,
        events_dt: typing.Optional[typing.List[float]] = None,
        ur5_init_pose: list = None,
        config: MainConfig = None,
    ) -> None:
        BaseController.__init__(self, name=name)
        self._config = config
        self._event = 0
        self._t = 0
        self._h1 = end_effector_initial_height
        if self._h1 is None:
            self._h1 = 0.3 / get_stage_units()
        self._h0 = None
        self._events_dt = events_dt
        if self._events_dt is None:
            self._events_dt = [0.008, 0.005, 0.1, 0.1, 0.0025, 0.001, 0.0025, 1, 0.008, 0.08]
        else:
            if not isinstance(self._events_dt, np.ndarray) and not isinstance(self._events_dt, list):
                raise Exception("events dt need to be list or numpy array")
            elif isinstance(self._events_dt, np.ndarray):
                self._events_dt = self._events_dt.tolist()
            if len(self._events_dt) > 10:
                raise Exception("events dt length must be less than 10")
        self._cspace_controller = cspace_controller
        self._gripper = gripper
        self._pause = False
        self.flag = False
        self._ur5_init_pose = ur5_init_pose
        self._trans_height = 0.6
        self.ppose = None
        self._t_press = 0
        self.back_to_init_is_done = False
        return

    def is_paused(self) -> bool:
        """

        Returns:
            bool: True if the state machine is paused. Otherwise False.
        """
        return self._pause

    def get_current_event(self) -> int:
        """

        Returns:
            int: Current event/ phase of the state machine
        """
        return self._event

    def forward(
        self,
        botton_position: np.ndarray,
        init_position: np.ndarray,
        # placing_position: np.ndarray,
        current_joint_positions: np.ndarray,
        end_effector_offset: typing.Optional[np.ndarray] = None,
        end_effector_orientation: typing.Optional[np.ndarray] = None,
        prim_trans_point=None,
        object_position=None,
        press_detect=None,
        inside=None
    ) -> ArticulationAction:
        """Runs the controller one step.

        Args:
            picking_position (np.ndarray): The object's position to be picked in local frame.
            placing_position (np.ndarray):  The object's position to be placed in local frame.
            current_joint_positions (np.ndarray): Current joint positions of the robot.
            end_effector_offset (typing.Optional[np.ndarray], optional):
            offset of the end effector target. Defaults to None.
            end_effector_orientation (typing.Optional[np.ndarray], optional):
            end effector orientation while picking and placing. Defaults to None.

        Returns:
            ArticulationAction: action to be executed by the ArticulationController
        """
        print(f"Now event: {self._event}")
        if end_effector_offset is None:
            end_effector_offset = np.array([0, 0, 0])
        if self._pause or self.is_done():
            self.pause()
            target_joint_positions = [None] * current_joint_positions.shape[0]
            return ArticulationAction(joint_positions=target_joint_positions)
        if self._event == 0 and not press_detect:
            if not inside:
                self.pre_press = np.array(
                    [
                        botton_position[0] + 0.25,
                        botton_position[1],
                        botton_position[2]
                    ]
                )
            else:
                self.pre_press = np.array(
                    [
                        botton_position[0],
                        botton_position[1] - 0.25,
                        botton_position[2]
                    ]
                )
            position_target = self.pre_press
            print(f"position_target: {position_target}\n")
            if end_effector_orientation is None:
                end_effector_orientation = euler_angles_to_quat(np.array(self._config.end_effector_orientation))
            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=position_target,
                target_end_effector_orientation=end_effector_orientation,
            )


        elif self._event == 1 and not press_detect:
            if not inside:
                target_y = botton_position[1]
                target_height = botton_position[2]
                alpha = self._mix_sin(self._t)
                target_x = (1 - alpha) * self.pre_press[0] + alpha * botton_position[0] 
                position_target = np.array([target_x, target_y, target_height])
                print(f"position_target: {position_target}\n")
            else:
                target_x = botton_position[0]
                target_height = botton_position[2]
                alpha = self._mix_sin(self._t)
                target_y = (1 - alpha) * self.pre_press[1] + alpha * botton_position[1] 
                position_target = np.array([target_x, target_y, target_height])
                print(f"position_target: {position_target}\n")  
            if end_effector_orientation is None:
                end_effector_orientation = euler_angles_to_quat(np.array(self._config.end_effector_orientation))

            target_joint_positions = self._cspace_controller.forward(
                target_end_effector_position=position_target,
                target_end_effector_orientation=end_effector_orientation,
            )
        elif press_detect:
            init_pose= np.zeros(42)
            interpolated_positions = self.interpolate_positions(current_joint_positions, init_pose, self._t_press)
            # interpolated_positions = np.append(interpolated_positions, [None, None, None, None, None, None])
            target_joint_positions = ArticulationAction(joint_positions= interpolated_positions,)
            self._t_press += 0.03
            print('self._t_press: ', self._t_press)
        
        if not press_detect: 
            self._t += self._events_dt[int(self._event)]
            if self._t >= 1.0: 
                self._event += 1
                self._t = 0

        if self._t_press >= 1.0:
            # self._event += 1
            self._t_press = 0
            self.back_to_init_is_done = True

        print(f"t = {self._t}")
        # print(f" self._events_dt: { self._events_dt}\n")

        return target_joint_positions

    def _get_interpolated_xy(self, target_x, target_y, current_x, current_y):
        alpha = self._get_alpha()
        # print(f"alpha={alpha}")
        xy_target = (1 - alpha) * np.array([current_x, current_y]) + alpha * np.array([target_x, target_y])
        return xy_target

    def _get_alpha(self):
        if self._event == 0:
            return self._mix_sin(self._t)
        elif self._event == 1:
            return self._mix_sin(self._t)
        else:
            raise ValueError()


    def _get_target_hs(self, target_height, init_height):
        if self._event == 0: 
            a = self._mix_sin(max(0, self._t))
            h = self._combine_convex(init_height, target_height, a)
        elif self._event == 1:
            h = init_height
        return h

    def interpolate_positions(self, current_joint_positions, joints_default_positions, t):

        target_indices = [6, 14, 18, 22, 26, 30, 34, 35] # index of fl_joint1 to fl_joint8
        
        interpolated_positions = np.copy(current_joint_positions)

        alpha = self._mix_sin(t)

        interpolated_positions[target_indices] = (
            (1 - alpha) * current_joint_positions[target_indices] +
            alpha * joints_default_positions[target_indices]
        )

        return interpolated_positions
    
    def _mix_sin(self, t):
        return 0.5 * (1 - np.cos(t * np.pi))

    def _combine_convex(self, a, b, alpha):
        return (1 - alpha) * a + alpha * b

    def reset(
        self,
        end_effector_initial_height: typing.Optional[float] = None,
        events_dt: typing.Optional[typing.List[float]] = None,
    ) -> None:
        """Resets the state machine to start from the first phase/ event

        Args:
            end_effector_initial_height (typing.Optional[float], optional):
            end effector initial picking height to start from.
            If not defined, set to 0.3 meters. Defaults to None.
            events_dt (typing.Optional[typing.List[float]], optional):
            Dt of each phase/ event step. 10 phases dt has to be defined. Defaults to None.

        Raises:
            Exception: events dt need to be list or numpy array
            Exception: events dt need have length of 10
        """
        BaseController.reset(self)
        self._cspace_controller.reset()
        self._event = 0
        self._t = 0
        if end_effector_initial_height is not None:
            self._h1 = end_effector_initial_height
        self._pause = False
        if events_dt is not None:
            self._events_dt = events_dt
            if not isinstance(self._events_dt, np.ndarray) and not isinstance(self._events_dt, list):
                raise Exception("event velocities need to be list or numpy array")
            elif isinstance(self._events_dt, np.ndarray):
                self._events_dt = self._events_dt.tolist()
            if len(self._events_dt) > 10:
                raise Exception("events dt length must be less than 10")
        return

    def is_done(self) -> bool:
        """
        Returns:
            bool: True if the state machine reached the last phase. Otherwise False.
        """
        if self._event >= len(self._events_dt) or self.back_to_init_is_done:
            return True
        else:
            return False

    def pause(self) -> None:
        """Pauses the state machine's time and phase."""
        self._pause = True
        return

    def resume(self) -> None:
        """Resumes the state machine's time and phase."""
        self._pause = False
        return
    