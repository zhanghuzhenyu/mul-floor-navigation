# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.isaac.motion_generation as mg
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.extensions import get_extension_path_from_name
import os


class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name: str, robot_articulation: Articulation, physics_dt: float = 1.0 / 60.0) -> None:
        self.rmpflow = mg.lula.motion_policies.RmpFlow(
            # self.rmpflow = RmpFlow(
            robot_description_path=os.path.join(os.path.dirname(__file__), "./aloha_rmpflow_cspace.yaml"),
            rmpflow_config_path=os.path.join(os.path.dirname(__file__), "./aloha_rmpflow.yaml"),
            urdf_path=os.path.join(os.path.dirname(__file__), "./arx5_description_isaac.urdf"),
            end_effector_frame_name="fl_link6",
            # end_effector_frame_name="gripper_center", # /arx5_description/fl_link6/gripper_center
            maximum_substep_size=0.00334,
        )
        # print("Init success!\n")
        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmpflow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        (
            self._default_position,
            self._default_orientation,
        ) = self._articulation_motion_policy._robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(
           robot_position=self._default_position, robot_orientation=self._default_orientation
        )

        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        (
            self._default_position,
            self._default_orientation,
        ) = self._articulation_motion_policy._robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        print("RMPFlow reset success!\n")

    # def reset(self):
    #     mg.MotionPolicyController.reset(self)
    #     self._motion_policy.set_robot_base_pose(
    #         robot_position=self._default_position, robot_orientation=self._default_orientation
    #     )
