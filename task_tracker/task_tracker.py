import time
import os
import json
import numpy as np

class TaskTracker:
    def __init__(self, actions, targets, save_dir, config):
        """
        Initialize the TaskTracker with actions, targets, and a save directory.

        Parameters:
            actions (list): A list of actions to perform (e.g., ['Move to', 'Press']).
            targets (list): A list of corresponding targets (e.g., ['3rd floor', 'down button']).
            save_dir (str): The directory where task records will be saved.
        """
        if len(actions) != len(targets):
            raise ValueError("Actions and targets must have the same length!")
        
        self.actions = actions
        self.targets = targets
        self.index = 0  # Start at the first task
        self.results = [None] * len(actions)  # Track success/failure of each task
        self.timestamps = [None] * len(actions)  # Track completion time of each task
        self.trajectory_points = []  # Initialize an empty list to track trajectory points
        
        # Set up save directory and file name
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.save_path = os.path.join(save_dir, f"task_records_{timestamp}.json")
        self.config = config

    def convert_ndarray_to_list(self, d):

        if isinstance(d, dict):
            return {key: self.convert_ndarray_to_list(value) for key, value in d.items()}
        elif isinstance(d, list):
            return [self.convert_ndarray_to_list(item) for item in d]
        elif isinstance(d, np.ndarray):
            return d.tolist()
        else:
            return d
        
    def current_task(self):
        """
        Get the current action and target.

        Returns:
            dict: The current task, including action and target.
        """
        if self.index < len(self.actions):
            return {
                "action": self.actions[self.index],
                "target": self.targets[self.index],
                "index": self.index
            }
        return None  # No more tasks

    def mark_task_complete(self, success=True, timestamps=None, dis_to_goal=None):
        """
        Mark the current task as complete and record its success/failure.

        Parameters:
            success (bool): Whether the task was completed successfully. Default is True.
            timestamps (float): The timestamp when the task was completed. Default is current time.
            dis_to_goal (float or None): The distance to the goal. If None, it will not be recorded.

        Returns:
            bool: True if successfully moved to the next task, False if no more tasks.
        """
        if timestamps is None:
            timestamps = time.time()

        if self.index < len(self.actions):
            self.results[self.index] = success
            self.timestamps[self.index] = timestamps  # Record the current timestamp
            status = "success" if success else "failure"
            print(f"Task completed: '{self.actions[self.index]}' <{self.targets[self.index]}> with {status} at {self.timestamps[self.index]}.")

            # Save the current task result to file
            self._save_task_record(self.index, success, timestamps, dis_to_goal)

            self.index += 1
            return True
        print("All tasks are already completed!")
        return False

    def _save_task_record(self, index, success, timestamp, dis_to_goal):
        """
        Save the current task result to a JSON file.

        Parameters:
            index (int): The index of the current task.
            success (bool): Whether the task was completed successfully.
            timestamp (float): The timestamp of task completion.
            dis_to_goal (float or None): The distance to the goal, if available.
        """
        # Convert trajectory points
        self.trajectory_points = self.convert_ndarray_to_list(self.trajectory_points)
        
        task_record = {
            "action": self.actions[index],
            "target": self.targets[index],
            "result": "success" if success else "failure",
            "timestamp": timestamp,
            "robot_init_pose": self.config.robot_init_pose,  # Add robot initial pose
            "robot_init_ori": self.config.robot_init_ori,    # Add robot initial orientation
            "trajectory_points": self.trajectory_points      # Add trajectory points
        }

        # If dis_to_goal is provided, add it to the task record
        if dis_to_goal is not None:
            task_record["dis_to_goal"] = dis_to_goal
        
        # Save to the file
        if os.path.exists(self.save_path):
            with open(self.save_path, "r") as file:
                data = json.load(file)
        else:
            data = []

        data.append(task_record)

        with open(self.save_path, "w") as file:
            json.dump(data, file, indent=4)

    def remaining_tasks(self):
        """
        Get a list of remaining tasks.

        Returns:
            list: A list of dictionaries representing the remaining tasks.
        """
        return [
            {"action": self.actions[i], "target": self.targets[i], "index": i}
            for i in range(self.index, len(self.actions))
        ]

    def is_complete(self):
        """
        Check if all tasks are completed.

        Returns:
            bool: True if all tasks are completed, False otherwise.
        """
        return self.index >= len(self.actions)

    def task_results(self):
        """
        Get the results of all tasks (success/failure) along with their timestamps.

        Returns:
            list: A list of dictionaries with action, target, result (success/failure), and timestamp.
        """
        return [
            {
                "action": self.actions[i],
                "target": self.targets[i],
                "result": self.results[i],
                "timestamp": self.timestamps[i]
            }
            for i in range(len(self.actions))
        ]

    def add_trajectory_point(self, point):
        """
        Add a new trajectory point to the trajectory list.

        Parameters:
            point (tuple): The new trajectory point to add (e.g., (x, y, z)).
        """
        self.trajectory_points.append(point)
        print(f"Added new trajectory point: {point}")
