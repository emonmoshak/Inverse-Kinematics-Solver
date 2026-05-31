import numpy as np

from src.kinematics.forward import ForwardKinematics
from src.robot.model import RobotModel


class WorkspaceGenerator:
    """
    Generates the reachable workspace of the robot arm.
    """

    def __init__(self, robot: RobotModel) -> None:
        self.robot = robot
        self.fk = ForwardKinematics()

    def generate(self, samples_per_joint: int = 15) -> np.ndarray:
        """
        Generates a cloud of reachable points.

        Args:
            samples_per_joint: Number of samples for each joint.

        Returns:
            np.ndarray: N x 3 array of reachable [x, y, z] points.
        """
        q1_range = np.linspace(
            self.robot.joint_limits[0].min_angle,
            self.robot.joint_limits[0].max_angle,
            samples_per_joint,
        )
        q2_range = np.linspace(
            self.robot.joint_limits[1].min_angle,
            self.robot.joint_limits[1].max_angle,
            samples_per_joint,
        )
        q3_range = np.linspace(
            self.robot.joint_limits[2].min_angle,
            self.robot.joint_limits[2].max_angle,
            samples_per_joint,
        )

        points = []
        for q1 in q1_range:
            for q2 in q2_range:
                for q3 in q3_range:
                    pos = self.fk.solve(self.robot, np.array([q1, q2, q3]))
                    points.append(pos)

        return np.array(points)

    def get_bounds(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Returns the min and max bounds of the workspace."""
        return np.min(points, axis=0), np.max(points, axis=0)
