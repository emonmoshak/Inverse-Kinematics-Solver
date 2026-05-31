import numpy as np

from src.robot.model import RobotModel


class ForwardKinematics:
    """
    Handles Forward Kinematics calculations using DH Parameters.
    """

    @staticmethod
    def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """
        Computes the 4x4 DH transformation matrix.

        Args:
            theta: Joint angle.
            d: Link offset.
            a: Link length.
            alpha: Link twist.

        Returns:
            np.ndarray: 4x4 transformation matrix.
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        return np.array(
            [
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0, sa, ca, d],
                [0, 0, 0, 1],
            ]
        )

    def solve(self, robot: RobotModel, joint_angles: np.ndarray) -> np.ndarray:
        """
        Computes the end-effector position from joint angles.

        Args:
            robot: The robot model instance.
            joint_angles: Array of 3 joint angles.

        Returns:
            np.ndarray: [x, y, z] coordinates of the end-effector.
        """
        dh_params = robot.get_dh_parameters(joint_angles)
        t_total = np.eye(4)

        for params in dh_params:
            t_total = t_total @ self.dh_transform(*params)

        return t_total[:3, 3]

    def get_all_joint_positions(
        self, robot: RobotModel, joint_angles: np.ndarray
    ) -> np.ndarray:
        """
        Computes the 3D positions of all joints.

        Args:
            robot: The robot model instance.
            joint_angles: Array of 3 joint angles.

        Returns:
            np.ndarray: N x 3 array of joint positions (including base [0,0,0]).
        """
        dh_params = robot.get_dh_parameters(joint_angles)
        positions = [np.array([0, 0, 0])]
        t_current = np.eye(4)

        for params in dh_params:
            t_current = t_current @ self.dh_transform(*params)
            positions.append(t_current[:3, 3])

        return np.array(positions)
