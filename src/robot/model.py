from dataclasses import dataclass

import numpy as np


@dataclass
class JointLimits:
    """Joint limits for the robot arm."""

    min_angle: float
    max_angle: float


EXPECTED_LINKS = 3


class RobotModel:
    """
    Represents a 3-DOF robotic arm model.

    Attributes:
        link_lengths (List[float]): Lengths of the three links [L1, L2, L3].
        joint_limits (List[JointLimits]): Min/max angles for each joint.
    """

    def __init__(
        self, link_lengths: list[float], joint_limits: list[JointLimits] = None
    ) -> None:
        """
        Initializes the RobotModel.

        Args:
            link_lengths: List of three link lengths.
            joint_limits: Optional list of joint limits. Defaults to [-pi, pi] for all.
        """
        if len(link_lengths) != EXPECTED_LINKS:
            raise ValueError(
                f"RobotModel requires exactly {EXPECTED_LINKS} link lengths."
            )

        self.link_lengths = np.array(link_lengths, dtype=float)

        if joint_limits is None:
            self.joint_limits = [
                JointLimits(-np.pi, np.pi),
                JointLimits(-np.pi, np.pi),
                JointLimits(-np.pi, np.pi),
            ]
        else:
            if len(joint_limits) != EXPECTED_LINKS:
                raise ValueError(
                    f"RobotModel requires exactly {EXPECTED_LINKS} joint limits."
                )
            self.joint_limits = joint_limits

    def get_dh_parameters(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Returns DH parameters for the given joint angles.

        Table format: [theta, d, a, alpha]

        Args:
            joint_angles: Array of 3 joint angles [q1, q2, q3].

        Returns:
            np.ndarray: DH parameter table.
        """
        L1, L2, L3 = self.link_lengths
        q1, q2, q3 = joint_angles

        # Standard DH table for a 3-DOF arm
        # Link 1: Base rotation (Z), offset L1, length 0, twist pi/2
        # Link 2: Shoulder rotation (Y), offset 0, length L2, twist 0
        # Link 3: Elbow rotation (Y), offset 0, length L3, twist 0
        return np.array([[q1, L1, 0, np.pi / 2], [q2, 0, L2, 0], [q3, 0, L3, 0]])
