from typing import Any

import numpy as np

from src.robot.model import RobotModel
from src.solvers.base import IKSolution, IKSolver


class AnalyticalIKSolver(IKSolver):
    """
    Analytical IK solver using a geometric approach for a 3-DOF arm.
    """

    def __init__(self, robot: RobotModel) -> None:
        self.robot = robot

    def solve(
        self, target_pos: np.ndarray, elbow_up: bool = True, **kwargs: Any
    ) -> list[IKSolution]:
        """
        Solves IK analytically.

        Args:
            target_pos: [x, y, z] target position.
            elbow_up: Whether to prefer the elbow-up solution.

        Returns:
            List[IKSolution]: List of joint angles.
        """
        x, y, z = target_pos
        L1, L2, L3 = self.robot.link_lengths

        # 1. Base rotation
        q1 = np.arctan2(y, x)

        # 2. Reduce to 2D
        r = np.sqrt(x**2 + y**2)
        z_prime = z - L1

        # Distance from shoulder to end-effector
        d_sq = r**2 + z_prime**2
        d = np.sqrt(d_sq)

        # Check reachability
        if d > (L2 + L3) or d < abs(L2 - L3):
            return [IKSolution(np.zeros(3), False, message="Target out of reach")]

        # 3. Use Law of Cosines for q3
        cos_q3 = (d_sq - L2**2 - L3**2) / (2 * L2 * L3)
        # Numerical stability check
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)

        q3_mag = np.arccos(cos_q3)
        q3 = -q3_mag if elbow_up else q3_mag

        # 4. Solve for q2
        alpha = np.arctan2(z_prime, r)
        beta = np.arctan2(L3 * np.sin(q3), L2 + L3 * np.cos(q3))

        # Correct for coordinate system (DH)
        # In our DH, q2=0 is horizontal.
        # But atan2(z_prime, r) gives angle from horizontal.
        # beta is angle between L2 and d.
        q2 = alpha - beta

        solutions = [IKSolution(np.array([q1, q2, q3]), True)]

        # Also return the other branch if requested
        # (but here we just return the preferred one)
        return solutions
