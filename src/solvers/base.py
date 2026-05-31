from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class IKSolution:
    """Represents a solution from an IK solver."""

    joint_angles: np.ndarray
    success: bool
    error: float = 0.0
    iterations: int = 0
    message: str = ""


class IKSolver(ABC):
    """Base class for Inverse Kinematics solvers."""

    @abstractmethod
    def solve(self, target_pos: np.ndarray, **kwargs) -> list[IKSolution]:
        """
        Solves the inverse kinematics problem.

        Args:
            target_pos: [x, y, z] target position.

        Returns:
            List[IKSolution]: List of possible solutions.
        """
        pass
