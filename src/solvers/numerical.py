from typing import Any

import numpy as np

from src.kinematics.forward import ForwardKinematics
from src.robot.model import RobotModel
from src.solvers.base import IKSolution, IKSolver


class NumericalIKSolver(IKSolver):
    """
    Numerical IK solver using Jacobian-based methods.
    """

    def __init__(self, robot: RobotModel) -> None:
        self.robot = robot
        self.fk = ForwardKinematics()

    def get_jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Computes the Jacobian matrix for the robot arm (3x3 for position).

        Args:
            joint_angles: Current joint angles.

        Returns:
            np.ndarray: 3x3 Jacobian matrix.
        """
        # Numerical Jacobian calculation (central difference)
        eps = 1e-6
        jacobian = np.zeros((3, 3))

        curr_pos = self.fk.solve(self.robot, joint_angles)

        for i in range(3):
            angles_plus = joint_angles.copy()
            angles_plus[i] += eps
            pos_plus = self.fk.solve(self.robot, angles_plus)

            jacobian[:, i] = (pos_plus - curr_pos) / eps

        return jacobian

    def solve(
        self,
        target_pos: np.ndarray,
        initial_guess: np.ndarray | None = None,
        method: str = "pseudoinverse",
        max_iterations: int = 100,
        tolerance: float = 1e-4,
        learning_rate: float = 0.5,
        **kwargs: Any,
    ) -> list[IKSolution]:
        """
        Solves IK numerically.

        Args:
            target_pos: [x, y, z] target position.
            initial_guess: Starting joint angles.
            method: "pseudoinverse" or "transpose".
            max_iterations: Max iteration count.
            tolerance: Convergence threshold.
            learning_rate: Step size.

        Returns:
            List[IKSolution]: Resulting solution.
        """
        if initial_guess is None:
            q = np.zeros(3)
        else:
            q = initial_guess.copy()

        err_norm = 0.0
        for i in range(max_iterations):
            curr_pos = self.fk.solve(self.robot, q)
            error = target_pos - curr_pos
            err_norm = float(np.linalg.norm(error))

            if err_norm < tolerance:
                return [IKSolution(q, True, err_norm, i, "Converged")]

            J = self.get_jacobian(q)

            if method == "pseudoinverse":
                # Pinverse with damping (Levenberg-Marquardt style)
                damping = 1e-3
                dq = np.linalg.pinv(J.T @ J + damping * np.eye(3)) @ J.T @ error
            elif method == "transpose":
                dq = J.T @ error * learning_rate
            else:
                raise ValueError(f"Unknown method: {method}")

            q += dq
            # Wrap angles to [-pi, pi] to keep them in a physically meaningful range
            q = (q + np.pi) % (2 * np.pi) - np.pi

        return [
            IKSolution(
                q, False, float(err_norm), max_iterations, "Max iterations reached"
            )
        ]

    def get_determinant(self, joint_angles: np.ndarray) -> float:
        """
        Returns the determinant of the Jacobian (singularity check).
        """
        J = self.get_jacobian(joint_angles)
        return float(np.linalg.det(J))
