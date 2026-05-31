import numpy as np
from scipy.interpolate import CubicSpline


class TrajectoryPlanner:
    """
    Handles path planning and trajectory generation.
    """

    @staticmethod
    def linear_interpolate(
        start: np.ndarray, end: np.ndarray, steps: int
    ) -> np.ndarray:
        """
        Generates a linear trajectory between two points.

        Args:
            start: Starting position/angles.
            end: Ending position/angles.
            steps: Number of points in the trajectory.

        Returns:
            np.ndarray: steps x N array of points.
        """
        return np.linspace(start, end, steps)

    @staticmethod
    def cubic_interpolate(waypoints: np.ndarray, steps: int) -> np.ndarray:
        """
        Generates a smooth cubic spline trajectory through waypoints.

        Args:
            waypoints: N x M array of waypoints.
            steps: Total number of points in the trajectory.

        Returns:
            np.ndarray: steps x M array of points.
        """
        n_waypoints = len(waypoints)
        t = np.linspace(0, 1, n_waypoints)
        cs = CubicSpline(t, waypoints, axis=0)

        t_new = np.linspace(0, 1, steps)
        return cs(t_new)
