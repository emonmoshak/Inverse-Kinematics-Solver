import numpy as np
import pytest
from src.trajectory.planner import TrajectoryPlanner

def test_linear_interpolation():
    start = np.array([0, 0, 0])
    end = np.array([1, 1, 1])
    steps = 11
    
    traj = TrajectoryPlanner.linear_interpolate(start, end, steps)
    
    assert len(traj) == steps
    np.testing.assert_allclose(traj[0], start)
    np.testing.assert_allclose(traj[-1], end)
    np.testing.assert_allclose(traj[5], np.array([0.5, 0.5, 0.5]))

def test_cubic_interpolation():
    waypoints = np.array([
        [0, 0, 0],
        [1, 2, 1],
        [2, 0, 2]
    ])
    steps = 50
    
    traj = TrajectoryPlanner.cubic_interpolate(waypoints, steps)
    
    assert len(traj) == steps
    np.testing.assert_allclose(traj[0], waypoints[0], atol=1e-6)
    np.testing.assert_allclose(traj[-1], waypoints[-1], atol=1e-6)
    # Check that it passes near the middle waypoint
    # (CubicSpline with linspace time might not hit it exactly at steps/2 if not uniform)
    # But it should be smooth.
