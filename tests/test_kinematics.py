import numpy as np
import pytest
from src.robot.model import RobotModel
from src.kinematics.forward import ForwardKinematics
from src.solvers.analytical import AnalyticalIKSolver
from src.solvers.numerical import NumericalIKSolver

@pytest.fixture
def robot():
    return RobotModel([1.0, 1.0, 1.0])

@pytest.fixture
def fk():
    return ForwardKinematics()

def test_forward_kinematics_origin(robot, fk):
    """Test FK with all zeros (arm points straight up along Z)."""
    angles = np.zeros(3)
    pos = fk.solve(robot, angles)
    # Expected: x=0, y=0, z = L1 + L2 + L3 = 3.0
    expected = np.array([0, 0, 3.0])
    np.testing.assert_allclose(pos, expected, atol=1e-6)

def test_forward_kinematics_rotated(robot, fk):
    """Test FK with 90 degree base rotation."""
    angles = np.array([np.pi/2, 0, 0])
    pos = fk.solve(robot, angles)
    # Expected: x=0, y=0, z=3.0 (rotation around Z doesn't change straight-up pos)
    expected = np.array([0, 0, 3.0])
    np.testing.assert_allclose(pos, expected, atol=1e-6)

def test_analytical_ik(robot, fk):
    """Test Analytical IK solver."""
    solver = AnalyticalIKSolver(robot)
    target = np.array([1.0, 0.0, 2.0])
    
    solutions = solver.solve(target, elbow_up=True)
    assert solutions[0].success
    
    # Verify with FK
    result_pos = fk.solve(robot, solutions[0].joint_angles)
    np.testing.assert_allclose(result_pos, target, atol=1e-6)

def test_numerical_ik(robot, fk):
    """Test Numerical IK solver."""
    solver = NumericalIKSolver(robot)
    target = np.array([0.5, 0.5, 1.5])
    
    solutions = solver.solve(target, method="pseudoinverse")
    assert solutions[0].success
    
    # Verify with FK
    result_pos = fk.solve(robot, solutions[0].joint_angles)
    np.testing.assert_allclose(result_pos, target, atol=1e-4)

def test_out_of_reach(robot):
    """Test IK solver with out-of-reach target."""
    solver = AnalyticalIKSolver(robot)
    target = np.array([10.0, 10.0, 10.0])
    
    solutions = solver.solve(target)
    assert not solutions[0].success
    assert "out of reach" in solutions[0].message.lower()
