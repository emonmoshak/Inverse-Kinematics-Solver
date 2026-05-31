import numpy as np
from src.robot.model import RobotModel
from src.kinematics.forward import ForwardKinematics
from src.solvers.analytical import AnalyticalIKSolver

def basic_usage():
    # 1. Initialize Robot
    robot = RobotModel([1.0, 1.0, 0.5])
    fk = ForwardKinematics()
    ik = AnalyticalIKSolver(robot)
    
    # 2. Forward Kinematics
    q = np.array([np.pi/4, np.pi/6, -np.pi/4])
    pos = fk.solve(robot, q)
    print(f"Forward Kinematics:")
    print(f"Joint Angles: {q}")
    print(f"End Effector Position: {pos}")
    
    # 3. Inverse Kinematics
    target = np.array([0.8, 0.8, 1.2])
    solutions = ik.solve(target)
    
    if solutions[0].success:
        print(f"\nInverse Kinematics:")
        print(f"Target: {target}")
        print(f"Solved Angles: {solutions[0].joint_angles}")
        
        # Verify
        verify_pos = fk.solve(robot, solutions[0].joint_angles)
        print(f"Verification Pos: {verify_pos}")
    else:
        print("\nTarget unreachable.")

if __name__ == "__main__":
    basic_usage()
