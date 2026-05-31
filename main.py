import matplotlib.pyplot as plt
import numpy as np

from src.kinematics.forward import ForwardKinematics
from src.robot.model import RobotModel
from src.solvers.analytical import AnalyticalIKSolver
from src.solvers.numerical import NumericalIKSolver
from src.trajectory.planner import TrajectoryPlanner
from src.visualization.plotter import RobotPlotter
from src.workspace.generator import WorkspaceGenerator

SINGULARITY_THRESHOLD = 1e-3


def setup_robot() -> tuple:
    """Initializes the robot and all core components."""
    link_lengths = [1.0, 1.0, 0.8]  # L1, L2, L3 in meters
    robot = RobotModel(link_lengths)
    fk = ForwardKinematics()
    analytical_solver = AnalyticalIKSolver(robot)
    numerical_solver = NumericalIKSolver(robot)
    workspace_gen = WorkspaceGenerator(robot)
    planner = TrajectoryPlanner()
    plotter = RobotPlotter(robot)
    return (
        robot,
        fk,
        analytical_solver,
        numerical_solver,
        workspace_gen,
        planner,
        plotter,
    )


def run_demo():
    """Runs the full IK visualizer demo."""
    print("Initializing 3-DOF Robot Arm IK Visualizer...")
    (
        robot,
        fk,
        analytical_solver,
        numerical_solver,
        workspace_gen,
        planner,
        plotter,
    ) = setup_robot()

    # 1. Workspace Generation (sampled for speed in demo)
    print("Generating workspace...")
    workspace_points = workspace_gen.generate(samples_per_joint=10)

    # 2. Define a Target Position
    target_pos = np.array([1.2, 0.5, 1.5])
    print(f"Target position: {target_pos}")

    # 3. Solve IK Analytically
    print("Solving IK analytically...")
    analytical_solutions = analytical_solver.solve(target_pos, elbow_up=True)
    q_analytical = (
        analytical_solutions[0].joint_angles
        if analytical_solutions[0].success
        else np.zeros(3)
    )
    if analytical_solutions[0].success:
        print(f"Analytical angles: {np.degrees(q_analytical)} deg")

    # 4. Solve IK Numerically
    print("Solving IK numerically...")
    numerical_solutions = numerical_solver.solve(target_pos, method="pseudoinverse")
    q_numerical = (
        numerical_solutions[0].joint_angles
        if numerical_solutions[0].success
        else np.zeros(3)
    )
    if numerical_solutions[0].success:
        print(f"Numerical angles: {np.degrees(q_numerical)} deg")
        print(f"Numerical iterations: {numerical_solutions[0].iterations}")
        print(f"Final error: {numerical_solutions[0].error:.6f}")

    # 5. Singularity Check
    det = numerical_solver.get_determinant(q_numerical)
    print(f"Jacobian determinant at target: {det:.6f}")
    if abs(det) < SINGULARITY_THRESHOLD:
        print("WARNING: Robot is near a singularity!")

    # 6. Trajectory Planning
    print("Generating trajectory...")
    start_pos = fk.solve(robot, np.zeros(3))
    waypoints = np.array([start_pos, [1.0, 1.0, 1.0], target_pos])
    trajectory = planner.cubic_interpolate(waypoints, steps=30)

    # 7. Visualization
    print("Opening visualization...")
    fig, ax = plotter.setup_plot()
    plotter.plot_workspace(ax, workspace_points)
    plotter.plot_target(ax, target_pos)

    # Animate trajectory
    for point in trajectory:
        sol = analytical_solver.solve(point, elbow_up=True)
        if sol[0].success:
            for line in ax.get_lines():
                if line.get_label() != "Target":
                    line.remove()
            plotter.plot_arm(ax, sol[0].joint_angles)
            plt.pause(0.05)

    print("Demo complete.")
    plt.show()


if __name__ == "__main__":
    run_demo()
