import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

from src.kinematics.forward import ForwardKinematics
from src.robot.model import RobotModel


class RobotPlotter:
    """
    Handles 3D visualization of the robot arm using Matplotlib.
    """

    def __init__(self, robot: RobotModel) -> None:
        self.robot = robot
        self.fk = ForwardKinematics()

        # UI Colors from PRD
        self.colors = {
            "bg": "#0F172A",
            "panel": "#1E293B",
            "accent": "#38BDF8",
            "success": "#22C55E",
            "warning": "#F59E0B",
            "error": "#EF4444",
            "links": "#94A3B8",
        }

    def setup_plot(self) -> tuple[plt.Figure, Axes3D]:
        """Sets up the 3D plot with PRD-compliant styling."""
        plt.style.use("dark_background")
        fig = plt.figure(figsize=(10, 8), facecolor=self.colors["bg"])
        ax = fig.add_subplot(111, projection="3d")
        ax.set_facecolor(self.colors["bg"])

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("3-DOF Robot Arm IK Visualizer", color=self.colors["accent"])

        # Set equal aspect ratio
        limit = np.sum(self.robot.link_lengths)
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(0, limit * 1.5)

        return fig, ax

    def plot_arm(self, ax: Axes3D, joint_angles: np.ndarray, color: str = None) -> None:
        """
        Plots the robot arm links and joints.

        Args:
            ax: Matplotlib 3D axis.
            joint_angles: Current joint angles.
            color: Link color.
        """
        positions = self.fk.get_all_joint_positions(self.robot, joint_angles)
        x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]

        link_color = color or self.colors["links"]

        # Plot links
        ax.plot(
            x,
            y,
            z,
            color=link_color,
            linewidth=5,
            marker="o",
            markersize=8,
            markerfacecolor=self.colors["accent"],
        )

        # Plot end-effector
        ax.scatter(
            x[-1],
            y[-1],
            z[-1],
            color=self.colors["success"],
            s=100,
            label="End Effector",
        )

    def plot_workspace(self, ax: Axes3D, points: np.ndarray) -> None:
        """Plots the reachable workspace as a point cloud."""
        ax.scatter(
            points[:, 0],
            points[:, 1],
            points[:, 2],
            color=self.colors["accent"],
            s=1,
            alpha=0.1,
        )

    def plot_target(self, ax: Axes3D, target: np.ndarray) -> None:
        """Plots the target position."""
        ax.scatter(
            target[0],
            target[1],
            target[2],
            color=self.colors["error"],
            s=100,
            marker="X",
            label="Target",
        )
