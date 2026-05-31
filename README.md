# 3-DOF Robot Arm Inverse Kinematics Visualizer

[![CI](https://github.com/your-username/inverse-kinematics-solver/actions/workflows/ci.yml/badge.svg)](https://github.com/your-username/inverse-kinematics-solver/actions/workflows/ci.yml)
[![Version](https://img.shields.io/badge/version-0.1.0-blue.svg)](CHANGELOG.md)

A professional robotics simulation platform demonstrating inverse kinematics for a 3-DOF robotic manipulator using both analytical and numerical methods.

## Features

- **Robot Model:** Configurable 3-link arm with DH parameters.
- **Forward Kinematics:** DH-based transformation matrix calculation.
- **Analytical IK:** Geometric closed-form solver with multiple solutions (elbow-up/down).
- **Numerical IK:** Jacobian-based iterative solver (Pseudoinverse & Transpose methods).
- **Workspace Visualization:** 3D reachable point cloud generation.
- **Singularity Detection:** Jacobian determinant analysis.
- **Path Planning:** Linear and smooth cubic spline trajectory generation.
- **Real-time 3D Visualization:** Matplotlib-based interactive plotting.

## Mathematical Derivations

### 1. Forward Kinematics (DH Parameters)

The transformation from frame $i-1$ to $i$ is given by:

$$T_i^{i-1} = \begin{bmatrix} \cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\ \sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\ 0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

For our 3-DOF arm:
- Link 1: Base rotation ($q_1$), height $L_1$
- Link 2: Shoulder rotation ($q_2$), length $L_2$
- Link 3: Elbow rotation ($q_3$), length $L_3$

### 2. Analytical Inverse Kinematics

Using a geometric approach:

1. **Base Angle:**
   $$q_1 = \text{atan2}(y, x)$$

2. **Planar Reduction:**
   Project target into 2D plane:
   $$r = \sqrt{x^2 + y^2}$$
   $$z' = z - L_1$$

3. **Elbow Angle ($q_3$):**
   Using Law of Cosines on triangle $(L_2, L_3, d)$ where $d^2 = r^2 + (z')^2$:
   $$\cos q_3 = \frac{d^2 - L_2^2 - L_3^2}{2 L_2 L_3}$$
   $$q_3 = \pm \text{acos}(\cos q_3)$$

4. **Shoulder Angle ($q_2$):**
   $$q_2 = \text{atan2}(z', r) - \text{atan2}(L_3 \sin q_3, L_2 + L_3 \cos q_3)$$

### 3. Numerical Inverse Kinematics (Jacobian)

The relationship between joint velocities and end-effector velocity is:
$$\dot{x} = J(q)\dot{q}$$

To find $\Delta q$ for a given error $\Delta x$:
$$\Delta q = J^\dagger \Delta x$$

Where $J^\dagger$ is the Moore-Penrose pseudoinverse, often implemented with damping to handle singularities:
$$J^\dagger = (J^T J + \lambda^2 I)^{-1} J^T$$

## Installation

```bash
# Clone the repository
git clone https://github.com/your-username/inverse-kinematics-solver.git
cd inverse-kinematics-solver

# Create virtual environment
python -m venv venv
source venv/bin/activate  # or venv\Scripts\activate on Windows

# Install dependencies
pip install -r requirements.txt
```

## Usage

Run the main demo:

```bash
python main.py
```

## Testing

Run unit tests with pytest:

```bash
pytest
```

## Project Structure

```text
├── src/
│   ├── robot/         # Robot model and definitions
│   ├── kinematics/    # FK implementation
│   ├── solvers/       # Analytical and Numerical IK solvers
│   ├── workspace/     # Workspace generation logic
│   ├── trajectory/    # Path planning algorithms
│   └── visualization/ # 3D Plotting tools
├── tests/             # Unit tests
├── main.py            # Main entry point
└── pyproject.toml     # Configuration for Ruff, Black, and MyPy
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.
