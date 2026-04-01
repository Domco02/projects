# AI Coding Instructions for Robotics Kinematics Codebase

## Project Overview
Educational robotics project focused on forward/inverse kinematics for 2-DOF planar arms and the Franka Panda 7-DOF robot. Uses analytical solutions, Denavit-Hartenberg (DH) parameters, and PyBullet physics simulation for validation.

## Architecture & Major Components

### 1. Kinematics Modules
- **`2dofAnalytical.py`**: Analytical inverse kinematics for 2-DOF planar arm using law of cosines; returns multiple solutions for elbow-up/down
- **`2dofDH.py`**: DH parameter-based transformations for 2-DOF
- **`franka.py`**: Franka Panda 7-DOF kinematics using DH parameters with PyBullet simulation; compares analytical FK against PyBullet's computed end-effector
- **`2dofArm.py`**: PyBullet-based simulation interface with motor control

### 2. Mathematical Utilities (Duplicated Across Modules)
- **`Rotations.py`**, **`Euler.py`**, **`HomogennaTransformacia.py`**: Elementary rotation matrices and utilities
  - `Rx()`, `Ry()`, `Rz()`: Rotation matrices around axes (all take radians)
  - `rpy_to_rot()`: Roll-Pitch-Yaw to rotation matrix (Rz @ Ry @ Rx composition order)
  - `skew()`: Skew-symmetric matrix for cross products
  - `homogeneous_transform()`: Builds 4×4 transformation from rotation + translation
- **`quaternion.py`**: `quat_to_rot()` converts quaternions [eta, x, y, z] to 3×3 rotation
- **`RigidBody.py`**: Frame visualization, point transformation utilities

## Data Structures & Conventions

### Homogeneous Transformations (4×4 matrices)
```python
T = np.eye(4)
T[:3, :3] = R       # 3×3 rotation
T[:3, 3] = origin   # 3×1 translation
```
- Extract: `translation = T[:3, 3]`, `rotation = T[:3, :3]`
- Apply: `p_transformed = T @ np.append(p, 1)`

### DH Parameters
Represented as 3-element lists: `[a, d, alpha]` (theta is joint variable, passed separately)
- Example from `franka.py`: `[0.333, 0.0, 0.0]` for joint 1

### Euler Angles
Convention: **XYZ intrinsic** (used with `scipy.spatial.transform.Rotation`)
- Default: radians (convert with `np.deg2rad()`, `np.radians()`)
- Extract from matrix: `R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False)`

## Key Patterns & Workflows

### Pattern 1: Analytical IK Multiple Solutions
`2dofAnalytical.py` returns list of solutions:
```python
solutions = inverse_kinematics(x, y)  # Returns [(theta1, theta2), ...]
for theta1, theta2 in solutions:
    x_coords, y_coords = forward_kinematics(theta1, theta2)
```

### Pattern 2: DH-Based Forward Kinematics
Compose transformations sequentially (see `franka.py`):
```python
T = np.eye(4)
for i, (theta_i, a, d, alpha) in enumerate(dh_params):
    T_i = dh_transformation(theta_i, a, d, alpha)
    T = T @ T_i  # Cumulative transformation
```

### Pattern 3: PyBullet Validation
Compare analytical FK with PyBullet simulation:
```python
# Analytical: Compute FK using DH
fk_position_dh = T[:3, 3]

# PyBullet: Get end-effector from simulation
ee_state = p.getLinkState(robot_id, link_index, computeForwardKinematics=True)
fk_position_pybullet = np.array(ee_state[4])
```

### Pattern 4: Frame Visualization
Common visualization function (duplicated in multiple modules):
```python
plot_frame(ax, R, origin, label, length=0.5)
# Draws RGB quivers: red=X, green=Y, blue=Z
```

## Important Conventions

1. **Angles always in radians** (except when explicitly converting)
2. **Frame visualization**: X→red, Y→green, Z→blue quivers
3. **Quaternion format**: `[eta (scalar), x, y, z]` in `quaternion.py`
4. **Joint angles in PyBullet**: Set joint indices 1, 2, 3... for first, second, third joints
5. **Link indices in PyBullet**: Offset by link number (e.g., Franka end-effector is link 8 for 7-DOF robot)
6. **Robot files**: URDF definitions in `urdf/` directory

## Integration Points

- **PyBullet**: Used for physics simulation and validation; requires URDF files from `urdf/` directory
- **NumPy/SciPy**: All math operations; `scipy.spatial.transform.Rotation` for quaternion/Euler conversions
- **Matplotlib**: 3D visualization with `mpl_toolkits.mplot3d`
- **Jupyter notebooks**: Interactive exploration (e.g., `Python_pisomka.ipynb`, `Abahazi_pisomka.ipynb`)

## Code Quality Notes

- **DRY violation**: `plot_frame()` is duplicated across modules—if editing, update all occurrences
- **Limited error handling**: `inverse_kinematics()` raises `ValueError` if target out of reach
- **Test approach**: Validation via side-by-side comparison (analytical vs. simulation)
