# Maze environment wrapper for active exploration training
import math
from typing import Optional, Sequence

import numpy as np
import pybullet as p
import pybullet_data
import torch
import torch.nn as nn
import torch.nn.functional as F

try:
    from torch.func import functional_call, stack_module_state, vmap  # type: ignore[attr-defined]
except ImportError:  # pragma: no cover - older PyTorch
    functional_call = None
    stack_module_state = None
    vmap = None


# ---------- Constants ----------
INCH = 0.0254
MAZE_INCH = 140.0
N = 12  # 13x13 cells
MAZE = MAZE_INCH * INCH               # total side length (meters)
CELL = MAZE / N                        # cell size (meters)
HALF = MAZE / 2.0
WALL_T = 0.04                          # wall thickness (meters)
WALL_H = 0.25                          # wall height (meters)

# Robot specs
ROBOT_R = 0.08                         # robot radius (meters)
ROBOT_H = 0.08                         # robot height (meters)
ROBOT_MASS = 1.0                       # robot mass (kg)
PROBE_DEPTH = 0.08                     # probe depth - extends from robot face (meters)
ROBOT_DIAMETER = 2.0 * ROBOT_R         # convenience (meters)
EDGE_RAY_EXTENSION = ROBOT_DIAMETER    # extend edge rays to cover corners

# Action scales (for normalized actions in [-1, 1])
ACTION_SCALE_FORWARD = 2.0             # max forward velocity (m/s)
ACTION_SCALE_YAW = 3.0                 # max angular velocity (rad/s)

# Raycast sensing (mirrors ray_test.py defaults)
RAY_LENGTH = 5.0                       # length of forward ray (meters)
RAY_STOP_TOL = 0.05                    # clearance (meters) before we consider contact

# Completion zone (legacy grid references retained for compatibility)
COMPLETION_ZONE_COL_MIN = 10
COMPLETION_ZONE_COL_MAX = 12
COMPLETION_ZONE_ROW_MIN = 0
COMPLETION_ZONE_ROW_MAX = 2

# Checkpoint configuration
CHECKPOINT_HALF_SIZE = 0.15  # 0.3 m square
CHECKPOINT_BONUS = 0.0
CHECKPOINT_DISTANCE_SCALE = 0.0
CHECKPOINT_CENTERS = (
    np.array([-1.608, -0.208], dtype=np.float32),
    np.array([-1.038, -0.108], dtype=np.float32), 
    np.array([-0.938, -0.108], dtype=np.float32),
    np.array([-0.738, -0.108], dtype=np.float32),
    np.array([-0.038, -0.108], dtype=np.float32),
    np.array([1.609, 0.692], dtype=np.float32),
)
GOAL_CENTER = np.array([1.408, 1.408], dtype=np.float32)
GOAL_HALF_SIZE = 0.50

# -----------------------------
# Maze Environment
# -----------------------------

class MazeEnv:
    
    def __init__(self, gui: bool = False, dt: float = 1/480,
                 control_dt: float = 0.1, max_steps: int = 30000,
                 forward_speed: float = 1.0, angular_speed: float = 2.0,
                 forward_distance: float = 0.1,
                 turn_tolerance: float = 0.01,
                 distance_tolerance: float = 1e-3,
                 collision_penalty: float = 0.0, step_penalty: float = 0.02,
                 checkpoint_step_penalty: float | Sequence[float] = 0.0,
                 goal_step_penalty: float = 0.0,
                 goal_reward: float = 50.0, goal_radius: float = 0.1,
                 substeps: int = 4, progress_weight: float = 1.0,
                 seed=None):
        """
        Args:
            gui: If True, use GUI mode (p.GUI), else headless (p.DIRECT).
            dt: Physics engine timestep (seconds).
            control_dt: Duration that each action command is applied (seconds).
            max_steps: Maximum steps per episode.
            forward_speed: Commanded forward speed when moving (m/s).
            angular_speed: Reserved for future use (kept for API compatibility).
            forward_distance: Nominal translation distance per forward command (meters).
            turn_tolerance: Yaw tolerance (radians) when snapping to target heading.
            distance_tolerance: Linear tolerance (meters) for forward moves.
            collision_penalty: Penalty applied when a collision occurs.
            step_penalty: Base per-step penalty applied regardless of stage.
            checkpoint_step_penalty: Additional per-step penalty applied before each checkpoint is
                reached. Provide a float to apply the same penalty to all checkpoints or a
                sequence to configure them individually (shorter sequences are extended using the
                final value).
            goal_step_penalty: Additional per-step penalty applied before the goal is reached.
            goal_reward: Reward granted when reaching the goal zone.
            goal_radius: Radius around goal center considered terminal.
            substeps: Internal Bullet substeps per `dt`.
            progress_weight: Optional shaping term based on distance to goal.
            seed: Random seed for reproducibility.
        """
        self.gui = gui
        self.dt = float(dt)
        self.control_dt = float(control_dt)
        self.max_steps = max_steps
        self.step_count = 0
        self.substeps = int(max(1, substeps))
        self.forward_speed = float(forward_speed)
        self.angular_speed = float(angular_speed)
        self.forward_distance = float(forward_distance)
        self.turn_tolerance = float(turn_tolerance)
        self.distance_tolerance = float(distance_tolerance)
        self.collision_penalty = float(collision_penalty)
        self.step_penalty = float(step_penalty)
        self.goal_reward = float(goal_reward)
        self.goal_radius = float(goal_radius)
        self._progress_weight = float(progress_weight)
        self._collision_this_step: bool = False
        self.v = self.forward_speed  # alias for planner code
        self._ray_length = RAY_LENGTH
        self._ray_stop_clearance = RAY_STOP_TOL
        self._edge_ray_extension = EDGE_RAY_EXTENSION
        
        # Connect to PyBullet
        self.cid = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self._plane_id = p.loadURDF("plane.urdf")
        
        if gui:
            p.resetDebugVisualizerCamera(3.3, 0, -80, [0,0,0])
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        
        # Build maze and robot
        self.robot_id = None
        self._wall_ids = []
        self._goal_center = GOAL_CENTER.copy()
        goal_half = GOAL_HALF_SIZE
        self._goal_bounds = (
            self._goal_center[0] - goal_half,
            self._goal_center[0] + goal_half,
            self._goal_center[1] - goal_half,
            self._goal_center[1] + goal_half,
        )
        self._last_goal_distance: Optional[float] = None
        clearance = WALL_T / 2 + 0.1
        start_x = -HALF + clearance + 0.05
        start_y = -HALF + clearance + 0.05
        self._start_xy = np.array([start_x, start_y], dtype=float)
        self._start_yaw = math.pi / 2
        self._checkpoint_half_extent = CHECKPOINT_HALF_SIZE
        self._checkpoint_bonus = CHECKPOINT_BONUS
        self._checkpoint_distance_scale = CHECKPOINT_DISTANCE_SCALE
        self._checkpoint_centers: list[np.ndarray] = [center.copy() for center in CHECKPOINT_CENTERS]
        self._checkpoint_stage: int = 0
        num_checkpoints = len(self._checkpoint_centers)
        if num_checkpoints:
            if isinstance(checkpoint_step_penalty, (tuple, list)):
                penalties = [float(p) for p in checkpoint_step_penalty]
                if not penalties:
                    penalties = [0.0]
                if len(penalties) < num_checkpoints:
                    penalties.extend([penalties[-1]] * (num_checkpoints - len(penalties)))
                elif len(penalties) > num_checkpoints:
                    penalties = penalties[:num_checkpoints]
            else:
                penalties = [float(checkpoint_step_penalty)] * num_checkpoints
        else:
            penalties = []
        self._checkpoint_step_penalties = penalties
        self._checkpoint_penalty_active: list[bool] = []
        self._goal_step_penalty = float(goal_step_penalty)
        self._goal_penalty_active = self._goal_step_penalty != 0.0
        self._reset_checkpoints()
        
        # Collision filtering groups
        self._robot_collision_group = 1
        self._wall_collision_group = 2
        
        self._rng = np.random.default_rng(seed)
        
        self._build_world()
        
        # Control horizon
        self._steps_per_action = max(1, int(round(self.control_dt / self.dt)))
        
        # Action space metadata (compatibility)
        self.action_dim = 4
        self.A = self.action_dim
        self.S = None

        p.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=self.substeps,
            numSolverIterations=50
        )

    def _build_world(self):
        """Build the complete maze including boundaries and interior walls."""
        # Build boundary walls
        self._add_h_run(0, 0, N)        # TOP
        self._add_h_run("L", 0, N)       # BOTTOM
        self._add_v_run(0, 0, "L")       # LEFT
        self._add_v_run(N, 0, "L")       # RIGHT
        
        # Build interior maze walls (horizontal bars)
        self._add_h_run("A", 3, 5)
        self._add_h_run("A", 6, 8)
        self._add_h_run("B", 5, 7)
        self._add_h_run("C", 1, 3)
        self._add_h_run("C", 6, 7)
        self._add_h_run("C", 9, 10)
        self._add_h_run("D", 6, 7)
        self._add_h_run("D", 9, 11)
        self._add_h_run("E", 4, 7)
        self._add_h_run("G", 3, 11)
        self._add_h_run("H", 1, 3)
        self._add_h_run("H", 5, 8)
        self._add_h_run("I", 1, 3)
        self._add_h_run("I", 5, 6)
        self._add_h_run("I", 9, 11)
        self._add_h_run("J", 4, 7)
        self._add_h_run("J", 8, 9)
        self._add_h_run("K", 5, 6)
        self._add_h_run("K", 7, 8)
        
        # Build interior maze walls (vertical bars)
        self._add_v_run(1, "C", "E")
        self._add_v_run(1, "H", "I")
        self._add_v_run(3, "A", "F")
        self._add_v_run(3, "I", "K")
        self._add_v_run(4, "B", "C")
        self._add_v_run(4, "C", "F")
        self._add_v_run(4, "G", "K")
        self._add_v_run(5, "H", "I")
        self._add_v_run(5, "K", "L")
        self._add_v_run(6, "C", "D")
        self._add_v_run(7, "B", "C")
        self._add_v_run(7, "D", "E")
        self._add_v_run(7, "H", "J")
        self._add_v_run(8, "A", "E")
        self._add_v_run(8, "H", "K")
        self._add_v_run(9, "A", "C")
        self._add_v_run(9, "G", "I")
        self._add_v_run(9, "J", "L")
        self._add_v_run(10, "C", "D")
        
        # Create robot
        self._create_robot()

    def _reset_checkpoints(self) -> None:
        """Reset checkpoint progression state."""
        self._checkpoint_stage = 0
        self._goal_reached = False
        if self._checkpoint_step_penalties:
            self._checkpoint_penalty_active = [penalty != 0.0 for penalty in self._checkpoint_step_penalties]
        else:
            self._checkpoint_penalty_active = []
        self._goal_penalty_active = self._goal_step_penalty != 0.0

    @staticmethod
    def _distance_to_square(point: np.ndarray, center: np.ndarray, half_extent: float) -> tuple[float, bool]:
        """Return distance from point to axis-aligned square and whether it lies inside."""
        dx = abs(point[0] - center[0]) - half_extent
        dy = abs(point[1] - center[1]) - half_extent
        dx = max(dx, 0.0)
        dy = max(dy, 0.0)
        dist = math.hypot(dx, dy)
        # print(f"point: {np.round(point, 3)}, center: {np.round(center, 3)}, half_extent: {round(half_extent, 3)}, dist: {round(dist, 3)}")
        inside = dx == 0.0 and dy == 0.0
        return dist, inside

    def _current_target_centers(self) -> tuple[list[np.ndarray], str]:
        """Return active target centers and stage label."""
        # if self._checkpoint_stage < len(self._checkpoint_centers):
        #     return [self._checkpoint_centers[self._checkpoint_stage]], "checkpoint"
        return [self._goal_center], "goal"

    def _x_at(self, i):
        """World x coordinate of grid line i (0..N)."""
        return -HALF + i * CELL

    def _y_at(self, j):
        """World y coordinate of grid line j (0..N)."""
        return -HALF + j * CELL

    def _row_index_top0(self, row):
        """Map a top-origin row label to 0..N (0=top grid line, N=bottom)."""
        LETTERS = {ch: i for i, ch in enumerate("0ABCDEFGHIJKLMNOPQRSTUVWXYZ")}
        if isinstance(row, int):
            return row
        else:
            return LETTERS[row]

    def _row_to_world_y_top0(self, row):
        """World y coordinate for a top-origin row grid line."""
        j_top0 = self._row_index_top0(row)    # 0..N (0=top)
        j_bottom0 = N - j_top0                 # convert to bottom-origin
        return self._y_at(j_bottom0)

    def _add_wall_segment(self, x, y, length, orientation='h', color=[0.85, 0.3, 0.3, 1]):
        """
        Place a single wall segment centered at (x,y), axis-aligned.
        orientation='h' -> long along X, thin along Y
        orientation='v' -> long along Y, thin along X
        """
        if orientation == 'h':
            hx, hy = length/2, WALL_T/2
        else:  # 'v'
            hx, hy = WALL_T/2, length/2

        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, WALL_H/2])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, WALL_H/2], rgbaColor=color)
        wall_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                    basePosition=[x, y, WALL_H/2])
        # Set collision group for filtering
        p.setCollisionFilterGroupMask(
            wall_id, -1, self._wall_collision_group, self._robot_collision_group
        )
        p.changeDynamics(
            wall_id,
            -1,
            lateralFriction=1.0,
            restitution=0.0,
        )
        self._wall_ids.append(wall_id)

    def _add_h_run(self, row, i0, i1):
        """Add horizontal wall run: row (0 or 'A'..'L'), columns i0..i1 (0..N)."""
        i0, i1 = sorted((int(i0), int(i1)))
        length = (i1 - i0) * CELL
        if length <= 0:
            return
        cx = (self._x_at(i0) + self._x_at(i1)) / 2.0
        cy = self._row_to_world_y_top0(row)
        self._add_wall_segment(cx, cy, length, 'h')

    def _add_v_run(self, col_i, r0, r1):
        """Add vertical wall run: column col_i (0..N), rows r0..r1 (0 or 'A'..'L')."""
        j0 = self._row_index_top0(r0)
        j1 = self._row_index_top0(r1)
        j0, j1 = sorted((j0, j1))
        length = (j1 - j0) * CELL
        if length <= 0:
            return
        cx = self._x_at(int(col_i))
        y0 = self._row_to_world_y_top0(j0)
        y1 = self._row_to_world_y_top0(j1)
        cy = (y0 + y1) / 2.0
        self._add_wall_segment(cx, cy, length, 'v')

    def _create_robot(self):
        """Create the cubic robot at starting position."""
        start_x, start_y = self._start_xy
        start_z = ROBOT_H / 2
        
        # Create cube with half-extents: x/y use ROBOT_R as half-extent (matching collision_detection.py convention)
        # This makes a cube with side length ROBOT_R*2 in x/y, height ROBOT_H in z
        # Note: In collision_detection.py, ROBOT_R represents the cube side length, so half-extent = ROBOT_R
        # But in maze_env, ROBOT_R was originally radius=0.07, so we use it directly as half-extent
        cube_half_extent = ROBOT_R  # Use ROBOT_R directly as half-extent to match collision_detection.py
        col_r = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_half_extent, cube_half_extent, ROBOT_H/2])
        vis_r = p.createVisualShape(p.GEOM_BOX, halfExtents=[cube_half_extent, cube_half_extent, ROBOT_H/2],
                                     rgbaColor=[0.1, 0.6, 0.9, 1])
        
        # Create visual identifier (bright red sphere) on front face to show direction
        # Front face is at +ROBOT_R in local +X direction (which aligns with heading when facing North)
        # Position it slightly in front of the face for visibility
        indicator_radius = 0.015  # 1.5cm radius sphere
        indicator_offset = ROBOT_R + indicator_radius * 0.5  # Position just in front of robot face
        vis_indicator = p.createVisualShape(p.GEOM_SPHERE, radius=indicator_radius,
                                            rgbaColor=[1.0, 0.0, 0.0, 1.0])  # Bright red
        # No collision shape for indicator (visual only)
        col_indicator = p.createCollisionShape(p.GEOM_SPHERE, radius=indicator_radius)
        
        # Create robot with main body as base and indicator as child link
        # Link 0 (child) will have the indicator at the front face
        self.robot_id = p.createMultiBody(
            baseMass=ROBOT_MASS,
            baseCollisionShapeIndex=col_r,
            baseVisualShapeIndex=vis_r,
            basePosition=[start_x, start_y, start_z],
            linkMasses=[0.0],  # Indicator has no mass (visual only)
            linkCollisionShapeIndices=[col_indicator],
            linkVisualShapeIndices=[vis_indicator],
            linkPositions=[[indicator_offset, 0, 0]],  # Position at front face (+X)
            linkOrientations=[[0, 0, 0, 1]],
            linkInertialFramePositions=[[0, 0, 0]],
            linkInertialFrameOrientations=[[0, 0, 0, 1]],
            linkParentIndices=[0],  # Child of base link
            linkJointTypes=[p.JOINT_FIXED],  # Fixed joint (no movement)
            linkJointAxis=[[0, 0, 1]]
        )
        self.base_z = start_z
        p.changeDynamics(
            self.robot_id,
            -1,
            linearDamping=0.1,
            angularDamping=0.1,
            lateralFriction=1.0,
            restitution=0.0,
            contactStiffness=1e5,
            contactDamping=1e3,
        )
        # Set collision group for filtering (base link)
        p.setCollisionFilterGroupMask(
            self.robot_id, -1, self._robot_collision_group, self._wall_collision_group
        )
        # Disable collision for indicator link (visual only)
        p.setCollisionFilterGroupMask(self.robot_id, 0, 0, 0)

    def get_state(self) -> np.ndarray:
        """Return current continuous state (x, y, yaw)."""
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        yaw = p.getEulerFromQuaternion(orn)[2]
        return np.array([pos[0], pos[1], yaw], dtype=float)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _fan_ray_specs(self, position: np.ndarray, yaw: float) -> list[dict]:
        """Compute forward-facing fan ray specifications mirroring ray_test.py."""
        px, py, pz = position
        specs: list[dict] = []
        for angle_deg in range(-30, 30, 5):
            angle_rad = math.radians(angle_deg)
            ray_yaw = yaw + angle_rad
            direction = [math.cos(ray_yaw), math.sin(ray_yaw), 0.0]
            origin = [
                px + direction[0] * ROBOT_R,
                py + direction[1] * ROBOT_R,
                pz,
            ]
            end = [
                origin[0] + direction[0] * self._ray_length,
                origin[1] + direction[1] * self._ray_length,
                origin[2] + direction[2] * self._ray_length,
            ]
            specs.append(
                {
                    "name": f"{angle_deg:+d}",
                    "origin": origin,
                    "end": end,
                    "length": self._ray_length,
                    "angle_offset": angle_rad,
                }
            )
        return specs

    def _cast_fan_rays(self, position: np.ndarray, yaw: float) -> list[dict]:
        """Cast the fan of rays and compute forward clearances."""
        specs = self._fan_ray_specs(position, yaw)
        results = p.rayTestBatch(
            [spec["origin"] for spec in specs],
            [spec["end"] for spec in specs],
        )
        for spec, result in zip(specs, results):
            hit_object = result[0]
            hit_fraction = result[2]
            length = float(spec["length"])
            distance_to_hit = None if hit_object == -1 else hit_fraction * length
            if distance_to_hit is None:
                forward_clearance = None
            else:
                angle_mag = abs(float(spec["angle_offset"]))
                projected_forward = distance_to_hit * math.cos(angle_mag)
                origin_forward_offset = ROBOT_R * (1.0 - math.cos(angle_mag))
                forward_clearance = max(0.0, projected_forward - origin_forward_offset)
            spec.update(
                {
                    "hit_object": hit_object,
                    "hit_fraction": hit_fraction,
                    "distance_to_hit": distance_to_hit,
                    "forward_clearance": forward_clearance,
                }
            )
        return specs

    def _check_wall_contact(self) -> bool:
        """Return True if robot is too close to a wall based on forward ray sensing."""
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        yaw = p.getEulerFromQuaternion(orn)[2]

        ray_infos = self._cast_fan_rays(pos, yaw)
        wall_clearances = [
            info["forward_clearance"]
            for info in ray_infos
            if info["hit_object"] in self._wall_ids and info["forward_clearance"] is not None
        ]
        if not wall_clearances:
            return False
        min_clearance = min(wall_clearances)
        # round to 3 decimal places
        min_clearance = round(min_clearance, 3)
        return min_clearance <= self._ray_stop_clearance

    def _rotate_to_yaw(self, target_yaw: float) -> None:
        """Rotate robot in-place until target yaw (radians) is reached."""
        target_yaw = self._wrap_angle(target_yaw)
        discrete_yaws = [0.0, math.pi / 2.0, -math.pi / 2.0, math.pi]
        target_yaw = min(
            discrete_yaws,
            key=lambda candidate: abs(self._wrap_angle(target_yaw - candidate)),
        )

        # Quick exit if already aligned
        _, _, current_yaw = self.get_state()
        error = self._wrap_angle(target_yaw - current_yaw)
        if abs(error) <= self.turn_tolerance:
            pos, _ = p.getBasePositionAndOrientation(self.robot_id)
            quat = p.getQuaternionFromEuler([0, 0, target_yaw])
            p.resetBasePositionAndOrientation(self.robot_id, [pos[0], pos[1], self.base_z], quat)
            p.resetBaseVelocity(self.robot_id, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            return

        # # Stop rotation and snap exactly to target
        p.resetBaseVelocity(self.robot_id, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        quat = p.getQuaternionFromEuler([0, 0, target_yaw])
        quat = p.getQuaternionFromEuler([0, 0, target_yaw])
        p.resetBasePositionAndOrientation(self.robot_id, [pos[0], pos[1], self.base_z], quat)
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        p.stepSimulation()

    def _drive_forward_distance(self, distance: float, yaw: float) -> None:
        """Drive the robot forward by a specified distance (meters) along yaw."""
        yaw = self._wrap_angle(yaw)
        direction = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
        start_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        start_xy = np.array(start_pos[:2], dtype=float)

        vx = self.forward_speed * direction[0]
        vy = self.forward_speed * direction[1]

        traveled = 0.0
        # Safety cap: assume ideal travel completed in distance / (speed*dt) steps
        ideal_steps = max(1, int(math.ceil(distance / max(self.forward_speed * self.dt, 1e-6))))
        max_steps = ideal_steps * 6  # generous margin

        for _ in range(max_steps):
            
            if traveled >= distance - self.distance_tolerance:
                break

            if not self._check_wall_contact():
                p.resetBaseVelocity(self.robot_id, [vx, vy, 0.0], [0.0, 0.0, 0.0])
            p.stepSimulation()

            if self._check_wall_contact():
                self._collision_this_step = True
                p.resetBaseVelocity(self.robot_id, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                break

            pos, _ = p.getBasePositionAndOrientation(self.robot_id)
            traveled = float(np.linalg.norm(np.array(pos[:2]) - start_xy))

        # Stop motion
        p.resetBaseVelocity(self.robot_id, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

        success = (
            not self._collision_this_step
            and traveled >= distance - self.distance_tolerance
        )
        if success:
            # Snap to commanded distance to ensure precise lattice-free motion
            final_xy = start_xy + direction * distance
            pos, orn = p.getBasePositionAndOrientation(self.robot_id)
            p.resetBasePositionAndOrientation(
                self.robot_id,
                [final_xy[0], final_xy[1], pos[2]],
                orn,
            )

    def _compute_reward_and_done(
        self,
        state: np.ndarray,
        *,
        collision: Optional[bool] = None,
        prev_goal_distance: Optional[float] = None,
        update_progress: bool = True,
        ) -> tuple[float, bool]:
        """Compute reward and termination flag based on continuous state."""
        xy = state[:2]
        targets, stage_label = self._current_target_centers()
        best_distance = float("inf")
        inside_target = False
        chosen_center: Optional[np.ndarray] = None
        for center in targets:
            dist, inside = self._distance_to_square(xy, center, self._checkpoint_half_extent if stage_label == "checkpoint" else GOAL_HALF_SIZE)
            if inside:
                inside_target = True
                chosen_center = center
                best_distance = 0.0
                break
            if dist < best_distance:
                best_distance = dist
                chosen_center = center

        # Time penalty
        reward = -self.step_penalty

        # Dense progress shaping toward current target
        if prev_goal_distance is not None and self._progress_weight > 0:
            reward += self._progress_weight * (prev_goal_distance - best_distance)
        else:
            # Fallback to absolute-distance shaping if previous distance unknown
            reward += -self._checkpoint_distance_scale * best_distance
        next_reference_distance = best_distance
        done = False

        if stage_label == "checkpoint":
            stage_index = self._checkpoint_stage
            if 0 <= stage_index < len(self._checkpoint_step_penalties):
                checkpoint_penalty = self._checkpoint_step_penalties[stage_index]
                penalty_active = (
                    stage_index < len(self._checkpoint_penalty_active)
                    and self._checkpoint_penalty_active[stage_index]
                )
                if inside_target and update_progress and penalty_active:
                    self._checkpoint_penalty_active[stage_index] = False
                    penalty_active = False
                if penalty_active and not inside_target:
                    reward -= checkpoint_penalty

            if inside_target:
                reward += self._checkpoint_bonus
                if update_progress:
                    self._checkpoint_stage += 1
                    if self._checkpoint_stage < len(self._checkpoint_centers):
                        next_center = self._checkpoint_centers[self._checkpoint_stage]
                        next_reference_distance = float(np.linalg.norm(xy - next_center))
                    else:
                        next_reference_distance = float(np.linalg.norm(xy - self._goal_center))
        else:
            penalty_active = self._goal_penalty_active
            if inside_target and update_progress and penalty_active:
                self._goal_penalty_active = False
                penalty_active = False
            if penalty_active and not inside_target:
                reward -= self._goal_step_penalty

            if inside_target and not self._goal_reached:
                reward += self.goal_reward
                done = True
                if update_progress:
                    self._goal_reached = True
            elif self._goal_reached:
                done = True

        collision_flag = self._collision_this_step if collision is None else collision
        if collision_flag and self.collision_penalty:
            reward -= self.collision_penalty
        if update_progress and self.step_count >= self.max_steps:
            done = True

        if update_progress:
            self._last_goal_distance = next_reference_distance
        return reward, done
    
    def _is_in_completion_zone(self, pos):
        """Check if robot position is inside the completion zone AABB."""
        x, y, _ = pos
        x_min, x_max, y_min, y_max = self._goal_bounds
        return x_min <= x <= x_max and y_min <= y <= y_max

    def reset(self, seed=None) -> np.ndarray:
        """Reset environment to start pose and return continuous state."""
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        
        self._reset_checkpoints()
        self.step_count = 0
        self._collision_this_step = False

        start_x, start_y = self._start_xy
        quat = p.getQuaternionFromEuler([0, 0, self._start_yaw])
        p.resetBasePositionAndOrientation(
            self.robot_id,
            [start_x, start_y, self.base_z],
            quat,
        )
        p.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])

        if self._checkpoint_centers:
            first_center = self._checkpoint_centers[0]
        else:
            first_center = self._goal_center
            
        distance, _ = self._distance_to_square(self._start_xy, self._goal_center, GOAL_HALF_SIZE)
        self._last_goal_distance = distance

        return self.get_state()

    def step(self, action: int) -> tuple[np.ndarray, float, bool, dict]:
        """
        Execute one control step in the environment with continuous dynamics.
        Action semantics: apply discrete turn then integrate forward velocity.
        """
        if action not in (0, 1, 2, 3):
            raise ValueError(f"Action must be 0-3, got {action}")

        self.step_count += 1
        
        state = self.get_state()
        x, y, yaw = state

        turn_map = {0: 0.0, 1: math.pi / 2, 2: -math.pi / 2, 3: math.pi}
        delta_yaw = turn_map[action]
        target_yaw = self._wrap_angle(yaw + delta_yaw)

        self._collision_this_step = False
        self._rotate_to_yaw(target_yaw)
        self._drive_forward_distance(self.forward_distance, target_yaw)

        next_state = self.get_state()
        reward, done = self._compute_reward_and_done(
            next_state,
            collision=self._collision_this_step,
            prev_goal_distance=self._last_goal_distance,
            update_progress=True,
        )
        targets, stage_label = self._current_target_centers()
        info = {
            "collision": self._collision_this_step,
            "action": action,
            "stage": stage_label,
            "checkpoint_stage": self._checkpoint_stage,
            "active_targets": [center.tolist() for center in targets],
        }
        
        return next_state, reward, done, info

    def simulate_model(self, state: np.ndarray, action: int, model) -> tuple[np.ndarray, float, bool]:
        """
        Roll out a learned dynamics model that predicts state deltas in continuous space.

        Args:
            state: Current state vector (x, y, yaw).
            action: Discrete turn action (0=straight, 1=right, 2=left, 3=turn-around).
            model: Dynamics model with a predict_delta(state, action) method.
        Returns:
            (next_state, reward, done)
        """
        if action not in (0, 1, 2, 3):
            raise ValueError(f"Action must be 0-3, got {action}")

        delta = model.predict_delta(state, action)
        next_state = state + delta
        prev_dist = float(np.linalg.norm(state[:2] - self._goal_center[:2]))
        reward, done = self._compute_reward_and_done(
            next_state,
            collision=False,
            prev_goal_distance=prev_dist,
            update_progress=False,
        )
        return next_state, reward, done

    def simulate_model_batch(
        self,
        states: np.ndarray,
        actions: np.ndarray,
        model,
        *,
        return_disagreement: bool = False,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor] | tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Batched variant of `simulate_model` evaluated in imagination.

        Args:
            states: [B, state_dim] batch of current states (np.ndarray or torch.Tensor).
            actions: [B] batch of discrete action indices.
            model: Dynamics model exposing `predict_delta_batch`.
            return_disagreement: If True, also returns epistemic disagreement per sample.

        Returns:
            next_states: [B, state_dim] torch.Tensor
            rewards: [B] torch.Tensor
            dones: [B] torch.BoolTensor
            disagreement (optional): [B] torch.Tensor of epistemic scores.
        """
        if not hasattr(model, "predict_delta_batch"):
            raise AttributeError("Model must implement predict_delta_batch for batched rollout.")

        device = getattr(model, "device", None)
        if device is None and hasattr(model, "ens"):
            device = getattr(model.ens, "device", None)
        if device is None:
            device = torch.device("cpu")
        device = torch.device(device)

        states_t = states if isinstance(states, torch.Tensor) else torch.as_tensor(states, dtype=torch.float32, device=device)
        if states_t.ndim != 2:
            raise ValueError(f"`states` must be 2D [B, state_dim], got shape {tuple(states_t.shape)}")

        actions_t = actions if isinstance(actions, torch.Tensor) else torch.as_tensor(actions, dtype=torch.int64, device=device)
        if actions_t.ndim != 1 or actions_t.shape[0] != states_t.shape[0]:
            raise ValueError("`actions` must be 1D with same length as states batch.")

        with torch.no_grad():
            preds = model.predict_delta_batch(states_t, actions_t, sample=False)
            if isinstance(preds, tuple) and len(preds) == 2:
                deltas, disagreement = preds
            else:
                deltas = preds
                disagreement = torch.zeros(states_t.shape[0], dtype=torch.float32, device=device)

            next_states = states_t + deltas

            rewards_list: list[float] = []
            dones_list: list[bool] = []
            for ns in next_states.detach().cpu().numpy():
                reward_val, done_flag = self._compute_reward_and_done(
                    ns,
                    collision=False,
                    prev_goal_distance=None,
                    update_progress=False,
                )
                rewards_list.append(float(reward_val))
                dones_list.append(bool(done_flag))
            rewards = torch.as_tensor(rewards_list, dtype=torch.float32, device=device)
            dones = torch.as_tensor(dones_list, dtype=torch.bool, device=device)

        if return_disagreement:
            return next_states, rewards, dones, disagreement
        return next_states, rewards, dones

    def close(self):
        """Clean up PyBullet connection."""
        if self.cid is not None:
            p.disconnect(self.cid)
            self.cid = None

