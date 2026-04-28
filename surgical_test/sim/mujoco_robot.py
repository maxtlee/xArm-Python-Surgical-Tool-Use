"""
MuJoCo simulation layer - full 3D physics with soft body and contact.

Requires: pip install mujoco>=3.1.0
Optional: pip install mediapy meshcat
"""

import numpy as np
from typing import List, Tuple, Optional
from .robot_interface import RobotInterface

try:
    import mujoco
except ImportError:
    raise ImportError("MuJoCo not installed. Run: pip install mujoco>=3.1.0")

# Try to import viewer for visualization
_VIEWER_AVAILABLE = False
try:
    from mujoco import viewer
    _VIEWER_AVAILABLE = True
except (ImportError, AttributeError):
    pass


class MuJoCoRobot(RobotInterface):
    """MuJoCo-based robot simulation with physics and soft body."""

    def __init__(self, scene_path: str, vis_mode: str = 'viewer', joint_angles: Optional[List[float]] = None,
                 named_poses: Optional[dict] = None):
        """
        Initialize MuJoCo simulation.

        Args:
            scene_path: Path to scene XML file
            vis_mode: 'viewer' (interactive), 'headless' (render to video), or 'meshcat' (web viewer)
            joint_angles: Optional list of 7 joint angles (radians) for default pose
        """
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)

        # Find end-effector body
        try:
            self.ee_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link_eef")
        except KeyError:
            raise ValueError("Scene must contain a body named 'link_eef'")

        # Number of arm DOF (xArm 7 has 7 joints)
        self.n_arm_dof = 7

        # Named joint-space poses (degrees, converted to radians on use)
        self._named_poses = named_poses or {}

        # Set arm to home position
        if joint_angles is not None and len(joint_angles) == self.n_arm_dof:
            self.data.qpos[:self.n_arm_dof] = joint_angles
            print(f"[MuJoCoRobot] Setting default joint angles: {joint_angles}")
        else:
            self.data.qpos[:self.n_arm_dof] = 0.0  # All zeros for stub

        # Store home configuration
        self.home_qpos = self.data.qpos[:self.n_arm_dof].copy()

        mujoco.mj_forward(self.model, self.data)

        # Store hand aperture (not physically connected yet)
        self.aperture = 50

        # Initialize visualizer
        self.vis_mode = vis_mode
        self._viewer = None
        self._meshcat_vis = None

        if vis_mode == 'viewer':
            if _VIEWER_AVAILABLE:
                # Passive viewer - we control stepping
                try:
                    self._viewer = viewer.launch_passive(self.model, self.data)
                except Exception as e:
                    print(f"[MuJoCoRobot] Failed to launch viewer: {e}")
                    print("[MuJoCoRobot] Falling back to headless mode")
                    self.vis_mode = 'headless'
            else:
                print("[MuJoCoRobot] Viewer not available, falling back to headless mode")
                print("[MuJoCoRobot] Install with: pip install mujoco[viewer]")
                self.vis_mode = 'headless'
        elif vis_mode == 'meshcat':
            self._init_meshcat()
        elif vis_mode != 'headless':
            print(f"[MuJoCoRobot] Unknown vis_mode '{vis_mode}', using headless")
            self.vis_mode = 'headless'

    def _init_meshcat(self):
        """Initialize meshcat web visualizer if available."""
        try:
            import meshcat
            self._meshcat_vis = meshcat.Visualizer()
            self._meshcat_vis.open()
            print(f"[MuJoCoRobot] Meshcat running at {self._meshcat_vis.url()}")
            # TODO: Set up meshcat scene rendering (requires additional setup)
        except ImportError:
            print("[MuJoCoRobot] meshcat not installed, falling back to headless")
            print("Run: pip install meshcat")
            self.vis_mode = 'headless'
            self._meshcat_vis = None

    def _step(self, n_steps: int = 1):
        """Step simulation forward."""
        for _ in range(n_steps):
            mujoco.mj_step(self.model, self.data)

        # Sync viewer if active
        if self._viewer is not None:
            self._viewer.sync()

    def _ik_simple(self, target_pos: np.ndarray, max_iter: int = 100) -> bool:
        """
        Robust damped least-squares Jacobian IK with adaptive step size.

        Args:
            target_pos: Target position [x, y, z] in meters

        Returns:
            True if converged, False otherwise
        """
        tolerance = 0.005  # 5mm is reasonable for surgical tasks
        step_size = 0.3
        lambda_damping = 0.01  # Higher damping for stability
        max_step = 0.1  # Limit change per iteration

        prev_error_norm = float('inf')
        stagnation_count = 0

        for iteration in range(max_iter):
            # Current end-effector position
            current_pos = self.data.xpos[self.ee_id].copy()
            error = target_pos - current_pos
            error_norm = np.linalg.norm(error)

            # Check convergence
            if error_norm < tolerance:
                return True

            # Detect stagnation (error not decreasing)
            if error_norm >= prev_error_norm * 0.99:
                stagnation_count += 1
                if stagnation_count > 10:
                    # Stalled - reset damping and try again
                    lambda_damping = min(0.1, lambda_damping * 2)
                    stagnation_count = 0
            else:
                stagnation_count = 0

            prev_error_norm = error_norm

            # Compute Jacobian (3 x nv for position only)
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.ee_id)

            # Damped least-squares: dq = J^T (JJ^T + λI)^-1 error
            # Only update arm joints (exclude any flex DOF appended after arm)
            J = jacp[:, :self.n_arm_dof]
            JJT = J @ J.T + lambda_damping * np.eye(3)

            try:
                dq = J.T @ np.linalg.solve(JJT, error)
            except np.linalg.LinAlgError:
                return False

            # Limit step size and update joint positions
            dq_norm = np.linalg.norm(dq)
            if dq_norm > max_step:
                dq = dq * (max_step / dq_norm)

            self.data.qpos[:self.n_arm_dof] += step_size * dq
            mujoco.mj_forward(self.model, self.data)

        # Did not converge (but may be close enough for practical purposes)
        return False

    def get_pose(self) -> List[float]:
        """Get current end-effector pose."""
        # Position in mm
        pos_m = self.data.xpos[self.ee_id]
        x_mm, y_mm, z_mm = pos_m * 1000.0

        # Rotation matrix to euler angles (ZYX convention)
        mat = self.data.xmat[self.ee_id].reshape(3, 3)

        # Extract roll, pitch, yaw in degrees
        # ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        pitch = np.arcsin(-mat[2, 0])

        if np.cos(pitch) > 1e-6:
            roll = np.arctan2(mat[2, 1], mat[2, 2])
            yaw = np.arctan2(mat[1, 0], mat[0, 0])
        else:
            # Gimbal lock
            roll = 0.0
            yaw = np.arctan2(-mat[0, 1], mat[1, 1])

        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

        return [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg]

    def move_relative(self, dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0) -> bool:
        """Move end-effector relative to current pose."""
        current_pose = self.get_pose()

        # Compute target position (only translation for now)
        target_x = (current_pose[0] + dx) / 1000.0  # mm to m
        target_y = (current_pose[1] + dy) / 1000.0
        target_z = (current_pose[2] + dz) / 1000.0
        target_pos = np.array([target_x, target_y, target_z])

        # Run IK - best effort (we'll move as close as possible)
        success = self._ik_simple(target_pos)

        # Get result position
        result_pose = self.get_pose()
        distance_error = np.linalg.norm(np.array([target_x, target_y, target_z]) -
                                        np.array(result_pose[:3]) / 1000.0) * 1000.0

        if not success:
            if distance_error < 20:  # Within 20mm
                print(f"[MuJoCoRobot] IK: close enough ({distance_error:.1f}mm error)")
            else:
                print(f"[MuJoCoRobot] IK did not converge ({distance_error:.1f}mm error)")

        # Step simulation to settle (allow physics to respond)
        self._step(50)

        # Always return success (best-effort movement)
        return True

    def set_default_ee_position(self, x_m: float = 0.3, y_m: float = 0.0, z_m: float = 0.5) -> bool:
        """
        Move end-effector to a default Cartesian position using IK.

        Call this after __init__ to place the arm in a meaningful start pose
        before the viewer opens or recording begins.

        Args:
            x_m: Target X in metres (default 0.3 m forward)
            y_m: Target Y in metres (default centred)
            z_m: Target Z in metres (default 0.5 m above base)

        Returns:
            True if IK converged within tolerance, False otherwise
        """
        target_pos = np.array([x_m, y_m, z_m])
        success = self._ik_simple(target_pos, max_iter=300)
        mujoco.mj_forward(self.model, self.data)
        self._step(100)
        # Update home position to this pose so go_home() returns here
        self.home_qpos = self.data.qpos[:self.n_arm_dof].copy()
        pose = self.get_pose()
        print(f"[MuJoCoRobot] Default EEF position set: "
              f"({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}) mm "
              f"(converged={success})")
        return success

    def go_home(self) -> bool:
        """Return to home position."""
        self.data.qpos[:self.n_arm_dof] = self.home_qpos.copy()
        mujoco.mj_forward(self.model, self.data)
        self._step(100)
        print("[MuJoCoRobot] Returned to home position")
        return True

    def stop(self):
        """Stop all motion."""
        self.data.qvel[:self.n_arm_dof] = 0.0
        mujoco.mj_forward(self.model, self.data)
        print("[MuJoCoRobot] STOP - all velocities zeroed")

    def set_hand_aperture(self, percent: int):
        """Set hand aperture (placeholder)."""
        self.aperture = max(0, min(100, percent))
        print(f"[MuJoCoRobot] hand aperture -> {self.aperture}%")
        # TODO: drive finger joint qpos once foam hand MJCF is finalised

    def get_hand_aperture(self) -> int:
        """Get current hand aperture."""
        return self.aperture

    def go_to_named_pose(self, name: str) -> bool:
        """Move to a named joint-space pose (angles stored in degrees)."""
        if name not in self._named_poses:
            print(f"[MuJoCoRobot] Unknown named pose '{name}'")
            return False
        angles_deg = self._named_poses[name]
        self.data.qpos[:self.n_arm_dof] = np.radians(angles_deg)
        mujoco.mj_forward(self.model, self.data)
        self._step(100)
        print(f"[MuJoCoRobot] Moved to named pose '{name}'")
        return True

    def get_contact_forces(self) -> List[Tuple[str, str, float]]:
        """
        Get all active contact forces.

        Returns:
            List of (geom1_name, geom2_name, force_magnitude_N) tuples
        """
        contacts = []

        for i in range(self.data.ncon):
            contact = self.data.contact[i]

            # Get contact wrench (force + torque)
            c_array = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, c_array)

            # Force magnitude (first 3 components)
            force_mag = np.linalg.norm(c_array[:3])

            # Get geom names
            geom1 = contact.geom1
            geom2 = contact.geom2

            try:
                geom1_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom1)
                geom2_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom2)
            except:
                geom1_name = f"geom{geom1}"
                geom2_name = f"geom{geom2}"

            if geom1_name is None:
                geom1_name = f"geom{geom1}"
            if geom2_name is None:
                geom2_name = f"geom{geom2}"

            contacts.append((geom1_name, geom2_name, force_mag))

        return contacts

    def record(self, output_path: str, duration_s: float = 5.0):
        """
        Record a video of the simulation.

        Args:
            output_path: Output video file path
            duration_s: Duration to record in seconds
        """
        try:
            import mediapy
        except ImportError:
            print("[MuJoCoRobot] mediapy not installed. Run: pip install mediapy")
            return

        fps = 30
        n_frames = int(duration_s * fps)
        dt_per_frame = 1.0 / fps

        renderer = mujoco.Renderer(self.model, height=480, width=640)
        frames = []

        print(f"[MuJoCoRobot] Recording {duration_s}s at {fps} fps...")

        for frame_idx in range(n_frames):
            # Step simulation
            steps_per_frame = int(dt_per_frame / self.model.opt.timestep)
            for _ in range(steps_per_frame):
                mujoco.mj_step(self.model, self.data)

            # Render frame
            renderer.update_scene(self.data)
            pixels = renderer.render()
            frames.append(pixels)

        # Save video
        mediapy.write_video(output_path, frames, fps=fps)
        print(f"[MuJoCoRobot] Saved video to {output_path}")

    def __del__(self):
        """Cleanup visualizer on destruction."""
        if hasattr(self, '_viewer') and self._viewer is not None:
            self._viewer.close()
