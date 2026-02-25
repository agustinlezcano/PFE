import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from .inverse_kinematics import RobotConfiguration, KinematicsSolver
from .Bspline import PathGenerator
from .verify_point_inside_ws import WorkspaceValidator


class TrajectoryPlanner:
    """Plans time-optimal trajectories with velocity and acceleration constraints."""
    
    def __init__(self, kinematics_solver, path_generator, workspace_validator,
                 qd_max=None, qdd_max=None):
        """
        Initialize trajectory planner.
        
        Args:
            kinematics_solver: KinematicsSolver instance
            path_generator: PathGenerator instance
            workspace_validator: WorkspaceValidator instance
            qd_max: Max joint velocities [deg/s] (default: [39.5833, 84.375, 84.375])
            qdd_max: Max joint accelerations [deg/s²] (default: [300.0, 300.0, 300.0])
        """
        self.kinematics = kinematics_solver
        self.path_generator = path_generator
        self.validator = workspace_validator
        
        # Joint constraints
        self.qd_max = qd_max if qd_max is not None else np.array([39.5833, 84.375, 84.375])
        self.qdd_max = qdd_max if qdd_max is not None else np.array([300.0, 300.0, 300.0])
        
        # Quintic polynomial constants
        self.C1 = 1.875    # max(s_dot)
        self.C2 = 5.77     # max(s_ddot)
    
    def _quintic_time_scaling(self, T, N):
        """
        Generate quintic time scaling profile.
        
        Args:
            T: Total time duration
            N: Number of points
            
        Returns:
            Tuple of (t, s, s_dot, s_ddot) arrays
        """
        t = np.linspace(0, T, N)
        tau = t / T
        
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        s_dot = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
        s_ddot = (60*tau - 180*tau**2 + 120*tau**3) / T**2
        
        return t, s, s_dot, s_ddot
    
    def plan(self, waypoints, plot=False, dt=None):
        """
        Plan complete trajectory from waypoints.
        
        Args:
            waypoints: Nx3 array of Cartesian waypoints [x, y, z]
            plot: If True, display trajectory plots
            dt: Optional time step [s]. If specified the planner will adjust the
                number of trajectory points such that the resulting
                trajectory interval is approximately `dt`. The total time `T`
                is still computed based on kinematic constraints.
        
        Returns:
            Tuple of (q, qd, T, N) where:
                q: Joint positions (Nx3)
                qd: Joint velocities (Nx3)
                T: Total trajectory time
                N: Number of trajectory points (may change if dt is provided)
        """
        # helper to compute geometry given lists of coordinates
        def _compute_geometry(x_arr, y_arr, z_arr):
            Nloc = len(x_arr)
            # inverse kinematics
            q_es = np.zeros((Nloc, 3))
            for kk in range(Nloc):
                T_obj = np.array([
                    [1, 0, 0, x_arr[kk]],
                    [0, 1, 0, y_arr[kk]],
                    [0, 0, 1, z_arr[kk] + self.kinematics.config.tool_z],
                    [0, 0, 0, 1]
                ])
                q_es[kk, :] = self.kinematics.inverse_kinematics(T_obj)

            s_g = np.linspace(0, 1, Nloc)
            ds_loc = s_g[1] - s_g[0]
            dq_ds_loc = np.gradient(q_es, ds_loc, axis=0)
            ddq_ds_loc = np.gradient(dq_ds_loc, ds_loc, axis=0)

            return q_es, s_g, dq_ds_loc, ddq_ds_loc

        # initial spline generation with default resolution
        x, y, z = self.path_generator.generate_bspline(waypoints)
        N = len(x)
        q_eslabon, s_grid, dq_ds, ddq_ds = _compute_geometry(x, y, z)

        # geometric maximums
        dqds_max = np.max(np.abs(dq_ds), axis=0)
        ddqds_max = np.max(np.abs(ddq_ds), axis=0)

        # calculate time based on constraints
        T_vel = np.max(self.C1 * dqds_max / self.qd_max)
        T_acc = np.max(np.sqrt(self.C2 * ddqds_max / self.qdd_max))
        T = max(T_vel, T_acc)

        print(f"Tiempo total seleccionado T = {T:.4f} s")
        print(f"Delta T (coarse) = {T/(N-1):.4f} s")

        # if user asked for specific time step, adjust number of points
        if dt is not None and dt > 0:
            N_new = int(np.ceil(T / dt)) + 1
            if N_new != N:
                print(f"Recalculando con dt={dt:.4f}s -> N={N_new}")
                x, y, z = self.path_generator.generate_bspline(waypoints, resolution=N_new)
                N = N_new
                q_eslabon, s_grid, dq_ds, ddq_ds = _compute_geometry(x, y, z)

        # time scaling and interpolation functions
        t, s, s_dot, s_ddot = self._quintic_time_scaling(T, N)
        q_fun = interp1d(s_grid, q_eslabon, axis=0, kind='cubic')
        dqds_fun = interp1d(s_grid, dq_ds, axis=0, kind='cubic')
        ddqds_fun = interp1d(s_grid, ddq_ds, axis=0, kind='cubic')

        # final joint trajectory arrays
        q = np.zeros((N, 3))
        qd = np.zeros((N, 3))
        qdd = np.zeros((N, 3))

        for k in range(N):
            q_s = q_fun(s[k])
            dqds_s = dqds_fun(s[k])
            ddqds_s = ddqds_fun(s[k])

            q[k, :] = q_s
            qd[k, :] = dqds_s * s_dot[k]
            qdd[k, :] = ddqds_s * s_dot[k]**2 + dqds_s * s_ddot[k]

        # validate trajectory
        isQOk = self.validator.validate_trajectory(q)

        print("Velocidad máxima real [deg/s]:", np.max(np.abs(qd), axis=0))
        print("Aceleración máxima real [deg/s^2]:", np.max(np.abs(qdd), axis=0))

        if plot:
            self._plot_trajectory(t, q, qd, qdd, s, s_dot, s_ddot)

        return q, qd, T, N
    
    def _plot_trajectory(self, t, q, qd, qdd, s, s_dot, s_ddot):
        """Plot trajectory profiles."""
        # Joint trajectories
        plt.figure()
        plt.subplot(3, 1, 1)
        plt.plot(t, q[:, 0], 'r-', label='q1')
        plt.plot(t, q[:, 1], 'g-', label='q2')
        plt.plot(t, q[:, 2], 'b-', label='q3')
        plt.ylabel('Ángulo [deg]')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(3, 1, 2)
        plt.plot(t, qd[:, 0], 'r-', label='qd1')
        plt.plot(t, qd[:, 1], 'g-', label='qd2')
        plt.plot(t, qd[:, 2], 'b-', label='qd3')
        plt.ylabel('Velocidad [deg/s]')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(3, 1, 3)
        plt.plot(t, qdd[:, 0], 'r-', label='qdd1')
        plt.plot(t, qdd[:, 1], 'g-', label='qdd2')
        plt.plot(t, qdd[:, 2], 'b-', label='qdd3')
        plt.ylabel('Aceleración [deg/s²]')
        plt.xlabel('Tiempo [s]')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()
        
        # Time scaling
        plt.figure()
        plt.subplot(3, 1, 1)
        plt.plot(t, s, 'k-')
        plt.ylabel('s')
        plt.grid(True)
        
        plt.subplot(3, 1, 2)
        plt.plot(t, s_dot, 'k-')
        plt.ylabel('ṡ')
        plt.grid(True)
        
        plt.subplot(3, 1, 3)
        plt.plot(t, s_ddot, 'k-')
        plt.ylabel('s̈')
        plt.xlabel('Tiempo [s]')
        plt.grid(True)
        plt.tight_layout()
        plt.show()


# Legacy functions for backward compatibility
def quintic_time_scaling(T, N):
    """Legacy function. Use TrajectoryPlanner._quintic_time_scaling() instead."""
    planner = TrajectoryPlanner(None, None, None)
    return planner._quintic_time_scaling(T, N)

def generate_trajectory():
    """Legacy function. Use TrajectoryPlanner.plan() instead."""
    # Setup
    config = RobotConfiguration()
    kinematics = KinematicsSolver(config)
    path_gen = PathGenerator(resolution=200, smoothing=0.5, degree=3)
    validator = WorkspaceValidator()
    planner = TrajectoryPlanner(kinematics, path_gen, validator)
    
    # Waypoints
    Pxyz = np.array([
        [0.15275, 0.15275, 0.05],
        [0.15275, 0.0, 0.18],
        [0.15275, -0.15275, 0.05]
    ])
    
    # example of requesting a 10 ms time interval (dt)
    return planner.plan(Pxyz, plot=True, dt=0.050)


if __name__ == "__main__":
    # Create instances
    config = RobotConfiguration()
    kinematics = KinematicsSolver(config)
    path_generator = PathGenerator(resolution=50, smoothing=0.5, degree=3)
    validator = WorkspaceValidator()
    planner = TrajectoryPlanner(kinematics, path_generator, validator)
    
    # Define waypoints
    waypoints = np.array([
        [0.15275, 0.15275, 0.05],
        [0.15275, 0.0, 0.18],
        [0.15275, -0.15275, 0.05]
    ])
    
    # Plan and plot trajectory
    q, qd, T, N = planner.plan(waypoints, plot=True, dt=0.050)
    print(f"Planned trajectory with T={T:.4f}s and N={N} points")