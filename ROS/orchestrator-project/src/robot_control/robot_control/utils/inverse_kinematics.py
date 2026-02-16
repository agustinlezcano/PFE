import math
import numpy as np


class RobotConfiguration:
    """Encapsulates robot DH parameters and configuration."""
    
    def __init__(self):
        """Initialize robot configuration with DH parameters and link lengths."""
        # Link lengths
        self.L1 = 0.078
        self.L2 = 0.135
        self.L3 = 0.147
        self.L4 = 0.0253
        self.tool_z = 0.04115
        
        # DH parameters: [theta, d, a, alpha, offset]
        self.dh = [
            [0, self.L1, 0, math.pi / 2, 0],
            [0, 0, self.L2, 0, 0],
            [0, 0, self.L3, 0, 0],
            [0, 0, self.L4, 0, 0]
        ]
        
        # Joint offsets
        self.offset = [0, math.pi / 2, -math.pi / 2, 0]
        
        # Base and Tool transformations
        self.Base = np.eye(4)
        self.Tool = np.eye(4)


class KinematicsSolver:
    """Solves forward and inverse kinematics for the robot."""
    
    def __init__(self, config):
        """
        Initialize kinematics solver.
        
        Args:
            config: RobotConfiguration instance
        """
        self.config = config
    
    @staticmethod
    def inv_homog(T):
        """
        Compute inverse of homogeneous transformation matrix.
        
        Args:
            T: 4x4 homogeneous transformation matrix
            
        Returns:
            4x4 inverse transformation matrix
        """
        R = T[0:3, 0:3]
        p = T[0:3, 3]
        
        iT = np.eye(4)
        iT[0:3, 0:3] = R.T
        iT[0:3, 3] = -R.T @ p
        
        return iT
    
    def forward_kinematics(self, dh_params, q):
        """
        Calculate transformation matrix using DH parameters.
        
        Args:
            dh_params: Single row of DH parameters [theta, d, a, alpha, offset]
            q: Joint angle
            
        Returns:
            4x4 transformation matrix
        """
        a = dh_params[2]
        d = dh_params[1]
        alpha = dh_params[3]
        
        cq = math.cos(q)
        sq = math.sin(q)
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        
        T = np.array([
            [cq, -sq*ca, sq*sa, a*cq],
            [sq, cq*ca, -cq*sa, a*sq],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ], dtype=float)
        return T
    
    def inverse_kinematics(self, T_obj_in):
        """
        Compute inverse kinematics for target pose.
        
        Args:
            T_obj_in: 4x4 target transformation matrix
            
        Returns:
            List of 3 motor angles in degrees [theta_m1, theta_m2, theta_m3]
        """
        dh = self.config.dh
        offset = self.config.offset
        Base = self.config.Base
        Tool = self.config.Tool
        
        # Decouple tool and base
        T_obj = self.inv_homog(Base) @ T_obj_in @ self.inv_homog(Tool)
        
        # Separate wrist from end effector
        Pf = T_obj[0:3, 3]
        ang_base = math.atan2(Pf[1], Pf[0])
        versor = [math.cos(ang_base), math.sin(ang_base), 0]
        Pm = [Pf[i] - dh[3][2] * versor[i] for i in range(3)]  # wrist position
        
        # REACHABLE CONFIGURATION: Elbow up - right (physical limitations)
        ## q1
        q1 = ang_base
        
        ## q2
        T01 = self.forward_kinematics(dh[0], q1)
        p2m = self.inv_homog(T01) @ np.array([Pm[0], Pm[1], Pm[2], 1.0])
        
        B = math.atan2(p2m[0], p2m[1])
        d = math.sqrt(p2m[0]**2 + p2m[1]**2)  # hypotenuse P12 to Pm
        C = math.acos((d**2 + dh[1][2]**2 - dh[2][2]**2) / (2 * d * dh[1][2]))
        # General: B +/- C
        # Elbow up configuration: B-C
        q2 = math.pi / 2 - (B - C)  # pi/2 is the offset from dh[1][4]
        
        ## q3
        phi = math.acos((dh[1][2]**2 + dh[2][2]**2 - d**2) / (2 * dh[1][2] * dh[2][2]))  # angle between L2 and L3
        q3 = -(math.pi - phi)  # Valid for elbow up
        
        ## Output
        qf = [q1 - offset[0], q2 - offset[1], q3 - offset[2]]
        theta_m1 = math.degrees(qf[0])
        theta_m2 = math.degrees(math.pi / 2 - qf[1])   # Horizontal axis to the left
        theta_m3 = math.degrees(-(qf[1] + qf[2]))  # Horizontal axis to the left
        q_motores = [theta_m1, theta_m2, theta_m3]
        
        return q_motores


# Legacy functions for backward compatibility
def invHomog(T):
    """Legacy function. Use KinematicsSolver.inv_homog() instead."""
    R = T[0:3, 0:3]
    p = T[0:3, 3]

    iT = np.eye(4)
    iT[0:3, 0:3] = R.T
    iT[0:3, 3]   = -R.T @ p

    return iT

def ikine(dh, T_obj_in, Base, Tool, offset):
    """Legacy function. Use KinematicsSolver.inverse_kinematics() instead."""
    config = RobotConfiguration()
    solver = KinematicsSolver(config)
    return solver.inverse_kinematics(T_obj_in)


def DH_Robot():
    """Legacy function. Use RobotConfiguration class instead."""
    config = RobotConfiguration()
    return config.dh, config.offset, config.Tool, config.Base, config.tool_z


def A_dh(dh, q):
    """Legacy function. Use KinematicsSolver.forward_kinematics() instead."""
    config = RobotConfiguration()
    solver = KinematicsSolver(config)
    return solver.forward_kinematics(dh, q)

def main():
    # Using new OOP approach
    config = RobotConfiguration()
    solver = KinematicsSolver(config)
    
    # Test coordinates
    x, y, z = 0.1104, 0.1392, 0.014
    T_obj_in = np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z + config.tool_z],
        [0, 0, 0, 1]
    ], dtype=float)

    q_motores = solver.inverse_kinematics(T_obj_in)
    print(q_motores)

if __name__ == "__main__":
    main()