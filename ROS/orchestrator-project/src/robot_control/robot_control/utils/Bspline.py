import numpy as np
from scipy import interpolate


class PathGenerator:
    """Generates smooth B-spline trajectories from waypoints."""
    
    def __init__(self, resolution=200, smoothing=0.5, degree=3):
        """
        Initialize PathGenerator.
        
        Args:
            resolution: Number of interpolated points (default: 200)
            smoothing: Smoothing factor for B-spline (default: 0.5)
            degree: Degree of B-spline curve (default: 3 for cubic)
        """
        self.resolution = resolution
        self.smoothing = smoothing
        self.degree = degree
    
    def generate_bspline(self, waypoints=None):
        """
        Generate smooth B-spline trajectory from waypoints.
        
        Args:
            waypoints: Nx3 numpy array of [x, y, z] control points
            
        Returns:
            Tuple of (x, y, z) arrays with interpolated trajectory points
        """
        if waypoints is None:
            raise ValueError("waypoints cannot be None")
        
        # Separate coordinates for splprep
        x, y, z = waypoints.T
        
        # Define weights: high weight on endpoints, lower on middle points for smoothing
        W = np.ones(len(x))
        W[0] = 1000   # Initial endpoint
        W[-1] = 1000  # Final endpoint
        
        # Calculate B-Spline (tck contains: knots, coefficients, and degree)
        if len(waypoints) < 4:
            K = len(waypoints) - 1  # Maximum allowed degree
            S = 0  # No smoothing
        else:
            K = self.degree
            S = self.smoothing
        
        tck, u = interpolate.splprep([x, y, z], s=S, k=K, w=W)
        
        # Evaluate trajectory at fine resolution for smooth path
        u_fine = np.linspace(0, 1, self.resolution)
        x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)
        
        return x_fine, y_fine, z_fine


# Legacy function for backward compatibility
def b_spline_trajectory(Pxyz=None):
    """
    Legacy function. Use PathGenerator class instead.
    
    Args:
        Pxyz: Nx3 numpy array of waypoints
        
    Returns:
        Tuple of (x, y, z) interpolated arrays
    """
    if Pxyz is None:
        raise ValueError("Pxyz cannot be None")
    
    generator = PathGenerator(resolution=200, smoothing=0.5, degree=3)
    return generator.generate_bspline(Pxyz)

