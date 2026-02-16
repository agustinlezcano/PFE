import numpy as np


class WorkspaceValidator:
    """Validates robot trajectories against workspace limits."""
    
    def __init__(self, csv_path="Tabla_angulos_Robot.csv"):
        """
        Initialize workspace validator.
        
        Args:
            csv_path: Path to CSV file with workspace lookup table
        """
        self.csv_path = csv_path
        self.q1_min = -45  # degrees
        self.q1_max = 90   # degrees
        self._load_workspace_table()
    
    def _load_workspace_table(self):
        """Load workspace lookup table from CSV file."""
        try:
            T = np.genfromtxt(self.csv_path, delimiter=';', dtype=int)
            self.workspace_matrix = T[1:, 1:]
            self.offset_rows = int(T[1, 0] - 1)
            self.offset_cols = int(T[0, 1] - 1)
            self.n_rows, self.n_cols = self.workspace_matrix.shape
        except Exception as e:
            print(f"Error loading workspace table: {e}")
            self.workspace_matrix = np.zeros((1, 1))
            self.offset_rows = 0
            self.offset_cols = 0
            self.n_rows = 1
            self.n_cols = 1
    
    def validate_trajectory(self, q_trajectory):
        """
        Validate if trajectory is within workspace limits.
        
        Args:
            q_trajectory: Nx3 array of joint angles [q1, q2, q3] in degrees
            
        Returns:
            bool: True if trajectory is valid, False otherwise
        """
        # Validate q1 limits
        q1 = q_trajectory[:, 0]
        if np.min(q1) < self.q1_min or np.max(q1) > self.q1_max:
            print("No es posible realizar la trayectoria, no verifica el GDL 1")
            return False
        
        # Validate table bounds
        max_q3 = int(np.ceil(np.max(q_trajectory[:, 2]))) - self.offset_rows
        max_q2 = int(np.ceil(np.max(q_trajectory[:, 1]))) - self.offset_cols
        
        if max_q3 >= self.n_rows or max_q2 >= self.n_cols:
            print("No es posible realizar la trayectoria, fuera del WS articular")
            return False
        
        # Validate point by point
        for i in range(q_trajectory.shape[0]):
            row = int(np.ceil(q_trajectory[i, 2])) - self.offset_rows
            col = int(np.ceil(q_trajectory[i, 1])) - self.offset_cols
            
            # Extra safety check
            if row < 0 or col < 0 or row >= self.n_rows or col >= self.n_cols:
                print("No es posible realizar la trayectoria, fuera del WS articular")
                return False
            
            if self.workspace_matrix[row, col] == 0:
                print("No es posible realizar la trayectoria, no verifican los GDL 2 y 3")
                return False
        
        print("Trayectoria correcta, todos los puntos verifican")
        return True


# Legacy function for backward compatibility
def verificar_workspace_articular(q_eslabon, csv_file="Tabla_angulos_Robot.csv"):
    """
    Legacy function. Use WorkspaceValidator class instead.
    
    Args:
        q_eslabon: Nx3 array of joint angles [q1, q2, q3] in degrees
        csv_file: Path to CSV workspace table
        
    Returns:
        bool: True if valid, False otherwise
    """
    validator = WorkspaceValidator(csv_path=csv_file)
    return validator.validate_trajectory(q_eslabon)
