"""
Módulo de validación de límites articulares y de espacio de trabajo.

Proporciona validación para:
- Límites articulares (ángulos en radianes)
- Límites de espacio de trabajo (coordenadas XYZ)
"""

from typing import Tuple, Optional


class JointLimits:
    """Define los límites articulares del robot."""
    
    def __init__(self):
        """
        Inicializa límites articulares para un robot 3DOF.
        Ajusta estos valores según las especificaciones de tu robot.
        """
        # Límites articulares en radianes
        # Q1, Q2, Q3
        self.q_min = [-3.14159, -3.14159, -3.14159]  # -180°
        self.q_max = [3.14159, 3.14159, 3.14159]     # +180°
        
        # Velocidad angular máxima (rad/s)
        self.q_vel_max = [1.57, 1.57, 1.57]  # ~90°/s
        
        # Aceleración angular máxima (rad/s²)
        self.q_acc_max = [3.14, 3.14, 3.14]  # ~180°/s²
    
    def set_limits(self, q_min: list, q_max: list):
        """
        Establece límites articulares personalizados.
        
        Args:
            q_min: Lista de límites mínimos [q1_min, q2_min, q3_min]
            q_max: Lista de límites máximos [q1_max, q2_max, q3_max]
        """
        if len(q_min) != 3 or len(q_max) != 3:
            raise ValueError("Los límites deben tener 3 elementos (DOF=3)")
        
        self.q_min = q_min
        self.q_max = q_max


class WorkspaceLimits:
    """Define los límites del espacio de trabajo (XYZ)."""
    
    def __init__(self):
        """
        Inicializa límites de espacio de trabajo.
        Ajusta estos valores según el alcance físico del robot.
        """
        # Límites de posición en metros
        self.x_min = 0.0
        self.x_max = 0.5
        self.y_min = 0.0
        self.y_max = 0.5
        self.z_min = 0.0
        self.z_max = 0.5
        
        # Velocidad máxima en espacio Cartesiano (m/s)
        self.v_max = 0.5
        
        # Aceleración máxima en espacio Cartesiano (m/s²)
        self.a_max = 1.0
    
    def set_limits(self, x_min: float, x_max: float, 
                   y_min: float, y_max: float,
                   z_min: float, z_max: float):
        """
        Establece límites de espacio de trabajo personalizados.
        
        Args:
            x_min, x_max: Límites en X (metros)
            y_min, y_max: Límites en Y (metros)
            z_min, z_max: Límites en Z (metros)
        """
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max


class LimitsValidator:
    """
    Validador de límites para el robot.
    
    Proporciona métodos para validar:
    - Posiciones articulares (Q-space)
    - Posiciones Cartesianas (X-Y-Z)
    - Trayectorias completas
    """
    
    def __init__(self):
        """Inicializa el validador con límites por defecto."""
        self.joint_limits = JointLimits()
        self.workspace_limits = WorkspaceLimits()
    
    def validate_joint_position(self, q1: float, q2: float, q3: float) -> Tuple[bool, str]:
        """
        Valida una posición articular.
        
        Args:
            q1, q2, q3: Ángulos articulares en radianes
            
        Returns:
            Tupla (válido, mensaje)
        """
        joints = [q1, q2, q3]
        joint_names = ['Q1', 'Q2', 'Q3']
        
        for i, (q, name) in enumerate(zip(joints, joint_names)):
            if q < self.joint_limits.q_min[i]:
                return False, f"{name} por debajo del mínimo: {q:.4f} < {self.joint_limits.q_min[i]:.4f}"
            if q > self.joint_limits.q_max[i]:
                return False, f"{name} por encima del máximo: {q:.4f} > {self.joint_limits.q_max[i]:.4f}"
        
        return True, "Posición articular válida"
    
    def validate_cartesian_position(self, x: float, y: float, z: float) -> Tuple[bool, str]:
        """
        Valida una posición Cartesiana.
        
        Args:
            x, y, z: Coordenadas en metros
            
        Returns:
            Tupla (válido, mensaje)
        """
        if x < self.workspace_limits.x_min or x > self.workspace_limits.x_max:
            return False, f"X fuera de rango: {x:.4f} (permitido: [{self.workspace_limits.x_min}, {self.workspace_limits.x_max}])"
        
        if y < self.workspace_limits.y_min or y > self.workspace_limits.y_max:
            return False, f"Y fuera de rango: {y:.4f} (permitido: [{self.workspace_limits.y_min}, {self.workspace_limits.y_max}])"
        
        if z < self.workspace_limits.z_min or z > self.workspace_limits.z_max:
            return False, f"Z fuera de rango: {z:.4f} (permitido: [{self.workspace_limits.z_min}, {self.workspace_limits.z_max}])"
        
        return True, "Posición Cartesiana válida"
    
    def validate_trajectory_point(self, x: float = None, y: float = None, z: float = None,
                                  q1: float = None, q2: float = None, q3: float = None) -> Tuple[bool, str]:
        """
        Valida un punto de trayectoria (puede ser Cartesiano o articular).
        
        Args:
            x, y, z: Coordenadas Cartesianas (opcionales)
            q1, q2, q3: Coordenadas articulares (opcionales)
            
        Returns:
            Tupla (válido, mensaje)
        """
        if x is not None and y is not None and z is not None:
            return self.validate_cartesian_position(x, y, z)
        
        if q1 is not None and q2 is not None and q3 is not None:
            return self.validate_joint_position(q1, q2, q3)
        
        return False, "Debe proporcionar coordenadas Cartesianas (XYZ) o articulares (Q1Q2Q3)"
    
    def get_joint_limits(self) -> dict:
        """Retorna un diccionario con los límites articulares."""
        return {
            'q_min': self.joint_limits.q_min,
            'q_max': self.joint_limits.q_max,
            'q_vel_max': self.joint_limits.q_vel_max,
            'q_acc_max': self.joint_limits.q_acc_max,
        }
    
    def get_workspace_limits(self) -> dict:
        """Retorna un diccionario con los límites de espacio de trabajo."""
        return {
            'x_range': [self.workspace_limits.x_min, self.workspace_limits.x_max],
            'y_range': [self.workspace_limits.y_min, self.workspace_limits.y_max],
            'z_range': [self.workspace_limits.z_min, self.workspace_limits.z_max],
            'v_max': self.workspace_limits.v_max,
            'a_max': self.workspace_limits.a_max,
        }
