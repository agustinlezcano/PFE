"""
Módulo para leer y procesar archivos CSV de trayectorias.

Permite cargar trayectorias desde archivos CSV con formato:
    - Cartesiano: x, y, z
    - Articular: q1, q2, q3
    - Mixto: x, y, z, q1, q2, q3
"""

import csv
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from .limits_validator import LimitsValidator


class TrajectoryPoint:
    """Representa un punto de trayectoria."""
    
    def __init__(self, point_id: int = 0, 
                 x: Optional[float] = None, y: Optional[float] = None, z: Optional[float] = None,
                 q1: Optional[float] = None, q2: Optional[float] = None, q3: Optional[float] = None,
                 **kwargs):
        """
        Inicializa un punto de trayectoria.
        
        Args:
            point_id: ID del punto (número de fila)
            x, y, z: Coordenadas Cartesianas (opcionales)
            q1, q2, q3: Coordenadas articulares (opcionales)
            **kwargs: Parámetros adicionales (velocidad, aceleración, etc.)
        """
        self.id = point_id
        self.x = x
        self.y = y
        self.z = z
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.extra = kwargs
    
    def is_cartesian(self) -> bool:
        """Retorna True si el punto tiene coordenadas Cartesianas."""
        return self.x is not None and self.y is not None and self.z is not None
    
    def is_joint(self) -> bool:
        """Retorna True si el punto tiene coordenadas articulares."""
        return self.q1 is not None and self.q2 is not None and self.q3 is not None
    
    def to_dict(self) -> dict:
        """Convierte el punto a diccionario."""
        return {
            'id': self.id,
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'q1': self.q1,
            'q2': self.q2,
            'q3': self.q3,
            **self.extra
        }
    
    def __repr__(self) -> str:
        if self.is_cartesian():
            return f"TrajectoryPoint(id={self.id}, X={self.x:.4f}, Y={self.y:.4f}, Z={self.z:.4f})"
        elif self.is_joint():
            return f"TrajectoryPoint(id={self.id}, Q1={self.q1:.4f}, Q2={self.q2:.4f}, Q3={self.q3:.4f})"
        else:
            return f"TrajectoryPoint(id={self.id})"


class CSVTrajectoryReader:
    """
    Lee archivos CSV de trayectorias y valida puntos.
    
    Formato esperado del CSV:
    - Opción 1 (Cartesiano): x,y,z
    - Opción 2 (Articular): q1,q2,q3
    - Opción 3 (Mixto): x,y,z,q1,q2,q3
    - Opción 4 (Con headers): Header1,Header2,Header3 (con nombres personalizados)
    """
    
    def __init__(self, validator: Optional[LimitsValidator] = None):
        """
        Inicializa el lector de CSV.
        
        Args:
            validator: Instancia de LimitsValidator para validar puntos
        """
        self.validator = validator or LimitsValidator()
        self.trajectory: List[TrajectoryPoint] = []
        self.header: List[str] = []
    
    def read_file(self, filepath: str, has_header: bool = False, 
                  delimiter: str = ',', skip_empty: bool = True) -> Tuple[bool, str]:
        """
        Lee un archivo CSV de trayectoria.
        
        Args:
            filepath: Ruta del archivo CSV
            has_header: Si la primera fila es encabezado
            delimiter: Delimitador del CSV (por defecto ',')
            skip_empty: Si salta líneas vacías
            
        Returns:
            Tupla (éxito, mensaje)
        """
        try:
            path = Path(filepath)
            if not path.exists():
                return False, f"Archivo no encontrado: {filepath}"
            
            if not path.suffix.lower() == '.csv':
                return False, f"Formato inválido. Se espera .csv, se encontró {path.suffix}"
            
            self.trajectory = []
            self.header = []
            
            with open(filepath, 'r', newline='', encoding='utf-8') as csvfile:
                reader = csv.reader(csvfile, delimiter=delimiter)
                
                row_num = 0
                for idx, row in enumerate(reader):
                    # Saltar líneas vacías
                    if skip_empty and not any(row):
                        continue
                    
                    # Procesar encabezado
                    if idx == 0 and has_header:
                        self.header = row
                        continue
                    
                    row_num += 1
                    success, message = self._parse_row(row, row_num)
                    
                    if not success:
                        return False, f"Error en fila {row_num}: {message}"
            
            return True, f"Trayectoria cargada con {len(self.trajectory)} puntos"
        
        except Exception as e:
            return False, f"Error al leer archivo: {str(e)}"
    
    def _parse_row(self, row: List[str], row_num: int) -> Tuple[bool, str]:
        """
        Parsea una fila del CSV.
        
        Args:
            row: Lista de valores de la fila
            row_num: Número de fila (para reportes de error)
            
        Returns:
            Tupla (éxito, mensaje)
        """
        try:
            # Convertir strings a floats
            values = []
            for val in row:
                if val.strip() == '':
                    values.append(None)
                else:
                    values.append(float(val.strip()))
            
            # Detectar formato basado en cantidad de columnas
            if len(values) == 3:  # Cartesiano (X, Y, Z)
                point = TrajectoryPoint(row_num, x=values[0], y=values[1], z=values[2])
            
            elif len(values) == 6:  # Mixto (X, Y, Z, Q1, Q2, Q3)
                point = TrajectoryPoint(row_num, 
                                       x=values[0], y=values[1], z=values[2],
                                       q1=values[3], q2=values[4], q3=values[5])
            
            else:
                return False, f"Número de columnas inválido: {len(values)} (esperado 3 o 6)"
            
            # Validar punto
            if point.is_cartesian():
                valid, msg = self.validator.validate_cartesian_position(point.x, point.y, point.z)
                if not valid:
                    return False, f"Posición Cartesiana inválida: {msg}"
            
            if point.is_joint():
                valid, msg = self.validator.validate_joint_position(point.q1, point.q2, point.q3)
                if not valid:
                    return False, f"Posición articular inválida: {msg}"
            
            self.trajectory.append(point)
            return True, "Punto válido"
        
        except ValueError as e:
            return False, f"Error al convertir valores a números: {str(e)}"
    
    def get_trajectory(self) -> List[TrajectoryPoint]:
        """Retorna la trayectoria cargada."""
        return self.trajectory
    
    def get_trajectory_as_dicts(self) -> List[Dict]:
        """Retorna la trayectoria como lista de diccionarios."""
        return [point.to_dict() for point in self.trajectory]
    
    def filter_cartesian(self) -> List[TrajectoryPoint]:
        """Retorna solo los puntos con coordenadas Cartesianas."""
        return [p for p in self.trajectory if p.is_cartesian()]
    
    def filter_joint(self) -> List[TrajectoryPoint]:
        """Retorna solo los puntos con coordenadas articulares."""
        return [p for p in self.trajectory if p.is_joint()]
    
    def validate_trajectory(self) -> Tuple[bool, List[str]]:
        """
        Valida todos los puntos de la trayectoria.
        
        Returns:
            Tupla (válido, lista_de_errores)
        """
        errors = []
        
        for point in self.trajectory:
            if point.is_cartesian():
                valid, msg = self.validator.validate_cartesian_position(point.x, point.y, point.z)
                if not valid:
                    errors.append(f"Punto {point.id}: {msg}")
            
            if point.is_joint():
                valid, msg = self.validator.validate_joint_position(point.q1, point.q2, point.q3)
                if not valid:
                    errors.append(f"Punto {point.id}: {msg}")
        
        return len(errors) == 0, errors
    
    def save_trajectory(self, filepath: str, format: str = 'csv') -> Tuple[bool, str]:
        """
        Guarda la trayectoria en un archivo.
        
        Args:
            filepath: Ruta del archivo destino
            format: Formato ('csv' o 'json')
            
        Returns:
            Tupla (éxito, mensaje)
        """
        try:
            path = Path(filepath)
            path.parent.mkdir(parents=True, exist_ok=True)
            
            if format == 'csv':
                with open(path, 'w', newline='', encoding='utf-8') as csvfile:
                    writer = csv.DictWriter(csvfile, 
                                          fieldnames=['id', 'x', 'y', 'z', 'q1', 'q2', 'q3'])
                    writer.writeheader()
                    for point in self.trajectory:
                        writer.writerow(point.to_dict())
            
            elif format == 'json':
                import json
                with open(path, 'w', encoding='utf-8') as jsonfile:
                    json.dump([p.to_dict() for p in self.trajectory], jsonfile, indent=2)
            
            else:
                return False, f"Formato no soportado: {format}"
            
            return True, f"Trayectoria guardada en {filepath}"
        
        except Exception as e:
            return False, f"Error al guardar trayectoria: {str(e)}"
