"""
Punto de entrada para ejecutar MinimalPublisher con UserInterface interactiva.

Este archivo inicia el nodo ROS con una interfaz interactiva que permite
al usuario enviar comandos al microcontrolador.
"""

import rclpy
from robot_control.publisher import MinimalPublisher


def main(args=None):
    """
    Función principal que inicializa y ejecuta el nodo con interfaz interactiva.
    
    Args:
        args: Argumentos de línea de comandos para ROS 2.
    """
    rclpy.init(args=args)
    
    # Crear nodo con interfaz de usuario habilitada
    minimal_publisher = MinimalPublisher(enable_ui=True)
    
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener la interfaz de usuario si está en ejecución
        if minimal_publisher.user_interface:
            minimal_publisher.user_interface.stop()
        
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
