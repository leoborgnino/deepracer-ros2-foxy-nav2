import time
import math
import rclpy
from rclpy.node import Node
from waypoint_follower_pkg.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')

        # Initialize Simple Commander Navigator
        self.navigator = BasicNavigator()

        # Path to follow
        self.waypoints_to_follow = [[0.05, 1.97],
                                    [-3.35, 2.04],
                                    [-3.20, 5.90]]

        # Setting up initial pose
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.initial_pose)

        self.get_logger().info('Starting Waypoint Follower Node')


    def create_pose_stamped(self, x, y, theta, frame_id='map', timestamp=None):
        """
        Crea un mensaje PoseStamped válido con las coordenadas x, y y la orientación theta.

    Args:
        x (float): Coordenada X.
        y (float): Coordenada Y.
        theta (float): Orientación en radianes.
        frame_id (str): ID del frame de referencia (default='map').
        timestamp: Tiempo actual (opcional, si no se pasa, se usará None).

        Returns:
        PoseStamped: Mensaje PoseStamped válido.
        """
        pose_stamped = PoseStamped()
        
        # Crear el Header
        header = Header()
        header.frame_id = frame_id
        header.stamp = timestamp if timestamp else rclpy.time.Time().to_msg()
        pose_stamped.header = header
        
        # Asignar la posición
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0
        
        # Asignar la orientación a partir del ángulo theta (fase)
        pose_stamped.pose.orientation.z = theta
        pose_stamped.pose.orientation.w = 1.0
        
        return pose_stamped

    
    def follow_waypoints(self):
        """Executes navigation using a list of waypoints"""

        # Waits until Nav2Stack is fully initialized
        self.navigator.waitUntilNav2Active()

        # Sends waypoint to nav2 stack
        self.get_logger().info("Navigating through waypoints...")
        #self.navigator.followWaypoints(self.waypoints_to_follow)

        # Lista de poses objetivo
        waypoints = [self.create_pose_stamped(2.0,0.0,0.0),self.create_pose_stamped(2.0,-3.0,0.0),self.create_pose_stamped(4.0,-3.0,0.0)]

        # Iterar sobre las poses y enviarlas secuencialmente
        for i, pose in enumerate(waypoints):
            self.get_logger().info(f'Enviando waypoint {i+1} de {len(waypoints)}')
            success = self.navigator.goToPose(pose)
            if not success:
                self.get_logger().error('Deteniendo la secuencia debido a un fallo.')
                break
            time.sleep(2.0)  # Esperar antes de enviar el siguiente waypoint

        self.get_logger().info('Secuencia de waypoints completada.')

        
def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollowerNode()

    try:
        waypoint_follower.follow_waypoints()
    except KeyboardInterrupt:
        waypoint_follower.get_logger().info("Keyboard interrupt detected, shutting down.")
    finally:
        waypoint_follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




