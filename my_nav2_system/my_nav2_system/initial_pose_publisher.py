import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.publish_initial_pose)

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Configuración de la pose
        msg.pose.pose.position = Point(x=0.25, y=0.11, z=0.0)  # Coordenadas arbitrarias
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Sin rotación

        # Publicar el mensaje
        self.publisher_.publish(msg)
        self.get_logger().info('Publicando posición inicial: "%s"' % msg.pose.pose)
        self.destroy_node()  # Opcional: Detener el nodo después de publicar

def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = InitialPosePublisher()
    rclpy.spin(initial_pose_publisher)
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()