import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Esperar hasta que el servidor esté disponible
        self.get_logger().info("Esperando que el servidor de acciones esté disponible...")
        self._action_client.wait_for_server()  # Bloquea hasta que el servidor esté disponible
        self.get_logger().info("Servidor de acciones disponible.")
        while not self._action_client.wait_for_server(timeout_sec=4.0):
            self.get_logger().info('Waiting for action server to be up...')
        self.send_goal(0.5, 0.6)  # Coordenadas x, y para el objetivo
        
    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Asumiendo orientación hacia adelante
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        #rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        current_position = feedback_msg.feedback.current_pose.pose.position
        distance_remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f'Received feedback - Current Position: ({current_position.x}, {current_position.y}, {current_position.z}), '
            f'Distance Remaining: {distance_remaining} meters'
        )

def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateToPoseClient()
    #action_client.send_goal(0.5, 0.6)  # Coordenadas x, y para el objetivo
    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()