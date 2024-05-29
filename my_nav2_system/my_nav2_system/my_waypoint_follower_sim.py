import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from custom_interface.srv import FollowWaypointsSrv

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.srv = self.create_service(FollowWaypointsSrv, 'start_waypoint_following', self.start_following_callback)

    def start_following_callback(self, request, response):
        #waypoints = self.create_waypoints()
        waypoints = request.waypoints
        self.send_waypoints(waypoints)
        response.success = True
        response.message = "Waypoints received and goal sent"
        return response

    def create_waypoints(self):
        return [
            PoseStamped(
                header=Header(frame_id="map", stamp=self.get_clock().now().to_msg()),
                pose=Pose(
                    position=Point(x=-1.0, y=0.5, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )
            ),
            PoseStamped(
                header=Header(frame_id="map", stamp=self.get_clock().now().to_msg()),
                pose=Pose(
                    position=Point(x=-2.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )
            ),
            PoseStamped(
                header=Header(frame_id="map", stamp=self.get_clock().now().to_msg()),
                pose=Pose(
                    position=Point(x=-3.3, y=3.8, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )
            )
        ]
    
    def send_waypoints(self, waypoints):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result received')
        for waypoint_result in result.missed_waypoints:
            self.get_logger().info(f'Missed waypoint {waypoint_result}')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    # Define tus waypoints aquí
    #waypoints = [
    #    PoseStamped(),  # Define cada PoseStamped con la posición adecuada
    #]
    #node.send_waypoints(waypoints)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()