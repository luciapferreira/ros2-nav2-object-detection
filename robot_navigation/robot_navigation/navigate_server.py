#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robot_navigation_interfaces.srv import SendGoal
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

from enum import Enum

class OccupancyGrid2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        
        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx


class NavigateToGoal(Node):
    def __init__(self):
        super().__init__('navigate_to_goal')

        self.initial_pose_received = False
        self.goal_pose = None
        self.currentPose = None
        self.map = None

        pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        self.model_pose_sub = self.create_subscription(Odometry, '/odom', self.poseCallback, 10)
        self.service = self.create_service(SendGoal, '/start_navigation', self.start_navigation_callback)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.mapSub = self.create_subscription(OccupancyGrid(), '/map', self.occupancyGridCallback, pose_qos)
        
        self.get_logger().info('NavigationServiceNode is ready.')

    
    def poseCallback(self, msg):
        if not self.initial_pose_received:
            self.get_logger().info('Received initial pose from odom')
            self.init_pose = PoseWithCovarianceStamped()
            self.init_pose.pose.pose = msg.pose.pose
            self.init_pose.header.frame_id = 'map'
            self.initial_pose_pub.publish(self.init_pose)
            self.currentPose = self.init_pose.pose.pose
            self.initial_pose_received = True
        else:
            self.currentPose = msg.pose.pose
    
    def occupancyGridCallback(self, msg):
        self.map = OccupancyGrid2d(msg)

    def start_navigation_callback(self, request, response):

        self.goal_pose=PoseStamped()
        self.goal_pose.header.frame_id='map'
        self.goal_pose.header.stamp=self.get_clock().now().to_msg()

        self.goal_pose.pose.position.x = request.x
        self.goal_pose.pose.position.y = request.y
        self.goal_pose.pose.orientation.z = request.orientation_z
        self.goal_pose.pose.orientation.w = request.orientation_w

        # Attempt to send the goal
        try:
            self.send_goal()
            response.success = True
            response.message = self.response_string
        except Exception as e:
            response.success = False
            response.message = f"Failed to send goal: {e}"

        # Return the response
        return response

    def send_goal(self):
        # Ensure the action server is ready
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available.')
            self.response_string = 'Goal rejected - Action Server not available.'
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose
        
        # Send goal to the action server
        self.get_logger().info('Sending goal request...')
        self.action_client.send_goal_async(goal_msg)
        self.response_string = 'Goal accepted by the action server.'
        
def main(args=None):
    rclpy.init()
    navigate_to_goal = NavigateToGoal()

    navigate_to_goal.get_logger().info('Waiting for initial pose from odometry...')
    while not navigate_to_goal.initial_pose_received:
        rclpy.spin_once(navigate_to_goal, timeout_sec=1.0)
    
    while navigate_to_goal.map is None:
        navigate_to_goal.get_logger().info('Getting initial map')
        rclpy.spin_once(navigate_to_goal, timeout_sec=1.0)

    rclpy.spin(navigate_to_goal)

if __name__ == '__main__':
    main()