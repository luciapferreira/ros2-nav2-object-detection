#!/usr/bin/env python3
import os, sys, yaml, time
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from robot_navigation_interfaces.srv import SendGoal
from ament_index_python.packages import get_package_share_directory
from contextlib import contextmanager
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

@contextmanager
def raw_mode(file):
    """Context manager to set the terminal to raw mode and restore it safely."""
    old_attrs = termios.tcgetattr(file)
    try:
        tty.setraw(file)
        yield
    finally:
        termios.tcsetattr(file, termios.TCSADRAIN, old_attrs)


class StartNavigationClient(Node):
    def __init__(self):
        super().__init__('navigate_client')

        self.client = self.create_client(SendGoal, '/start_navigation')
        self.rviz_2d_pose = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.waypoint_pose_callback ,10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.pose2d = None

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/start_navigation service not available, waiting...')

        self.get_logger().info('Client for /start_navigation service is ready.')

        share_dir = get_package_share_directory('robot_navigation')
        self.yaml_file = os.path.join(share_dir, 'config', 'navigate_waypoint.yaml')

        self.menu()
    
    def waypoint_pose_callback(self,msg):
        self.pose2d=msg.pose.pose
        x_waypoint=self.pose2d.position.x
        y_waypoint=self.pose2d.position.y
        z_waypoint=self.pose2d.orientation.z
        w_waypoint=self.pose2d.orientation.w
        self.get_logger().info(f'Extracted goal point: x={x_waypoint}, y={y_waypoint}, z={z_waypoint}, w={w_waypoint}')

    def goal_pose_callback(self, msg):
        """Triggered when '2D Goal Pose' is clicked in RViz."""
        self.pose2d = msg.pose
        x = self.pose2d.position.x
        y = self.pose2d.position.y
        z = self.pose2d.orientation.z
        w = self.pose2d.orientation.w
        self.get_logger().info(f'Extracted goal pose from RViz: x={x}, y={y}, z={z}, w={w}'
    )


    def menu(self):
        self.get_logger().info("Menu:")
        
        with open(self.yaml_file, 'r') as file:
            self.goals = yaml.safe_load(file)
            self.waypoint_count = sum(1 for key in self.goals if key.startswith("waypoint_"))
            self.get_logger().info("Press 1-%d to send predefined goals:" %(self.waypoint_count))
            for i in range(1, self.waypoint_count + 1):
                waypoint = self.goals[f"waypoint_{i}"]
                description=waypoint['description']
                self.get_logger().info(f"{i} - %s" %(description))
            file.close()

        self.get_logger().info("Press 'e' to enter a custom goal.")
        self.get_logger().info("Press 'q' to quit.")
    
    def get_key(self):
        """Reads a sequence of characters to handle multi-digit inputs."""
        with raw_mode(sys.stdin.fileno()):
            rlist, _, _ = select.select([sys.stdin], [], [], 0.5)
            if rlist:
                key = sys.stdin.read(1)
                if key.isdigit():
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                    while rlist:
                        next_key = sys.stdin.read(1)
                        if not next_key.isdigit():
                            break
                        key += next_key
                        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                return key
            return ''


    def handle_key_press(self, key):
        """Handles key press events."""
        waypoint_key = f"waypoint_{key}"
        if waypoint_key in self.goals:
            if key == 'e' or key == 'q':
                self.get_logger().error(f"Dont client binded key: {key}")
                self.get_logger().error(f"Fix yaml and rerun the client\nForce Exiting...")
                os._exit(os.EX_CONFIG)
            waypoint = self.goals[waypoint_key]
            x, y = waypoint['x'], waypoint['y']
            orientation_z, orientation_w = waypoint['z'], waypoint['w']
            self.send_request(x, y, orientation_z, orientation_w)
        elif key == 'e':
            # Make sure pose2d is None (prevent recovery the same pose twices)
            self.pose2d=None
            comment=input("What is the comment for the goal (enter for auto):")
            if comment == "": comment=f'Custom Goal {self.waypoint_count+1}'

            self.get_logger().warn(f"Use \"2D Goal Pose\" in RViz")
            while True:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.pose2d is not None:
                    break

            yaml_waypoint={
                f'waypoint_{self.waypoint_count+1}': {
                'x' : float(self.pose2d.position.x), 
                'y' : float(self.pose2d.position.y),
                'z' : float(self.pose2d.orientation.z),
                'w' : float(self.pose2d.orientation.w),
                'description' : f'{comment}'
                }                    
            }
            with open(self.yaml_file,"a") as file:
                file.write("\n\n")
                yaml.dump(yaml_waypoint,file)
                self.get_logger().info(f"Waypoint write to:{ self.yaml_file }")
                file.close()
            self.menu()
        elif key == 'q':
            self.get_logger().info("Exiting...")
            os._exit(os.EX_OK)

    def send_request(self, x, y, orientation_z, orientation_w):
        request = SendGoal.Request()
        request.x = x
        request.y = y
        request.orientation_z = orientation_z
        request.orientation_w = orientation_w

        self.get_logger().info(f'Sending goal: x={x}, y={y}, z={orientation_z}, w={orientation_w}')
        self.get_logger().info('Wait for response...')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Navigation started: {response.message}')
                else:
                    self.get_logger().error(f'Navigation failed to start: {response.message}')
            except Exception as e:
                self.get_logger().error(f'Exception while calling service: {e}')
            
            self.menu()

def main(args=None):
    rclpy.init(args=args)
    client_node = StartNavigationClient()
    
    try:
        while rclpy.ok():
            key = client_node.get_key()
            if key:
                client_node.handle_key_press(key)
    except KeyboardInterrupt:
        client_node.get_logger().info("Shutting down due to keyboard interrupt.")
    finally:
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
