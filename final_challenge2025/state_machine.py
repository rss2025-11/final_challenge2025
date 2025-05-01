import rclpy
from rclpy.node import Node


from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Odometry

from final_challenge2025.controller import Controller

from enum import Enum


class Phase(Enum):
    IDLE = 0
    FOLLOWING_PATH = 1
    TRAFFIC_SIGNAL = 2
    BANANA_DETECTION = 3
    BANANA_COLLECTION = 4


class StateMachine(Node):
    """
    Main state machine for the Shrink Ray Heist challenge.

    This node manages the different phases of the challenge:
    - IDLE: Initialization and path planning
    - FOLLOWING_PATH: Following the planned path to the next point of interest
    - TRAFFIC_SIGNAL: Detecting and responding to traffic signals
    - BANANA_DETECTION: Identifying and collecting the correct banana (shrink ray component)
    - BANANA_COLLECTION: Collecting the banana
    """

    def __init__(self):
        super().__init__("state_machine")
        self.declare_parameter("banana_detection_topic", "/banana_detection")
        self.declare_parameter("signal_detection_topic", "/signal_detection")
        self.declare_parameter("shell_points_topic", "/shell_points")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("main_loop_rate", 60.0)  # Hz

        self.banana_detection_topic = self.get_parameter("banana_detection_topic").get_parameter_value().string_value
        self.signal_detection_topic = self.get_parameter("signal_detection_topic").get_parameter_value().string_value
        self.shell_points_topic = self.get_parameter("shell_points_topic").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.main_loop_rate = self.get_parameter("main_loop_rate").get_parameter_value().double_value

        # subscribers
        self.shell_points_subscriber = self.create_subscription(
            PoseArray,
            self.shell_points_topic,
            self.shell_points_callback,
            1,
        )
        self.detection_subscriber = self.create_subscription(
            Float32MultiArray,
            self.banana_detection_topic,
            self.detection_callback,
            1,
        )
        self.signal_subscriber = self.create_subscription(
            Bool,
            self.signal_detection_topic,
            self.signal_callback,
            1,
        )
        self.location_subscriber = self.create_subscription(
            Odometry, self.odom_topic, self.location_callback, 1
        )

        # state vars
        self.goal_points = []  # Navigation waypoints
        self.goal_phases = []  # Phase to execute at each point
        self.current_pointer = -1
        self.current_pose = None
        self.current_speed = 0.0
        self.current_phase = Phase.IDLE
        self.detection = None
        self.stop_signal = None
        self.current_path_plan = None

        # Timers
        self.main_loop_timer = self.create_timer(
            1.0 / self.get_parameter("main_loop_rate").value, self.main_loop
        )
        self.execute_path_timer = None

        self.controller = Controller(self)

        self.get_logger().info("State Machine Initialized")

    def shell_points_callback(self, shell_points: PoseArray):
        """
        Callback for receiving shell points (navigation waypoints).

        Args:
            shell_points (PoseArray): Array of waypoints to visit
        """
        self.get_logger().info("Received shell points")
        signal_pose = Pose()  # TODO: need to change for real world
        signal_pose.position.x = -10.613014221191406
        signal_pose.position.y = 16.343740463256836
        signal_pose.position.z = 0.0
        signal_pose.orientation.x = 0.0
        signal_pose.orientation.y = 0.0
        signal_pose.orientation.z = 0.3809232176506139
        signal_pose.orientation.w = 0.9246066743511552
        
        # Set up navigation points
        self.goal_points = [
            shell_points.poses[0],  # First banana region
            None,  # Placeholder for first banana location (will be updated)
            signal_pose,  # Signal location
            shell_points.poses[1],  # Second banana region
            None,  # Placeholder for second banana location
            signal_pose,  # Signal location
            None,  # End location
        ]

        # Set up phases for each point
        self.goal_phases = [
            Phase.BANANA_DETECTION,
            Phase.BANANA_COLLECTION,
            Phase.TRAFFIC_SIGNAL,
            Phase.BANANA_DETECTION,
            Phase.BANANA_COLLECTION,
            Phase.TRAFFIC_SIGNAL,
            Phase.IDLE,  # End
        ]
        
        # Add debug info
        self.get_logger().info(f"Goal points length: {len(self.goal_points)}")
        self.get_logger().info(f"Current pointer: {self.current_pointer}")
        self.get_logger().info(f"Current phase: {self.current_phase}")

    def detection_callback(self, detection: Float32MultiArray):
        """
        Callback for receiving banana detection.

        Args:
            detection (Float32MultiArray): Location of the banana [x, y] m in robot frame
        """
        self.detection = detection.data

    def signal_callback(self, signal: Bool):
        """
        Callback for receiving a traffice signal detection.

        Args:
            signal (Bool): True if Red, False if Green
        """
        self.stop_signal = signal.data

    def location_callback(self, location: Odometry):
        """
        Callback for receiving current vehicle position.

        Args:
            location (Odometry): Current odometry of the vehicle
        """
        self.current_pose = location.pose.pose  # Extract the Pose from PoseWithCovariance

    def main_loop(self):
        """Main control loop that runs at a fixed frequency."""
        if self.current_pose is None:
            return

        if self.current_phase == Phase.IDLE:
            if len(self.goal_points) > 0 and self.current_pointer < 0:
                self.get_logger().info("Leaving idle phase, starting challenge")
                self.current_pointer = 0
                self.current_phase = Phase.FOLLOWING_PATH
                self.follow_path_phase()

        elif self.current_phase == Phase.FOLLOWING_PATH:
            if self.check_goal_condition():
                # Cancel the path execution timer
                if self.execute_path_timer:
                    self.execute_path_timer.cancel()
                    self.execute_path_timer = None
                
                # Stop the car
                self.controller.stop_car()
                
                # Then transition to next phase
                self.current_phase = self.goal_phases[self.current_pointer]
                self.get_logger().info(f"Reached goal, transitioning to {self.current_phase}")

        elif self.current_phase == Phase.TRAFFIC_SIGNAL:
            if self.stop_signal is not None and not self.stop_signal:
                # Green light, continue
                self.current_pointer += 1
                self.current_phase = Phase.FOLLOWING_PATH
                self.follow_path_phase()
            else:
                # Red light, wait
                self.controller.await_signal()

        elif self.current_phase == Phase.BANANA_DETECTION:
            if self.detection is not None:
                # Update the next point (banana collection location)
                banana_pose = self.controller.robot_to_map_frame(self.detection)
                self.goal_points[self.current_pointer + 1] = banana_pose
                self.detection = None
            
                # Move to the next point (banana collection)
                self.current_pointer += 1
                self.current_phase = Phase.FOLLOWING_PATH
                self.follow_path_phase()

        elif self.current_phase == Phase.BANANA_COLLECTION:
            # Collect banana and move to next point
            self.controller.collect_banana(self.current_pose)
            self.current_pointer += 1
            self.current_phase = Phase.FOLLOWING_PATH
            self.follow_path_phase()

    def check_goal_condition(self, threshold: float = 0.5):
        """Check if we've reached the current goal."""
        if self.current_pointer < 0 or self.current_pointer >= len(self.goal_points):
            return False

        current_goal_point = self.goal_points[self.current_pointer]

        # Calculate distance to goal (simple Euclidean distance for now)
        dx = self.current_pose.position.x - current_goal_point.position.x
        dy = self.current_pose.position.y - current_goal_point.position.y
        distance = (dx**2 + dy**2) ** 0.5

        return distance < threshold

    def follow_path_phase(self):
        """Follow a path to the current goal point."""
        if self.current_pointer >= len(self.goal_points) or self.goal_points[self.current_pointer] is None:
            self.get_logger().warn("Invalid goal point, skipping path following")
            return
            
        self.get_logger().info(f"Following path to {self.goal_phases[self.current_pointer]}")
        self.controller.follow_path(self.current_pose, self.goal_points[self.current_pointer])


def main(args=None):
    """Main entry point for the state machine node."""
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
