import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool, Float32MultiArray

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
        self.declare_parameter("location_topic", "/pf/pose")
        self.declare_parameter("main_loop_rate", 60.0)  # Hz

        # subscribers
        self.shell_points_subscriber = self.create_subscription(
            PoseArray,
            self.get_parameter("shell_points_topic").value,
            self.shell_points_callback,
            1,
        )
        self.detection_subscriber = self.create_subscription(
            Float32MultiArray,
            self.get_parameter("banana_detection_topic").value,
            self.detection_callback,
            1,
        )
        self.signal_subscriber = self.create_subscription(
            Bool,
            self.get_parameter("signal_detection_topic").value,
            self.signal_callback,
            1,
        )
        self.location_subscriber = self.create_subscription(
            Pose, self.get_parameter("location_topic").value, self.location_callback, 1
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

        # Create main loop timer
        self.main_loop_timer = self.create_timer(
            1.0 / self.get_parameter("main_loop_rate").value, self.main_loop
        )

        self.get_logger().info("State Machine Initialized")

        self.controller = Controller(self)

    def shell_points_callback(self, shell_points: PoseArray):
        """
        Callback for receiving shell points (navigation waypoints).

        Args:
            shell_points (PoseArray): Array of waypoints to visit
        """
        signal_pose = None  # TODO: need to hardcode

        # Set up navigation points
        self.goal_points = [
            shell_points.poses[0],  # First banana region
            None,  # Placeholder for first banana location (will be updated)
            signal_pose,  # Signal location
            shell_points.poses[2],  # Second banana region
            None,  # Placeholder for second banana location
            signal_pose,  # Signal location
            shell_points.poses[3],  # End location
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

    def location_callback(self, location: Pose):
        """
        Callback for receiving current vehicle position.

        Args:
            location (Pose): Current position of the vehicle
        """
        self.current_pose = location

    def main_loop(self):
        """Main control loop that runs at a fixed frequency."""
        if self.current_pose is None:
            return

        if self.current_phase == Phase.IDLE:
            if len(self.goal_points) > 0 and self.current_pointer < 0:
                self.current_pointer = 0
                self.current_phase = Phase.FOLLOWING_PATH
                self.follow_path_phase()

        elif self.current_phase == Phase.FOLLOWING_PATH:
            if self.check_goal_condition():
                # Execute the phase for the point we just reached
                self.current_phase = self.goal_phases[self.current_pointer]
                # TODO: use controller to make robot stop

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
                # Move to the next point (banana collection)
                self.current_pointer += 1
                self.current_phase = Phase.FOLLOWING_PATH
                self.detection = None
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
        """Follow a path to the current goal point from current pose."""
        if (
            self.current_pointer < len(self.goal_points)
            and self.current_pose is not None
        ):
            goal_point = self.goal_points[self.current_pointer]
            # TODO: this might block the main phases loop/timer
            self.controller.follow_path(
                self.current_pose,
                goal_point,
            )
        else:
            self.get_logger().info("Reached end of path or current pose is None")


def main(args=None):
    """Main entry point for the state machine node."""
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
