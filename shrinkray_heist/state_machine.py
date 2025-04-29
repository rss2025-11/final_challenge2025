import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool, Float32MultiArray

from shrinkray_heist.controller import Controller

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
        self.points_of_interest = []  # tuples of (point, phase)
        self.points_of_interest_ptr = 0
        self.current_pose = None
        self.current_speed = 0.0
        self.current_phase = Phase.IDLE
        self.detection = None
        self.stop_signal = False

        # constants
        self.goal_threshold = 0.5

        # Create timer for periodic state checking
        self.timer = self.create_timer(0.1, self.state_update)

        self.get_logger().info("State Machine Initialized")

        self.PHASE_MAPPING = {
            Phase.BANANA_DETECTION: self.banana_detection_phase,
            Phase.TRAFFIC_SIGNAL: self.traffic_signal_phase,
            Phase.BANANA_COLLECTION: self.banana_collection_phase,
            Phase.IDLE: lambda: None,
        }

        self.controller = Controller(self)

    def shell_points_callback(self, shell_points: PoseArray):
        """
        Callback for receiving shell points (navigation waypoints).

        Args:
            shell_points (PoseArray): Array of waypoints to visit
        """
        signal_pose = None  # TODO: need to hardcode

        self.points_of_interest = [
            (shell_points.poses[0], Phase.BANANA_DETECTION),  # first banana region
            (0, Phase.BANANA_COLLECTION),  # first banana location, filler
            (signal_pose, Phase.TRAFFIC_SIGNAL),  # signal location
            (
                shell_points.poses[2],
                Phase.BANANA_DETECTION,
            ),  # second banana region
            (0, Phase.BANANA_COLLECTION),  # second banana location, filler
            (signal_pose, Phase.TRAFFIC_SIGNAL),  # signal location
            (shell_points.poses[3], Phase.IDLE),  # end location
        ]

        if self.current_phase == Phase.IDLE and self.current_pose is not None:
            self.transition()

    def detection_callback(self, detection: Float32MultiArray):
        """
        Callback for receiving banana detection.

        Args:
            detection (Float32MultiArray): Location of the banana [x, y] m in robot frame
        """
        # If we're in banana detection phase, process the detection
        if self.current_phase == Phase.BANANA_DETECTION:
            self.detection = detection.data

    def signal_callback(self, signal: Bool):
        """
        Callback for receiving a traffice signal detection.

        Args:
            signal (Bool): True if Red, False if Green
        """
        # If we're in signal detection phase, save the traffic signal
        if self.current_phase == Phase.TRAFFIC_SIGNAL:
            self.stop_signal = signal.data

    def location_callback(self, location: Pose):
        """
        Callback for receiving current vehicle position.

        Args:
            location (Pose): Current position of the vehicle
        """
        self.current_pose = location

        if self.current_phase == Phase.FOLLOWING_PATH:
            self.check_goal_condition()

    def transition(self):
        """Transition to the next phase."""
        self.current_phase = self.points_of_interest[self.points_of_interest_ptr][1]
        self.PHASE_MAPPING[self.current_phase]()

    def check_goal_condition(self):
        """Check if we've reached the current goal."""
        current_goal_point = self.points_of_interest[self.points_of_interest_ptr][0]
        current_goal_phase = self.points_of_interest[self.points_of_interest_ptr][1]

        # Calculate distance to goal (simple Euclidean distance for now)
        dx = self.current_pose.position.x - current_goal_point.position.x
        dy = self.current_pose.position.y - current_goal_point.position.y
        distance = (dx**2 + dy**2) ** 0.5

        if distance < self.goal_threshold:
            self.current_phase = current_goal_phase
            self.PHASE_MAPPING[current_goal_phase]()

    def banana_detection_phase(self):
        """Process banana detection during BANANA_DETECTION phase."""
        self.points_of_interest[self.points_of_interest_ptr + 1] = (
            self.controller.robot_to_map_frame(self.detection),
            Phase.BANANA_COLLECTION,
        )
        self.transition()

    def banana_collection_phase(self):
        """Process banana collection during BANANA_COLLECTION phase."""
        self.controller.collect_banana(self.current_pose)  # will take some time
        self.transition()

    def traffic_signal_phase(self):
        """Process traffic signal detection during TRAFFIC_SIGNAL phase."""
        while self.stop_signal is None or self.stop_signal:
            self.controller.await_signal()  # will just send a full zero control at a higher mux
            # TODO: decide how to handle tight loop, maybe sleep, maybe follow vesc command freq

        self.transition()

    def follow_path_phase(self):
        """Follow a path to a goal point."""
        self.points_of_interest_ptr += 1
        if (
            self.points_of_interest_ptr < len(self.points_of_interest)
            and self.current_pose is not None
        ):
            self.current_phase = Phase.FOLLOWING_PATH
            self.controller.follow_path(
                self.current_pose,
                self.points_of_interest[self.points_of_interest_ptr][0],
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
