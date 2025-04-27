import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose

from shrinkray_heist.controller import Controller

from custom_msgs.msg import (
    DetectionArray,
    TrafficSignal,
)  # TODO: defines these new msg types

from enum import Enum


class Phase(Enum):
    IDLE = 0
    FOLLOWING_PATH = 1
    SIGNAL_DETECTION = 2
    OBJECT_DETECTION = 3
    HUMAN_OBSTACLE = 4


class StateMachine(Node):
    """
    Main state machine for the Shrink Ray Heist challenge.

    This node manages the different phases of the challenge:
    - IDLE: Initialization and path planning
    - FOLLOWING_PATH: Following the planned path to the next point of interest
    - SIGNAL_DETECTION: Detecting and responding to traffic signals
    - OBJECT_DETECTION: Identifying and collecting the correct banana (shrink ray component)
    - HUMAN_OBSTACLE: Avoiding human obstacles in the path
    """

    def __init__(self):
        super().__init__("state_machine")

        # subscribers
        self.shell_points_subscriber = self.create_subscription(
            PoseArray, "/shell_points", self.shell_points_callback, 1
        )
        self.detection_subscriber = self.create_subscription(
            DetectionArray, "/detections", self.detection_callback, 1
        )
        self.signal_subscriber = self.create_subscription(
            TrafficSignal, "/signals", self.signal_callback, 1
        )
        self.location_subscriber = self.create_subscription(
            Pose, "/pf/pose", self.location_callback, 1
        )

        # publishers
        self.path_request_publisher = self.create_publisher(
            PoseArray, "/path_request", 1
        )

        # state vars
        self.points_of_interest = []  # tuples of (point, phase)
        self.points_of_interest_ptr = 0
        self.current_pose = None
        self.current_speed = 0.0
        self.current_phase = Phase.IDLE
        self.detections = {}
        self.stop_signal = False

        # constants
        self.goal_threshold = 0.5
        self.detection_wait_time = 5.0  # seconds to wait in front of correct object
        self.signal_stop_distance = 0.75  # meters to stop in front of signal
        self.human_avoidance_distance = 1.5  # meters to start avoidance

        # Create timer for periodic state checking
        self.timer = self.create_timer(0.1, self.state_update)

        self.get_logger().info("State Machine Initialized")

        self.PHASE_MAPPING = {
            Phase.OBJECT_DETECTION: self.object_detection_phase,
            Phase.SIGNAL_DETECTION: self.signal_detection_phase,
            Phase.HUMAN_OBSTACLE: self.human_obstacle_phase,
            Phase.IDLE: lambda: None,
        }

        self.controller = Controller()

    def shell_points_callback(self, shell_points: PoseArray):
        """
        Callback for receiving shell points (navigation waypoints).

        Args:
            shell_points (PoseArray): Array of waypoints to visit
        """
        signal_pose = None  # TODO: need to hardcode
        human_pose = None  # TODO: need to hardcode

        self.points_of_interest = [
            (shell_points.poses[0], Phase.OBJECT_DETECTION),  # first banana location
            (signal_pose, Phase.SIGNAL_DETECTION),  # signal location
            (human_pose, Phase.HUMAN_OBSTACLE),  # human location
            (
                shell_points.poses[2],
                Phase.OBJECT_DETECTION,
            ),  # second banana location, start backtrack
            (human_pose, Phase.HUMAN_OBSTACLE),  # human location
            (signal_pose, Phase.SIGNAL_DETECTION),  # signal location
            (shell_points.poses[3], Phase.IDLE),  # end location
        ]

        if self.current_phase == Phase.IDLE and self.current_pose is not None:
            self.request_path_to_next_point()

    def detection_callback(self, detections: DetectionArray):
        """
        Callback for receiving object detections.

        Args:
            detections (DetectionArray): Array of detected objects
        """
        # If we're in object detection phase, process the detections
        if self.current_phase == Phase.OBJECT_DETECTION:
            self.detections = detections

    def signal_callback(self, signal: TrafficSignal):
        """
        Callback for receiving a traffice signal detection.

        Args:
            signal (TrafficSignal): A detected traffic signal
        """
        # If we're in signal detection phase, save the traffic signal
        if self.current_phase == Phase.SIGNAL_DETECTION:
            self.stop_signal = signal.is_stop_signal

    def location_callback(self, location: Pose):
        """
        Callback for receiving current vehicle position.

        Args:
            location (Pose): Current position of the vehicle
        """
        self.current_pose = location

        if self.current_phase == Phase.FOLLOWING_PATH:
            self.handle_path_following()

    def handle_path_following(self):
        """Handle logic during path following phase."""
        # Check if we've reached the current goal
        current_goal_point = self.points_of_interest[self.points_of_interest_ptr][0]
        current_goal_phase = self.points_of_interest[self.points_of_interest_ptr][1]

        # Calculate distance to goal (simple Euclidean distance for now)
        dx = self.current_pose.position.x - current_goal_point.position.x
        dy = self.current_pose.position.y - current_goal_point.position.y
        distance = (dx**2 + dy**2) ** 0.5

        if distance < self.goal_threshold:
            self.current_phase = current_goal_phase
            self.PHASE_MAPPING[current_goal_phase]()

    def object_detection_phase(self):
        """Process object detections during OBJECT_DETECTION phase."""
        # Find the correct banana (shrink ray component)
        correct_banana = None
        for _, detection in self.detections.items():
            if detection.is_correct_part:  # Assuming this field exists
                correct_banana = detection
                break

        if correct_banana is not None:
            self.controller.collect_banana()  # will take some time
            self.next_goal()

    def signal_detection_phase(self):
        """Process signal detection during SIGNAL_DETECTION phase."""
        while self.stop_signal is None or self.stop_signal:
            self.controller.await_signal()  # will just send a full zero control at a higher mux
            # TODO: decide how to handle tight loop, maybe sleep, maybe follow vesc command freq

        self.next_goal()

    def human_obstacle_phase(self):
        """Handle human obstacle avoidance logic."""
        self.controller.avoid_human()  # will go past the human and return once done
        # TODO: might need a sleep here if we are waiting for lower level controller to
        # complete the human avoidance path follow
        self.next_goal()

    def next_goal(self):
        """Transition to the next phase."""
        self.points_of_interest_ptr += 1
        self.request_path_to_next_point()

    def request_path_to_next_point(self):
        """Request a path from the current location to the current goal point.
        Assumes that the path planner will initiate following a path from start pose to end pose once it is planned.
        """
        if (
            self.points_of_interest_ptr < len(self.points_of_interest)
            and self.current_pose is not None
        ):
            path_request = PoseArray()
            path_request.poses = [
                self.current_pose,
                self.points_of_interest[self.points_of_interest_ptr][0],
            ]
            self.path_request_publisher.publish(path_request)
            self.get_logger().info(
                f"Requesting path to point {self.points_of_interest_ptr} and phase {self.points_of_interest[self.points_of_interest_ptr][1]}"
            )

            # TODO: transitions directly into path following, should we idle until path is planned?
            self.current_phase = Phase.FOLLOWING_PATH


def main(args=None):
    """Main entry point for the state machine node."""
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
