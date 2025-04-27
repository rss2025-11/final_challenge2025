import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32, Bool

from custom_msgs.msg import (
    DetectionArray,
    SignalArray,
)  # TODO: defines these new msg types

from enum import Enum
import time


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
            SignalArray, "/signals", self.signal_callback, 1
        )
        self.location_subscriber = self.create_subscription(
            Pose, "/pf/pose", self.location_callback, 1
        )

        # publishers
        self.speed_publisher = self.create_publisher(Float32, "/speed", 1)
        self.path_request_publisher = self.create_publisher(
            PoseArray, "/path_request", 1
        )

        # state vars
        self.shell_points = []
        self.shell_points_ptr = 0
        self.current_pose = None
        self.current_speed = 0.0
        self.current_phase = Phase.IDLE

        # timer for object detection phase
        self.detection_timer = None
        self.detection_start_time = None

        # return journey tracking
        self.is_return_journey = False
        self.all_points_visited = False

        # signal detection vars
        self.signal_color = None
        self.waiting_for_green = False

        # constants
        self.goal_threshold = 0.5
        self.detection_wait_time = 5.0  # seconds to wait in front of correct object
        self.signal_stop_distance = 0.75  # meters to stop in front of signal
        self.human_avoidance_distance = 1.5  # meters to start avoidance

        # Create timer for periodic state checking
        self.timer = self.create_timer(0.1, self.state_update)

        self.get_logger().info("State Machine Initialized")

    def shell_points_callback(self, shell_points: PoseArray):
        """
        Callback for receiving shell points (navigation waypoints).

        Args:
            shell_points (PoseArray): Array of waypoints to visit
        """
        self.shell_points = shell_points.poses
        # TODO: make sure this list contains all points of interest that we need to visit during the challenge

        if self.current_phase == Phase.IDLE and self.current_pose is not None:
            self.request_path_to_next_point()
            self.transition_to_phase(Phase.FOLLOWING_PATH)

    def detection_callback(self, detections: DetectionArray):
        """
        Callback for receiving object detections.

        Args:
            detections (DetectionArray): Array of detected objects
        """
        # If we're in object detection phase, process the detections
        if self.current_phase == Phase.OBJECT_DETECTION:
            self.process_object_detections()


    def signal_callback(self, signals: SignalArray):
        """
        Callback for receiving signal detections.

        Args:
            signals (SignalArray): Array of detected signals
        """
        # If we're in signal detection phase, process the signal
        if self.current_phase == Phase.SIGNAL_DETECTION:
            self.process_signal()

    def location_callback(self, location: Pose):
        """
        Callback for receiving current vehicle position.

        Args:
            location (Pose): Current position of the vehicle
        """
        self.current_pose = location

        if self.current_phase == Phase.FOLLOWING_PATH:
            self.handle_path_following()

    def state_update(self):
        """Periodic function to check and update the state machine."""
        if self.current_phase is None and self.current_pose is not None:
            # Initialize to IDLE if we have a position but no phase
            self.transition_to_phase(Phase.IDLE)

        # Check if we've completed all points and need to return
        if (
            not self.is_return_journey
            and self.shell_points_ptr >= len(self.shell_points)
            and not self.all_points_visited
        ):

            self.all_points_visited = True
            self.start_return_journey()

    def transition_to_phase(self, new_phase):
        """
        Handle state transition logic.

        Args:
            new_phase (Phase): Phase to transition to
        """
        old_phase = self.current_phase
        self.current_phase = new_phase

        self.get_logger().info(f"Transitioning from {old_phase} to {new_phase}")

        # Phase entry actions
        if new_phase == Phase.IDLE:
            # Reset speed when entering IDLE
            self.publish_speed(0.0)

        elif new_phase == Phase.FOLLOWING_PATH:
            # Set normal speed for path following
            self.publish_speed(1.0)

        elif new_phase == Phase.OBJECT_DETECTION:
            # Stop and start detection timer
            self.publish_speed(0.0)
            self.detection_start_time = time.time()

        elif new_phase == Phase.SIGNAL_DETECTION:
            # Slow down for signal detection
            self.publish_speed(0.5)

        elif new_phase == Phase.HUMAN_OBSTACLE:
            # Slow down for obstacle avoidance
            self.publish_speed(0.5)

    def publish_speed(self, speed):
        """
        Publish vehicle speed command.

        Args:
            speed (float): Speed in m/s
        """
        msg = Float32()
        msg.data = speed
        self.speed_publisher.publish(msg)
        self.current_speed = speed

    def handle_path_following(self):
        """Handle logic during path following phase."""
        # Check if we've reached the current goal
        if self.shell_points_ptr < len(self.shell_points):
            current_goal = self.shell_points[self.shell_points_ptr]

            # Calculate distance to goal (simple Euclidean distance for now)
            dx = self.current_pose.position.x - current_goal.position.x
            dy = self.current_pose.position.y - current_goal.position.y
            distance = (dx**2 + dy**2) ** 0.5

            if distance < self.goal_threshold:
                self.shell_points_ptr += 1

                # If there are more points, request path to next point
                if self.shell_points_ptr < len(self.shell_points):
                    self.get_logger().info(
                        f"Reached waypoint {self.shell_points_ptr-1}, moving to next point"
                    )
                    self.request_path_to_next_point()

                    # Determine what's at this waypoint (signal, object, human)
                    # This would be based on predefined knowledge or detection
                    # For now, we'll just transition to IDLE as a placeholder
                    self.transition_to_phase(Phase.IDLE)

                    # In actual implementation, you would analyze what's at the waypoint:
                    # if detect_signal_at_current_location():
                    #     self.transition_to_phase(Phase.SIGNAL_DETECTION)
                    # elif detect_object_at_current_location():
                    #     self.transition_to_phase(Phase.OBJECT_DETECTION)
                    # elif detect_human_at_current_location():
                    #     self.transition_to_phase(Phase.HUMAN_OBSTACLE)
                else:
                    # Reached final waypoint
                    self.get_logger().info("Reached final waypoint")
                    self.transition_to_phase(Phase.IDLE)

    def process_object_detections(self):
        """Process object detections during OBJECT_DETECTION phase."""
        # Find the correct banana (shrink ray component)
        correct_banana = None
        for detection_id, detection in self.detections.items():
            if detection.is_correct_part:  # Assuming this field exists
                correct_banana = detection
                break

        if correct_banana is not None:
            # Position car in front of correct banana
            # This would involve calculating position and adjusting path

            # Check if we've been in front of the banana long enough
            current_time = time.time()
            if self.detection_start_time is not None:
                elapsed_time = current_time - self.detection_start_time

                if elapsed_time >= self.detection_wait_time:
                    self.get_logger().info(
                        f"Object detection complete after {elapsed_time:.1f} seconds"
                    )
                    self.detection_start_time = None

                    # Return to IDLE to continue journey
                    self.transition_to_phase(Phase.IDLE)

    def process_signal(self):
        """Process signal detection during SIGNAL_DETECTION phase."""
        if self.current_signal is None:
            return

        # Check signal color (assuming the signal has a color field)
        if hasattr(self.current_signal, "color"):
            self.signal_color = self.current_signal.color

            if self.signal_color == "red":
                # Stop for red light
                self.publish_speed(0.0)
                self.waiting_for_green = True
                self.get_logger().info("Stopping for red light")

            elif self.signal_color == "green" and self.waiting_for_green:
                # Continue on green light
                self.waiting_for_green = False
                self.get_logger().info("Green light detected, continuing")
                self.transition_to_phase(Phase.IDLE)

    def handle_human_obstacle(self):
        """Handle human obstacle avoidance logic."""
        # This would involve detecting humans and planning a path around them
        # For example, using dynamic obstacle avoidance like potential fields

        # Once the human is avoided, return to path following
        # self.transition_to_phase(Phase.FOLLOWING_PATH)
        pass

    def request_path_to_next_point(self):
        """Request a path to the next waypoint."""
        if (
            self.shell_points_ptr < len(self.shell_points)
            and self.current_pose is not None
        ):
            path_request = PoseArray()
            path_request.poses = [
                self.current_pose,
                self.shell_points[self.shell_points_ptr],
            ]
            self.path_request_publisher.publish(path_request)
            self.get_logger().info(f"Requesting path to point {self.shell_points_ptr}")

    def start_return_journey(self):
        """Begin the return journey to the start location."""
        self.is_return_journey = True
        self.get_logger().info("Starting return journey to start location")

        # Request path back to start location
        # This assumes the start location is available or can be determined
        start_location = Pose()  # This would be the actual start location

        path_request = PoseArray()
        path_request.poses = [self.current_pose, start_location]
        self.path_request_publisher.publish(path_request)

        self.transition_to_phase(Phase.FOLLOWING_PATH)


def main(args=None):
    """Main entry point for the state machine node."""
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
