import rclpy
from rclpy.node import Node


from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker

from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Odometry

from final_challenge2025.controller import Controller
from final_challenge2025.utils import SIGNAL_MAP_POSE, robot_to_map_frame

from enum import Enum


class Phase(Enum):
    IDLE = 0
    FOLLOWING_PATH = 1
    BANANA_DETECTION = 2
    BANANA_COLLECTION = 3
    BANANA_PARKING = 4


class StateMachine(Node):
    """
    Main state machine for the Shrink Ray Heist challenge.

    This node manages the different phases of the challenge:
    - IDLE: Initialization and path planning
    - FOLLOWING_PATH: Following the planned path to the next point of interest
    - BANANA_DETECTION: Identifying and collecting the correct banana (shrink ray component)
    - BANANA_COLLECTION: Collecting the banana
    """

    def __init__(self):
        super().__init__("state_machine")
        self.declare_parameter("banana_detection_topic", "/detections/banana")
        self.declare_parameter("signal_detection_topic", "/detections/traffic_light")
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
        self.reached_end_subscriber = self.create_subscription(
            Float32MultiArray, "/reached_end", self.reached_end_callback, 1
        )
        self.reached_end_subscriber = self.create_subscription(
            Bool, "/parked", self.parked_callback, 1
        )
        self.pure_pursuit_publisher = self.create_publisher(
            Odometry, '/follow_path_phase_odom', 1
        )
        self.image_processing_publisher = self.create_publisher(
            Bool, "/process_image", 1
        )
        self.marker_pub = self.create_publisher(Marker, "/cone_marker", 1)
        self.exit_pub = self.create_publisher(Bool, "/exit_follow", 1)

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
        self.reached_end = False
        self.parked = False
        self.previous_data = None
        self.prev_detection = None
        self.detector_running = False

        # Timers
        self.main_loop_timer = self.create_timer(
            1.0 / self.get_parameter("main_loop_rate").value, self.main_loop
        )
        self.execute_path_timer = None

        self.controller = Controller(self)

        self.get_logger().info("State Machine Initialized")

    def reached_end_callback(self, reached_end_msg):
        self.reached_end = not (reached_end_msg.data == self.previous_data) # not (distance < 0.1)
        self.previous_data = reached_end_msg.data

    def parked_callback(self, parked_msg):
        self.parked = parked_msg.data

    def shell_points_callback(self, shell_points: PoseArray):
        """
        Callback for receiving shell points (navigation waypoints).

        Args:
            shell_points (PoseArray): Array of waypoints to visit
        """
        self.get_logger().info("Received shell points")
        
        # usage in shell_points_callback
        # self.goal_points = self.order_goals(shell_points)

        # Set up navigation points
        self.goal_points = [
            shell_points.poses[0],  # First banana region
            None,  # Placeholder for first banana location (will be updated)
            shell_points.poses[1],  # Second banana region
            None,  # Placeholder for second banana location
            self.current_pose,  # End location
        ]

        # Set up phases for each point
        self.goal_phases = [
            Phase.BANANA_DETECTION,
            Phase.BANANA_COLLECTION,
            Phase.BANANA_DETECTION,
            Phase.BANANA_COLLECTION,
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
        if self.near_signal():
            if self.detector_running == False:
                msg = Bool()
                msg.data = True
                self.image_processing_publisher.publish(msg)
                self.detector_running = msg.data

            if self.stop_signal == True:
                self.get_logger().info("STOPPING CAR")
                # TODO: might cause jitter
                self.controller.stop_car()
        else:
            if self.current_phase == Phase.FOLLOWING_PATH and self.detector_running == True:
                msg = Bool()
                msg.data = False
                self.image_processing_publisher.publish(msg)
                self.detector_running = msg.data

    def near_signal(self, threshold: float = 20.0):
        """
        Check if the car is near the traffic signal.
        """
        if self.current_pose is None:
            return False

        dx = self.current_pose.position.x - SIGNAL_MAP_POSE.position.x
        dy = self.current_pose.position.y - SIGNAL_MAP_POSE.position.y
        distance = (dx**2 + dy**2) ** 0.5

        return distance < threshold

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
                # Then transition to next phase
                self.current_phase = self.goal_phases[self.current_pointer]
                exit_msg = Bool()
                exit_msg.data = True
                self.exit_pub.publish(exit_msg)
                self.controller.stop_car()
                self.get_logger().info(f"Reached goal, transitioning to {self.current_phase}")
                self.reached_end = False

                msg = Bool()
                msg.data = True
                self.image_processing_publisher.publish(msg)
                self.detector_running = msg.data

        elif self.current_phase == Phase.BANANA_DETECTION:
            # banana detected, begin parking towards it


            if self.detection is not None and not (self.detection[0] == 0.0 and self.detection[1] == 0.0):
                banana_pose = robot_to_map_frame([self.detection[0], self.detection[1], 0.0], self.current_pose) 
                self.goal_points[self.current_pointer + 1] = banana_pose
                # self.detection = None

                # Move to the next point (banana collection)
                self.current_pointer += 1
                self.current_phase = Phase.BANANA_PARKING
                self.get_logger().info(f"Reached goal, transitioning to {self.current_phase}")
                exit_msg = Bool()
                exit_msg.data = True
                self.exit_pub.publish(exit_msg)
                self.controller.stop_car()

            # sweep until we detect the banana
            else:
                self.controller.sweep_banana()


        elif self.current_phase == Phase.BANANA_PARKING:

            if self.parked:
                self.current_phase = Phase.BANANA_COLLECTION
                self.get_logger().info(f"Entering phase {self.current_phase}")
                self.detection = None

            else:
                # ensure detection is not empty
                if self.detection is not None and not (self.detection[0] == 0.0 and self.detection[1] == 0.0):
                    # self.detection gives [x,y] in robot frame
                    self.controller.banana_parking_phase(self.detection[0], self.detection[1])
                    self.prev_detection = self.detection
                elif self.prev_detection is not None:
                    self.controller.banana_parking_phase(self.prev_detection[0], self.prev_detection[1])

            
        elif self.current_phase == Phase.BANANA_COLLECTION:
            # Collect banana and move to next point
            self.controller.collect_banana()
            self.current_pointer += 1
            self.current_phase = Phase.FOLLOWING_PATH
            self.follow_path_phase()

            msg = Bool()
            msg.data = False
            self.image_processing_publisher.publish(msg)
            self.detector_running = msg.data


    def draw_marker(self, x, y):
        """
        Publish a marker to represent the cone in rviz
        """
        marker = Marker()
        marker.header.frame_id = 'map' # self.message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .5
        marker.scale.y = .5
        marker.scale.z = .5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        self.marker_pub.publish(marker)

    def order_goals(self, shell_points: PoseArray):
        """
        Order the goals based on the distance to the current position.
        """
        goals = []
        for point in shell_points.poses:
            # self.get_logger().info(f"POINT: {point.pose}")
            goals.append((point, "banana"))
        goals.append((SIGNAL_MAP_POSE, "signal"))

        # Sort goals by distance to current position
        #Change to use path distances from start
        goals.sort(
            key=lambda x: (x[0].position.x - self.current_pose.position.x) ** 2
            + (x[0].position.y - self.current_pose.position.y) ** 2
        )

        goals.append((self.current_pose, "end"))

        ret = []
        for goal in goals:
            ret.append(goal[0])
            if goal[1] == "banana":
                ret.append(None)

        return ret

    def check_goal_condition(self, threshold: float = 0.6):
        """Check if we've reached the current goal."""
        # return self.reached_end

        # dx = self.current_pose.position.x - self.reached_end[0]
        # dy = self.current_pose.position.y - self.reached_end[1]
        # distance = (dx**2 + dy**2) ** 0.5
        
        # return  distance < threshold

        if self.current_pointer < 0 or self.current_pointer >= len(self.goal_points):
            return False

        current_goal_point = self.goal_points[self.current_pointer]

        # Calculate distance to goal (simple Euclidean distance for now)
        dx = self.current_pose.position.x - current_goal_point.position.x
        dy = self.current_pose.position.y - current_goal_point.position.y
        distance = (dx**2 + dy**2) ** 0.5

        return distance < 0.5 #1.0 # threshold

    def follow_path_phase(self):
        """Follow a path to the current goal point."""
        if self.current_pointer >= len(self.goal_points) or self.goal_points[self.current_pointer] is None:
            self.get_logger().warn("Invalid goal point, skipping path following")
            return
            
        self.get_logger().info(f"CURRENT POINTER: {self.current_pointer}")
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
