import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from rclpy.clock import ROSClock
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
import os
import re
import time
from .config import ROBOT_NAMES
from functools import partial
import asyncio
import json
from tf_transformations import euler_from_quaternion
import math

class Nav2Functions:
    def __init__(self, node, use_toy_robots):
        self.node = node
        self.loop = asyncio.get_event_loop()
        self.use_toy_robots = use_toy_robots
        self.robot_task_mapping = {}
        self.robot_goal_status = {}

        self.robot_goals = {}  # {robot_id: goal_position}
        self.robot_current_positions = {}  # {robot_id: current_position}
        self.robot_current_orientations = {}  # {robot_id: current_orientation} - Add this line
        self.task_status = {}  # {task_name: {robot_id: {reached: bool, goal: Point}}}
        if not self.use_toy_robots:
            self.goal_threshold = 5  # [m]
            self.angle_threshold = 2 # [rad]
        else:
            self.goal_threshold = 0.25  # [m]
            self.angle_threshold = 0.1 # [rad]
        self.reach_stable_start_time = {}  # {robot_id: time when angle stability started}
        self.last_log_time = 0  # [s]


        # Subscribe to the pose topics for each robot
        for robot_id in ROBOT_NAMES.values():
            topic_name = self.parse_robot_id(robot_id['id'], format_type='topic') + 'base_link/pose'
            self.node.get_logger().info(f"Subscribing to {topic_name}...")
            self.node.create_subscription(
                PoseStamped,
                topic_name,
                partial(self.pose_callback, robot_id=robot_id['id']),
                10
            )

    def load_object_data(self, target_object):
        if not self.use_toy_robots:
            json_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src', 'breakdown_function_handler', 'object_database', 'object_database.json'))
        else:
            json_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src', 'breakdown_function_handler', 'object_database', 'object_database_4_toy_robots.json'))
        
        with open(json_path, 'r') as json_file:
            data = json.load(json_file)
        
        # Find the data for the specific target object
        for obj in data:
            if obj['object_name'] == target_object:
                return obj
        
        # Return an empty dictionary if the target object is not found
        return {}


    def pose_callback(self, msg: PoseStamped, robot_id: str):
        # Update the current position of the robot
        self.robot_current_positions[robot_id] = msg.pose.position
        # Store orientation as a quaternion
        self.robot_current_orientations[robot_id] = msg.pose.orientation
        asyncio.run_coroutine_threadsafe(self.check_goal_reached(robot_id), self.loop)
        # if robot_id == 'robot_dump_truck_02':
        #     self.node.get_logger().info('\033[95m' + f"Robot {robot_id} current position: {msg.pose.position}" + '\033[0m')


    async def check_goal_reached(self, robot_id):
        current_time = time.time()
        if robot_id in self.robot_goals and robot_id in self.robot_current_positions:
            goal_position = self.robot_goals[robot_id]
            current_position = self.robot_current_positions[robot_id]

            # Calculate distance between current position and goal position
            distance = np.sqrt(
                (goal_position.x - current_position.x) ** 2 +
                (goal_position.y - current_position.y) ** 2
            )

            # Extract goal orientation (assuming goal has orientation set)
            goal_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            current_orientation = self.robot_current_orientations[robot_id]

            # Convert quaternions to Euler angles for easier comparison
            goal_yaw = euler_from_quaternion([goal_orientation.x, goal_orientation.y, goal_orientation.z, goal_orientation.w])[2]
            current_yaw = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])[2]

            # Calculate the absolute angle difference
            angle_diff = abs(math.atan2(math.sin(goal_yaw - current_yaw), math.cos(goal_yaw - current_yaw)))

            # Check if both position and orientation are within acceptable thresholds
            if distance < self.goal_threshold and angle_diff < self.angle_threshold:
                if robot_id not in self.reach_stable_start_time:
                    # Start tracking stability time
                    self.reach_stable_start_time[robot_id] = current_time

                # Check if the angle has been stable for at least 3 seconds
                if current_time - self.reach_stable_start_time[robot_id] >= 3.0:
                    for task_name, robot_status in list(self.task_status.items()):
                        if robot_id in robot_status and not robot_status[robot_id]['reached']:
                            robot_status[robot_id]['reached'] = True
                            self.node.get_logger().info(f"Robot {robot_id} reached its goal for task {task_name}!")
                            if all(status['reached'] for status in robot_status.values()):
                                if not await self.node.blackboard.is_task_completed(task_name):
                                    self.node.get_logger().info(f"Task {task_name} completed!")
                                    await self.node.blackboard.mark_task_completed(task_name)
                                    self.node.get_logger().info(f"Task {task_name} marked as completed")
                                    if task_name in self.task_status:
                                        del self.task_status[task_name]
                                    
                                    # After completing a task, check for any tasks that might now be ready
                                    await self.node.check_and_execute_ready_tasks()
            else:
                # Reset the stability start time if the angle goes out of range
                self.reach_stable_start_time.pop(robot_id, None)
                if current_time - self.last_log_time >= 1:
                    self.node.get_logger().info(f"Robot {robot_id} distance to goal: {distance:.2f} [m], angle difference: {angle_diff:.2f} [rad]")
                    self.last_log_time = current_time

    # ------------------------------------------------------------------------------
    # Breakdown Navigation Functions
    # - avoid areas for all robots    
    # - allow areas for all robots
    # - avoid areas for specific robots
    # - allow areas for specific robots
    #
    # - target area for all robots
    # - return to start for all robots
    # - target area for specific robots
    # - return to start for specific robots
    # ------------------------------------------------------------------------------
    async def avoid_areas_for_all_robots(self, target_object='puddle1', robot_ids=None, task_name=None):
        await self.process_map(True, robot_ids, target_object)

    async def allow_areas_for_all_robots(self, target_object='puddle1', robot_ids=None, task_name=None):
        await self.process_map(False, robot_ids, target_object)

    async def avoid_areas_for_specific_robots(self, target_object='puddle1', robot_ids=None, task_name=None):
        await self.process_map(True, robot_ids, target_object)

    async def allow_areas_for_specific_robots(self, target_object='puddle1', robot_ids=None, task_name=None):
        await self.process_map(False, robot_ids, target_object)

    async def target_area_for_all_robots(self, target_object='puddle1', robot_ids=None, task_name=None):
        await self.target_area_for_specific_robots(target_object, robot_ids, task_name)

    async def return_to_start_for_all_robots(self, target_object='puddle1', robot_ids=None, task_name=None):
        await self.return_to_start_for_specific_robots(target_object, robot_ids, task_name)

    async def target_area_for_specific_robots(self, target_object='puddle1', robot_ids=None, task_name=None):
        if await self.node.blackboard.is_task_completed(task_name):
            self.node.get_logger().info(f'Task {task_name} already completed, skipping execution.')
            return

        self.task_status[task_name] = {}

        object_data = self.load_object_data(target_object)
        target_x, target_y = object_data.get("target_position", {}).get('x', 0), object_data.get("target_position", {}).get('y', 0)
        dump_truck_count = sum(1 for robot_details in ROBOT_NAMES.values() if robot_details['type'] == 'dump_truck')
        offsets = self.calculate_offsets_for_robots(robot_ids, dump_truck_count)

        for robot_id, (offset_x, offset_y) in zip(robot_ids, offsets):
            offset_x, offset_y = 0.0, 0.0
            goal_position = Point(x=target_x + offset_x, y=target_y + offset_y, z=0.0)

            await self.publish_goal_for_robot(robot_id, goal_position, task_name)
            self.task_status[task_name][robot_id] = {'reached': False, 'goal': goal_position}

        self.node.get_logger().info('\033[92m' + f'Started target_area_for_specific_robots: {robot_ids}, {task_name}, {target_object}' + '\033[0m')

    async def return_to_start_for_specific_robots(self, target_object='puddle1', robot_ids=None, task_name=None):
        if await self.node.blackboard.is_task_completed(task_name):
            self.node.get_logger().info(f'Task {task_name} already completed, skipping execution.')
            return
        self.task_status[task_name] = {}

        dump_truck_count = sum(1 for robot_details in ROBOT_NAMES.values() if robot_details['type'] == 'dump_truck')

        for robot_id in robot_ids:
            try:
                robot_type, number = self.parse_robot_id(robot_id, format_type='numeric')
                ros_start_position = self.calculate_start_position(robot_type, number, dump_truck_count)
                await self.publish_goal_for_robot(robot_id, ros_start_position, task_name)
                self.task_status[task_name][robot_id] = {'reached': False, 'goal': ros_start_position}
            except ValueError as e:
                self.node.get_logger().error(f'Error processing robot ID {robot_id}: {e}')

        self.node.get_logger().info('\033[92m' + f'Started return_to_start_for_specific_robots: {robot_ids}, {task_name}, {target_object}' + '\033[0m')

    # ------------------------------------------------------------------------------
    # Utility Functions
    # ------------------------------------------------------------------------------
    async def process_map(self, avoid, robot_ids, target_object='puddle1'):
        object_data = self.load_object_data(target_object)
        red_image_path = os.path.join(self.node.maps_path, f'clip_output_{target_object}.jpg')
        red_image = cv2.imread(red_image_path)
        cv2.imwrite(os.path.join(self.node.maps_path, f'301_map_image_from_{target_object}.jpg'), red_image)
        map_image_from_nav2 = self.node.occupancy_grid_to_image(self.node.map_data_from_nav2)
        cv2.imwrite(os.path.join(self.node.maps_path, '302_map_image_from_nav2.jpg'), map_image_from_nav2)

        lower_red = np.array([0, 0, 120])
        upper_red = np.array([100, 100, 255])
        red_mask = cv2.inRange(red_image, lower_red, upper_red)
        cv2.imwrite(os.path.join(self.node.maps_path, '303_red_mask.jpg'), red_mask)

        # Change red areas to black (0) or white (255) based on the avoid flag
        red_image_gray = np.where(red_mask == 255, 0 if avoid else 255, 255).astype(np.uint8)
        cv2.imwrite(os.path.join(self.node.maps_path, '304_red_image_gray.jpg'), red_image_gray)

        # Resize the red image to match the map's dimensions
        resized_red_image = cv2.resize(red_image_gray, (map_image_from_nav2.shape[1], map_image_from_nav2.shape[0]))
        cv2.imwrite(os.path.join(self.node.maps_path, '305_resized_red_image_gray.jpg'), resized_red_image)

        # Combine the resized red image with the map image
        map_image_combined = np.where(resized_red_image == 0, 0, map_image_from_nav2)
        cv2.imwrite(os.path.join(self.node.maps_path, '306_combined_map_image.jpg'), map_image_combined)

        updated_map_data = self.image_to_occupancy_grid(map_image_combined)
            
        for robot_id in robot_ids:
            updated_map_msg = OccupancyGrid()
            updated_map_msg.header = self.node.map_data_from_nav2.header
            updated_map_msg.info = self.node.map_data_from_nav2.info
            updated_map_msg.data = updated_map_data
            topic_path = self.parse_robot_id(robot_id, format_type='topic')
            channel_publisher = self.node.create_publisher(OccupancyGrid, topic_path + 'map', self.node.publisher.qos_profile)
            self.node.get_logger().info(f'Publishing updated map to {topic_path}map...')
            channel_publisher.publish(updated_map_msg)


    def wait_for_area_image(self, timeout=16.0, check_interval=0.1):
        """Waits for the area_image to become available or until timeout."""
        start_time = time.time()
        while self.area_image is None and (time.time() - start_time) < timeout:
            time.sleep(check_interval)
        return self.area_image is not None
    
    def image_to_occupancy_grid(self, image):
        if image.ndim > 2:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Normalize the image to ensure it's binary
        _, image = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY)
        
        flat_image = image.flatten()
        occupancy_data = np.zeros(flat_image.shape, dtype=np.int8)
        
        occupancy_data[flat_image == 0] = 100  # Obstacle
        occupancy_data[flat_image == 255] = 0  # Free space
        occupancy_data[(flat_image != 0) & (flat_image != 255)] = -1  # Unknown space

        occupancy_data = np.clip(occupancy_data, -128, 127)
    
        return occupancy_data.tolist()

    def calculate_offsets_for_robots(self, robot_ids, dump_truck_count):
        offsets = []
        for robot_id in robot_ids:
            robot_type, numeric_part = self.parse_robot_id(robot_id, format_type='numeric')
            
            if robot_type == "dump_truck":
                unity_x = 4 * numeric_part
                unity_y = 0
                unity_z = 0
                # For L2_T2
                unity_x += (-8)
                unity_z += (+3)
                
            elif robot_type == "excavator":
                base_offset = 4 * dump_truck_count
                if numeric_part % 2 == 0:  # Even-numbered excavators are on the left side, e.g. [0] excavator_1
                    unity_x = -4 * (numeric_part // 2 + 1)
                    unity_y = 0 
                    unity_z = 0
                else:  # Odd-numbered excavators are on the right side, e.g. [1] excavator_2
                    unity_x = base_offset + 4 * (numeric_part // 2)
                    unity_y = 0
                    unity_z = 0

            # Convert from Unity to ROS2 coordinates
            point = self.unity_to_ros2(unity_x, unity_y, unity_z)
            ros2_x = point.x
            ros2_y = point.y
        
            if self.use_toy_robots:
                ros2_x = ros2_x / 20
                ros2_y = ros2_y / 20

            offsets.append((ros2_x, ros2_y))
        
        return offsets

    def unity_to_ros2(self, unity_x, unity_y, unity_z):
        """
        Convert a point from Unity coordinate system to ROS2 coordinate system.
        Args:
            unity_x (loat): X coordinate in Unity.
            unity_y (float): Y coordinate in Unity.
            unity_z (float): Z coordinate in Unity.
        Returns:
            Point: A Point object in ROS2 coordinate system.
        """
        # Convert Unity coordinates to ROS2 coordinates
        ros2_x = unity_z
        ros2_y = -unity_x
        ros2_z = unity_y
        return Point(x=float(ros2_x), y=float(ros2_y), z=float(ros2_z))
    
    def ros2_to_unity(self, ros2_x, ros2_y, ros2_z):
        """
        Convert a point from ROS2 coordinate system to Unity coordinate system.
        Args:
            ros2_x (float): X coordinate in ROS2.
            ros2_y (float): Y coordinate in ROS2.
            ros2_z (float): Z coordinate in ROS2.
        Returns:
            Point: A Point object in Unity coordinate system.
        """
        # Convert ROS2 coordinates to Unity coordinates
        unity_x = -ros2_y
        unity_y = ros2_z
        unity_z = ros2_x
        return Point(x=float(unity_x), y=float(unity_y), z=float(unity_z))

    def parse_robot_id(self, robot_name, format_type='topic'):
        match = re.search(r"robot_([a-zA-Z_]+)_(\d+)", robot_name)
        if not match:
            raise ValueError(f"Invalid robot name: {robot_name}")

        robot_type = match.group(1)
        number = int(match.group(2)) - 1

        if format_type == 'numeric':
            return robot_type, number
        else:
            type_prefix = 'zx120' if 'excavator' in robot_type else 'c30r'
            return f'/{type_prefix}_{number}/'

    async def publish_goal_for_robot(self, robot_id, goal_position, task_name):
        now = ROSClock().now()
        goal = PoseStamped(
            header=Header(frame_id="map", stamp=now.to_msg()),
            pose=Pose(
                position=goal_position,
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            )
        )
        
        topic_path = self.parse_robot_id(robot_id, format_type='topic')
        publisher = self.node.create_publisher(PoseStamped, topic_path + 'goal_pose', 10)
        publisher.publish(goal)

        self.robot_goals[robot_id] = goal_position
        if task_name in self.task_status:
            if robot_id in self.task_status[task_name]:
                self.task_status[task_name][robot_id]['goal'] = goal_position
                self.task_status[task_name][robot_id]['reached'] = False
            else:
                self.task_status[task_name][robot_id] = {'goal': goal_position, 'reached': False}
        else:
            self.task_status[task_name] = {robot_id: {'goal': goal_position, 'reached': False}}



    def calculate_start_position(self, robot_type, number, dump_truck_count):
        if robot_type == "dump_truck":
            unity_start_x = -16 + number * 8
            unity_start_y = 0.0
            unity_start_z = -20.0
        elif robot_type == "excavator":
            base_offset = 8 * dump_truck_count
            self.node.get_logger().info(f'dump_truck_count: {dump_truck_count}')
            if number % 2 == 0: # Even-numbered excavators are on the left side, e.g. [0] excavator_1
                unity_start_x = -16 - ((number // 2 + 1) * 8)
            else: # Odd-numbered excavators are on the right side, e.g. [1] excavator_2
                unity_start_x = -16 + base_offset + ((number // 2) * 8)
            unity_start_y = 0.0
            unity_start_z = -20.0
        else:
            raise ValueError(f"Unknown robot type: {robot_type}")
        return self.unity_to_ros2(unity_start_x, unity_start_y, unity_start_z)