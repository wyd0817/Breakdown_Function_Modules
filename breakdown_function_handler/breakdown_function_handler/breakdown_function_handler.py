import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
from breakdown_function_handler_msgs.srv import SetRedImagePath
from std_srvs.srv import SetBool  
import cv2
from std_msgs.msg import String
import json
import time
from .robot_functions import RobotFunctions
from .nav2_functions import Nav2Functions
import asyncio
import threading
from .config import DESCRIPTION_ROBOT, NAVIGATION_FUNCTIONS, ROBOT_NAMES, ROBOT_STATUS, USE_TOY_ROBOTS
from .global_blackboard import Blackboard

class BreakdownFunctionHandlerNode(Node):
    def __init__(self):
        super().__init__('breakdown_function_handler_node')

        # Initialize asyncio event loop and queue
        self.loop = asyncio.get_event_loop()
        self.instruction_queue = asyncio.Queue()

        # Initialize robot functions and navigation functions
        self.robot_functions = RobotFunctions(self, USE_TOY_ROBOTS)
        self.nav2_functions = Nav2Functions(self, USE_TOY_ROBOTS)
        
        # Initialize blackboard for task management
        self.blackboard = Blackboard()

        # Initialize other attributes
        self.received_tasks = []
        self.description_robot = DESCRIPTION_ROBOT
        self.navigation_functions = NAVIGATION_FUNCTIONS
        self.robot_names = ROBOT_NAMES
        
        # Channels to publish the updated map
        # self.channels = [0, 2, 5] 
        self.channels = [0, 1, 2, 3, 4, 5]
        self.robot_status = ROBOT_STATUS 

        # Set up paths
        self.maps_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../src/maps_4_LLM_2_costmap/'))
        self.map_image_from_lib_path = os.path.join(self.maps_path, 'map_image_from_libr.jpg')
        self.map_image_from_clip_path = os.path.join(self.maps_path, 'map_image_from_clip.jpg')

        # Initialize map-related attributes
        self.map_data_from_nav2 = None
        self.merge_red = True
        self.scale_percent = 100
        self.offset_x = 0
        self.offset_y = 0
        self.rotation_angle = 0

        # Set up QoS profile for reliable communication
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Create subscribers
        self.subscription = self.create_subscription(
            OccupancyGrid,
            # '/c30r_0/global_costmap/costmap',
            "/c30r_0/map",
            self.map_callback,
            qos_profile)
        
        self.image_subscription = self.create_subscription(
            Image, 'area_image_topic', self.area_image_callback, QoSProfile(depth=1))
        
        self.keywords_subscription = self.create_subscription(
            String,
            'keywords_topic',
            self.keywords_callback,
            QoSProfile(depth=10))

        self.instruction_subscription = self.create_subscription(
            String,
            'instruction_topic',
            self.instruction_callback,
            QoSProfile(depth=10))

        # Create publishers
        self.publisher = self.create_publisher(
            OccupancyGrid,
            # '/c30r_0/global_costmap/costmap',
            "/c30r_0/map",
            qos_profile)
        
        self.create_timer(1.0, self.check_ready_tasks_timer_callback)

        # Create services
        self.merge_red_service = self.create_service(
            SetBool, 'set_merge_red', self.set_merge_red_callback)
        
        self.service = self.create_service(
            SetRedImagePath, 'set_red_image_path', self.set_red_image_path_callback)

        # Initialize other attributes
        self.bridge = CvBridge()
        self.area_image = None
        self.message_sent = False

        self.get_logger().info('\033[92m' + 'breakdown_function_handler_node initialized successfully!' + '\033[0m')

    def set_merge_red_callback(self, request, response):
        self.merge_red = request.data
        self.message_sent = False
        response.success = True
        response.message = f"Merge red set to: {'enabled' if self.merge_red else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def set_red_image_path_callback(self, request, response):
        self.red_image_path = os.path.join(self.maps_path, request.data)
        response.success = True
        response.message = "Red image path updated successfully"
        self.get_logger().info(f'Red image path set to: {self.red_image_path}')
        return response

    def area_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.red_image = cv_image  # Update the red image used in map processing
            cv2.imwrite(self.map_image_from_clip_path, cv_image)
            self.get_logger().info('\033[92m' + 'Area image received and saved successfully!' + '\033[0m')
            self.area_image = cv_image
        except Exception as e:
            self.get_logger().error('Failed to convert and save image: ' + str(e))

    def map_callback(self, msg):
        try:
            if self.map_data_from_nav2 is None:
                self.map_data_from_nav2 = msg
                map_image_from_nav2 = self.occupancy_grid_to_image(msg)
                # cv2.imwrite(os.path.join(self.maps_path, 'map_image_from_nav2.png'), map_image_from_nav2)
                cv2.imwrite(os.path.join(self.maps_path, 'map_image_from_nav2.pgm'), map_image_from_nav2)

            if not self.message_sent:
                self.message_sent = True
        except Exception as e:
            self.get_logger().error(f'Error processing map: {e}')

    def occupancy_grid_to_image(self, msg):
        width, height = msg.info.width, msg.info.height
        map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        image = np.zeros((height, width), dtype=np.uint8)
        image[map_data == 0] = 255
        image[map_data == 100] = 0
        image[map_data == -1] = 127
        return image

    def get_idle_robots(self, robot_types):
        idle_robots = []
        for robot_type in robot_types:
            prefix = ''
            if robot_type == 'dump_truck':
                prefix = 'robot_dump_truck_'
            elif robot_type == 'excavator':
                prefix = 'robot_excavator_'
            else:
                self.get_logger().error(f'Unknown robot type: {robot_type}')
                continue
            idle_robots.extend([robot_id for robot_id, status in self.robot_status.items() if status == "idle" and robot_id.startswith(prefix)])
        return idle_robots

    def update_robot_status(self, robot_id, status):
        if robot_id in self.robot_status:
            self.robot_status[robot_id] = status
            self.get_logger().info('\033[95m' + f'Updated {robot_id} status to {status}' + '\033[0m')

    def keywords_callback(self, msg):
        try:
            keywords = json.loads(msg.data)
            self.get_logger().info('\033[92m' + f'Received keywords: {keywords}' + '\033[0m')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode keywords: {str(e)}')

    def standardize_robot_ids(self, robot_ids):
        # self.get_logger().info('\033[95m' + f'Initial Robot IDs: {robot_ids}' + '\033[0m')
        converted_robot_ids = []
        for robot_id in robot_ids:
            cleaned_id = robot_id.strip()
            if cleaned_id in ROBOT_NAMES:
                converted_robot_ids.append(ROBOT_NAMES[cleaned_id]["id"])
            else:
                self.get_logger().error(f'Invalid or unrecognized robot ID: {cleaned_id}')
        # self.get_logger().info('\033[95m' + f'Converted Robot IDs: {converted_robot_ids}' + '\033[0m')
        return converted_robot_ids

    def get_function(self, func_name):
        """
        Get the function object based on the function name.
        This method searches for the function in the node, robot_functions, and nav2_functions.
        """
        if hasattr(self, func_name):
            return getattr(self, func_name)
        elif hasattr(self.robot_functions, func_name):
            return getattr(self.robot_functions, func_name)
        elif hasattr(self.nav2_functions, func_name):
            return getattr(self.nav2_functions, func_name)
        return None
    
    def select_robots(self, robot_type, robot_count):
        """
        Select robots based on type and count.
        This method handles the logic for selecting appropriate robots for a task.
        """
        if robot_count == 'all':
            # Handle both string and list for robot_type
            if isinstance(robot_type, list):
                selected_robots = [robot_id for robot_id in self.robot_status.keys() if any(rt in robot_id for rt in robot_type)]
            else:
                selected_robots = [robot_id for robot_id in self.robot_status.keys() if robot_type in robot_id]
            
            return selected_robots
        
        else:
            # Handle both string and list for robot_type
            if isinstance(robot_type, list):
                idle_robots = [robot_id for robot_id in self.robot_status.keys() 
                            if any(rt in robot_id for rt in robot_type) and self.robot_status[robot_id] == 'idle']
            else:
                idle_robots = [robot_id for robot_id in self.robot_status.keys() 
                            if robot_type in robot_id and self.robot_status[robot_id] == 'idle']
            if len(idle_robots) >= robot_count:
                return idle_robots[:robot_count]
            else:
                self.get_logger().error(f'Not enough idle robots available for type {robot_type}')
                return []

    def instruction_callback(self, msg):
        """
        Callback for receiving instructions.
        This method is called when a new instruction is received from the 'instruction_topic'.
        """
        try:
            instruction = json.loads(msg.data)
            # Use run_coroutine_threadsafe to safely schedule the coroutine from this thread
            asyncio.run_coroutine_threadsafe(self.process_instruction(instruction), self.loop)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode instruction: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error processing instruction: {str(e)}')


    async def process_instruction(self, instruction):
        """
        Process a single instruction.
        This method handles the logic for processing an individual instruction,
        including robot selection and task creation.
        """
        task_id = f"task_{time.time()}"
        task_name = instruction.get('task_name', task_id)
        callback_fun_name = instruction.get('name', task_id)
        target_object = instruction.get('object_keywords', 'puddle1')
        
        robot_ids = instruction.get('robot_ids', [])

        if not robot_ids:
            robot_type = instruction.get('robot_type', 'dump_truck')
            robot_count = instruction.get('robot_count', 'all')
            robot_ids = self.select_robots(robot_type, robot_count)

        dependencies = instruction.get('dependencies', [])

        # Check if the task is already executing
        existing_task = await self.blackboard.get_task_info(task_id)
        if existing_task and not existing_task.get('completed', False):
            self.get_logger().info(f"Task {task_id} is already executing. Skipping.")
            return

        # Convert dependency task names to task IDs, considering only uncompleted tasks
        converted_dependencies = []
        async with self.blackboard.lock:
            for dep_name in dependencies:
                for node_id, data in self.blackboard.graph.nodes(data=True):
                    task_info = data.get('task_info', {})
                    if task_info.get('task_name') == dep_name and not data.get('completed', False):
                        converted_dependencies.append(node_id)
                        break
                else:
                    self.get_logger().warning(f"Dependency task '{dep_name}' not found among uncompleted tasks on blackboard.")

        task_info = {
            'task_id': task_id,
            'task_name': task_name,
            'callback_fun_name': callback_fun_name,
            'target_object': target_object,
            'robot_ids': robot_ids,
            'dependencies': converted_dependencies
        }

        self.get_logger().info(f'Creating task info: {task_info}')

        await self.blackboard.add_task(task_id, task_info)


    async def execute_task(self, task_info):
        task_id = task_info['task_id']
        task_name = task_info['task_name']
        func_name = task_info['callback_fun_name']
        target_object = task_info.get('target_object', ['puddle1'])
        dependencies = task_info.get('dependencies', [])
            
        while not await self.blackboard.check_dependencies(task_info):
            await asyncio.sleep(1)

        if not self.blackboard.graph.has_node(task_id) or self.blackboard.graph.nodes[task_id].get('completed', False):
            self.get_logger().info(f'Task {task_name} (ID: {task_id}) already completed or removed, skipping execution.')
            return

        self.blackboard.graph.nodes[task_id]['in_progress'] = True

        self.get_logger().info(f'Executing Task: {func_name} (ID: {task_id})')

        func = self.get_function(func_name)
        if func is None:
            self.get_logger().error(f"No such function: {func_name}")
            return

        try:
            if isinstance(target_object, list) and target_object:
                target_object_value = target_object[0]  # Extract the first element
            # If target_object is an empty list or None
            elif target_object in [[], None]:
                target_object_value = None
            # Otherwise, return target_object itself
            else:
                target_object_value = target_object

            if 'robot_ids' in task_info:
                robot_ids = task_info['robot_ids']
                if not isinstance(robot_ids, list):
                    robot_ids = [robot_ids]
                await func(target_object_value, robot_ids, task_id)
            else:
                await func(target_object_value, task_id)

            if func_name not in ['target_area_for_all_robots', 'return_to_start_for_all_robots', 'target_area_for_specific_robots', 'return_to_start_for_specific_robots']:
                print(f"Task {func_name} (ID: {task_id}) completed!")
                await self.blackboard.mark_task_completed(task_id)
            self.get_logger().info(f'Completed Task: {func_name} (ID: {task_id})')

        except Exception as e:
            self.get_logger().error(f"Error executing task {task_name} (ID: {task_id}): {e}")


    def check_ready_tasks_timer_callback(self):
        asyncio.run_coroutine_threadsafe(self.check_and_execute_ready_tasks(), self.loop)

    async def check_and_execute_ready_tasks(self):
        ready_tasks = await self.blackboard.get_ready_tasks()
        for task_info in ready_tasks:
            task_id = task_info['task_id']
            if not await self.blackboard.is_task_completed(task_id):
                dependencies = task_info.get('dependencies', [])
                all_deps_completed = True
                for dep_id in dependencies:
                    if not await self.blackboard.is_task_completed(dep_id):
                        all_deps_completed = False
                        break
                if all_deps_completed:
                    asyncio.create_task(self.execute_task(task_info))

    async def shutdown(self):
        """
        Shutdown the node and cancel all running tasks.
        This method ensures a clean shutdown of the node and its asyncio tasks.
        """
        # Cancel all running tasks
        tasks = [t for t in asyncio.all_tasks(self.loop) if t is not asyncio.current_task()]
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        
        # Stop the event loop
        self.loop.stop()

def main(args=None):
    rclpy.init(args=args)
    node = BreakdownFunctionHandlerNode()

    # Create and start the asyncio event loop in a separate thread
    loop_thread = threading.Thread(target=node.loop.run_forever)
    loop_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
            # Check and execute ready tasks
            node.check_ready_tasks_timer_callback()
            
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown the node and its asyncio tasks
        node.loop.call_soon_threadsafe(lambda: asyncio.create_task(node.shutdown()))
        node.destroy_node()
        rclpy.shutdown()
        
        # Stop the event loop and wait for the thread to finish
        node.loop.call_soon_threadsafe(node.loop.stop)
        loop_thread.join()

if __name__ == '__main__':
    main()