import time
import re
import math
from .config import ROBOT_NAMES
from std_msgs.msg import Float64, Float32MultiArray, Float32
import asyncio

def degrees_to_radians(degrees):
    return degrees * (math.pi / 180)

class RobotFunctions:
    def __init__(self, node, use_toy_robots):
        self.node = node
        self.use_toy_robots = use_toy_robots
        self.joint_publishers = {}
        self.task_status = {} # {task_name: {robot_id: status}}

    """
    swing_joint->boom_joint->arm_joint->bucket_joint
    body_link  ->boom_link-> arm_link-> bucket_link


      S. : State          T. A.: Target Angle
    | S. | Joint        | T. A. | Possible Meaning                                           |
    |---:|:-------------|------:|:-----------------------------------------------------------|

    |  0 | arm_joint    |   -25 | Move the arm downwards to prepare for digging              |
    |  1 | swing_joint  |     0 | Rot. the exc. to align the bucket with the digging target  |

    |  2 | boom_joint   |   -20 | Lift down the robot arm to bring the buc. closer to the gro.|
    |  3 | boom_joint   |    10 | Keep lifting down the robot arm                            |
    |  4 | bucket_joint |   150 | Move the boom upwards to let the bucket carry the soil     |

    |  5 | boom_joint   |   -60 | Lift up the robot arm                                      |
    |  6 | arm_joint    |   -30 | Keep lifting up the robot arm                              |

    |  7 | swing_joint  |    80 | Rot. the exc. to transport the soil to another place       |
    |  8 | bucket_joint |     0 | Release the soil from the bucket                           |
    """
    async def Excavation(self, _target_object, robot_ids, task_name):
        self.task_status[task_name] = {}
        for robot_id in robot_ids:
            if not self.use_toy_robots:
                # Move back to initial position
                # Step 1: Move back to initial position
                self.send_joint_command(robot_id, 'boom_joint', -60)  # Move boom_joint back to 90 degrees (Joint 1)
                await self.display_waiting_and_output_task_progress(1, "[EE Step 1] Moving boom_joint (Joint 1) to 90 degrees...", robot_id)

                # Step 2: Move arm_joint back to 0 degrees
                self.send_joint_command(robot_id, 'arm_joint', 90)  # Move arm_joint back to 0 degrees (Joint 2)
                await self.display_waiting_and_output_task_progress(1, "[EE Step 2] Moving arm_joint (Joint 2) to 0 degrees...", robot_id)

                # Step 3: Move swing_joint back to 0 degrees
                self.send_joint_command(robot_id, 'swing_joint', 0)  # Move swing_joint back to 0 degrees (Joint 0)
                await self.display_waiting_and_output_task_progress(1, "[EE Step 3] Moving swing_joint (Joint 0) to 0 degrees...", robot_id)

                # Step 4: Move bucket_joint back to 50 degrees
                self.send_joint_command(robot_id, 'bucket_joint', 45)  # Move bucket_joint back to 50 degrees (Joint 3)
                await self.display_waiting_and_output_task_progress(1, "[EE Step 4] Moving bucket_joint (Joint 3) to 50 degrees...", robot_id)




                # Step 5: Move arm_joint to -25 degrees
                self.send_joint_command(robot_id, 'arm_joint', -25)  # Move arm_joint to -25 degrees (Joint 2)
                await self.display_waiting_and_output_task_progress(1, "[EE Step 5] Moving arm_joint (Joint 2) to -25 degrees...", robot_id)

                # Step 6: Move swing_joint to 0 degrees
                self.send_joint_command(robot_id, 'swing_joint', 0)  # Move swing_joint to 0 degrees (Joint 0)
                await self.display_waiting_and_output_task_progress(1, "[EE Step 6] Moving swing_joint (Joint 0) to 0 degrees...", robot_id)

                # Step 7: Move boom_joint to -20 degrees
                self.send_joint_command(robot_id, 'boom_joint', 20)  # Move boom_joint to 20 degrees (Joint 1)
                await self.display_waiting_and_output_task_progress(3, "[EE Step 7] Moving boom_joint (Joint 1) to -20 degrees...", robot_id)

                # Step 8: Move boom_joint to 10 degrees
                self.send_joint_command(robot_id, 'boom_joint', 10)  # Move boom_joint to 10 degrees (Joint 1)
                await self.display_waiting_and_output_task_progress(1, "[EE Step 8] Moving boom_joint (Joint 1) to 10 degrees...", robot_id)

                # Step 9: Move bucket_joint to 150 degrees
                self.send_joint_command(robot_id, 'bucket_joint', 150)  # Move bucket_joint to 150 degrees (Joint 3)
                await self.display_waiting_and_output_task_progress(1, "[EE Step 9] Moving bucket_joint (Joint 3) to 150 degrees...", robot_id)


                self.node.get_logger().info(f"\033[1;32mExcavation operation for robot {robot_id} is completed.\033[0m")
            else:
                await self.publish_toy_robot_commands_excavation(robot_id)
            self.task_status[task_name][robot_id] = True


    async def ExcavatorUnloading(self, _target_object, robot_ids, task_name):
        self.task_status[task_name] = {}
        for robot_id in robot_ids:
            if not self.use_toy_robots:
                # Step 1: Move boom_joint to -60 degrees
                self.send_joint_command(robot_id, 'boom_joint', -60)  # -60 degrees
                await self.display_waiting_and_output_task_progress(2, "[EU Step 1] Moving boom_joint (Joint 1) to -60 degrees", robot_id)

                # Step 2: Move arm_joint to -30 degrees
                self.send_joint_command(robot_id, 'arm_joint', -30)  # -30 degrees
                await self.display_waiting_and_output_task_progress(2, "[EU Step 2] Moving arm_joint (Joint 2) to -30 degrees", robot_id)

                # Step 3: Move swing_joint to 80 degrees
                self.send_joint_command(robot_id, 'swing_joint', 80)  # 80 degrees
                await self.display_waiting_and_output_task_progress(3, "[EU Step 3] Moving swing_joint (Joint 0) to 80 degrees", robot_id)

                # Step 4: Move bucket_joint to 0 degrees
                self.send_joint_command(robot_id, 'bucket_joint', 0)  # 0 degrees
                await self.display_waiting_and_output_task_progress(3, "[EU Step 4] ExcavatorUnloading: Moving bucket_joint to 0 degrees", robot_id)




                # Move back to initial position
                # Step 5: Move boom_joint back to 90 degrees
                self.send_joint_command(robot_id, 'boom_joint', -60)  # Move boom_joint back to 90 degrees
                await self.display_waiting_and_output_task_progress(1, "[EU Step 5] Returning to initial position: Moving boom_joint to 90 degrees", robot_id)

                # Step 6: Move arm_joint back to 0 degrees
                self.send_joint_command(robot_id, 'arm_joint', 90)  # Move arm_joint back to 0 degrees
                await self.display_waiting_and_output_task_progress(1, "[EU Step 6] Returning to initial position: Moving arm_joint to 0 degrees", robot_id)

                # Step 7: Move swing_joint back to 0 degrees
                self.send_joint_command(robot_id, 'swing_joint', 0)  # Move swing_joint back to 0 degrees
                await self.display_waiting_and_output_task_progress(1, "[EU Step 7] Returning to initial position: Moving swing_joint to 0 degrees", robot_id)

                # Step 8: Move bucket_joint back to 50 degrees
                self.send_joint_command(robot_id, 'bucket_joint', 45)  # Move bucket_joint back to 50 degrees
                await self.display_waiting_and_output_task_progress(1, "[EU Step 8] Returning to initial position: Moving bucket_joint to 50 degrees", robot_id)


                self.node.get_logger().info(f"\033[1;32mUnloading operation for robot {robot_id} is completed.\033[0m")
            else:
                await self.publish_toy_robot_commands_unloading(robot_id)
            self.task_status[task_name][robot_id] = True


    async def DumpLoading(self, _target_object, robot_ids, task_name):
        self.task_status[task_name] = {}
        for robot_id in robot_ids:
            if not self.use_toy_robots:
                # Step 1: Move vessel_joint to 0 degrees
                self.send_joint_command(robot_id, 'vessel_joint', 0)  # 0 degrees
                await self.display_waiting_and_output_task_progress(3, "[DL Step 1] Moving vessel_joint to 0 degrees", robot_id)
                self.send_joint_command(robot_id, 'vessel_joint', -85)  # -85 degrees
            else:
                await self.publish_toy_robot_dump_loading(robot_id)
            self.task_status[task_name][robot_id] = True



    async def DumpUnloading(self, _target_object, robot_ids, task_name):
        self.task_status[task_name] = {}
        for robot_id in robot_ids:
            if not self.use_toy_robots:
                # Step 1: Move vessel_joint to -85 degrees
                self.send_joint_command(robot_id, 'vessel_joint', -85)  # -85 degrees
                await self.display_waiting_and_output_task_progress(3, "[DU Step 1] Moving vessel_joint to -85 degrees", robot_id)

                # Step 2: Move vessel_joint back to 0 degrees
                self.send_joint_command(robot_id, 'vessel_joint', 0)  # 0 degrees
                await self.display_waiting_and_output_task_progress(3, "[DU Step 2] Moving vessel_joint to 0 degrees", robot_id)

                self.node.get_logger().info(f"\033[1;32mDumpUnloading operation for robot {robot_id} is completed.\033[0m")
            else:
                await self.publish_toy_robot_dump_unloading(robot_id)
            self.task_status[task_name][robot_id] = True


    async def publish_toy_robot_commands_excavation(self, robot_id):
        self.create_publishers_for_toy_excavator(robot_id)
        
        msg_rot = Float32MultiArray()
        msg_boom = Float32MultiArray()
        msg_arm = Float32MultiArray()
        msg_bkt = Float32MultiArray()

        # Return to initial pose
        msg_rot.data = [0.0, 0.5]
        self.pub_rot.publish(msg_rot)
        msg_boom.data = [-60.0, 0.7]
        self.pub_boom.publish(msg_boom)
        msg_arm.data = [90.0, 0.5]
        self.pub_arm.publish(msg_arm)
        msg_bkt.data = [60.0, 0.5]
        self.pub_bkt.publish(msg_bkt)
        self.node.get_logger().info(f"\033[1;32mInitial pose for {robot_id} is set.\033[0m")

        # Approach
        msg_boom.data = [-10.0, 0.3]
        self.pub_boom.publish(msg_boom)
        msg_arm.data = [60.0, 0.5]
        self.pub_arm.publish(msg_arm)
        msg_bkt.data = [30.0, 0.5]
        await asyncio.sleep(5.0)
        self.node.get_logger().info(f"\033[1;32mApproach operation for {robot_id} is completed.\033[0m")

        # Excavate
        msg_arm.data = [80.0, 0.9]
        self.pub_arm.publish(msg_arm)
        await asyncio.sleep(2.0)
        msg_bkt.data = [90.0, 0.9]
        self.pub_bkt.publish(msg_bkt)
        await asyncio.sleep(2.0)
        msg_boom.data = [-35.0, 0.7]
        self.pub_boom.publish(msg_boom)
        await asyncio.sleep(2.0)
        self.node.get_logger().info(f"\033[1;32mExcavation operation for {robot_id} is completed.\033[0m")

        # Return to initial pose except for the bucket
        msg_rot.data = [0.0, 0.5]
        self.pub_rot.publish(msg_rot)
        msg_boom.data = [-60.0, 0.7]
        self.pub_boom.publish(msg_boom)
        msg_arm.data = [90.0, 0.5]
        self.pub_arm.publish(msg_arm)
        await asyncio.sleep(2.0)




    async def publish_toy_robot_commands_unloading(self, robot_id):
        self.create_publishers_for_toy_excavator(robot_id)
        
        msg_rot = Float32MultiArray()
        msg_boom = Float32MultiArray()
        msg_arm = Float32MultiArray()
        msg_bkt = Float32MultiArray()

        # Swing
        msg_rot.data = [80.0, 0.7]
        self.pub_rot.publish(msg_rot)
        await asyncio.sleep(3.0)

        # Drop
        msg_bkt.data = [0.0, 0.5]
        self.pub_bkt.publish(msg_bkt)
        await asyncio.sleep(3.0)

        # Return to initial pose
        msg_rot.data = [0.0, 0.5]
        self.pub_rot.publish(msg_rot)
        msg_boom.data = [-60.0, 0.7]
        self.pub_boom.publish(msg_boom)
        msg_arm.data = [110.0, 0.5]
        self.pub_arm.publish(msg_arm)
        msg_bkt.data = [90.0, 0.5]
        self.pub_bkt.publish(msg_bkt)

    async def publish_toy_robot_dump_loading(self, robot_id):
        self.create_publishers_for_toy_dump_truck(robot_id)
        
        msg_vessel = Float32()
        msg_vessel.data = 0.0
        self.pub_vessel.publish(msg_vessel)
        await asyncio.sleep(3.0)
        self.node.get_logger().info(f"\033[1;32mLoading operation for {robot_id} is completed.\033[0m")

    async def publish_toy_robot_dump_unloading(self, robot_id):
        self.create_publishers_for_toy_dump_truck(robot_id)
        
        msg_vessel = Float32()
        msg_vessel.data = 90.0
        self.pub_vessel.publish(msg_vessel)
        await asyncio.sleep(6.0)
        msg_vessel.data = 0.0
        self.pub_vessel.publish(msg_vessel)
        self.node.get_logger().info(f"\033[1;32mUnloading operation for {robot_id} is completed.\033[0m")

    def parse_robot_id(self, robot_id, format_type='namespace'):
        match = re.search(r"robot_([a-zA-Z_]+)_(\d+)", robot_id)
        if not match:
            raise ValueError(f"Invalid robot name: {robot_id}")

        robot_type = match.group(1)
        number = int(match.group(2)) - 1

        if format_type == 'numeric':
            return robot_type, number
        else:
            type_prefix = 'zx120' if 'excavator' in robot_type else 'c30r'
            return f'/{type_prefix}_{number}/'

    def create_publishers_for_toy_excavator(self, robot_id):
        rot_topic, boom_topic, arm_topic, bkt_topic = self.get_toy_excavator_topics(robot_id)
        self.pub_rot = self.node.create_publisher(Float32MultiArray, rot_topic, 10)
        self.pub_boom = self.node.create_publisher(Float32MultiArray, boom_topic, 10)
        self.pub_arm = self.node.create_publisher(Float32MultiArray, arm_topic, 10)
        self.pub_bkt = self.node.create_publisher(Float32MultiArray, bkt_topic, 10)

    def create_publishers_for_toy_dump_truck(self, robot_id):
        vessel_topic = self.get_toy_dump_truck_topic(robot_id)
        self.pub_vessel = self.node.create_publisher(Float32, vessel_topic, 10)
        self.node.get_logger().info(f"\033[1;32mPublishers for truck with ID {robot_id} are created.\033[0m")

    def get_toy_excavator_topics(self, robot_id):
        ex_id = int(robot_id.split('_')[-1]) + 114 - 1
        rot_topic = f"/ex_{ex_id}/cmd/rotator/angle"
        boom_topic = f"/ex_{ex_id}/cmd/boom/angle"
        arm_topic = f"/ex_{ex_id}/cmd/arm/angle"
        bkt_topic = f"/ex_{ex_id}/cmd/bucket/angle"
        self.node.get_logger().info(f"\033[1;32mTopics for excavator with ID {robot_id}: {rot_topic}, {boom_topic}, {arm_topic}, {bkt_topic}\033[0m")
        return rot_topic, boom_topic, arm_topic, bkt_topic

    def get_toy_dump_truck_topic(self, robot_id):
        du_id = int(robot_id.split('_')[-1]) + 120
        if du_id == 121:
            du_id = 123 # Change to the real ID on the toy truck
        elif du_id == 122:
            du_id = 124 # Change to the real ID on the toy truck
        elif du_id == 123:
            du_id = 125 # Change to the real ID on the toy truck
        vessel_topic = f"/du_{du_id}/cmd/vessel/ang"
        self.node.get_logger().info(f"\033[1;32mVessel topic for truck with ID {robot_id}: {vessel_topic}\033[0m")
        return vessel_topic

    def publish_joint_command_for_excavator(self, robot_id):
        robot_ns = self.parse_robot_id(robot_id, format_type='namespace')
        publishers = {
            'arm_joint': self.node.create_publisher(Float64, f'{robot_ns}arm/cmd', 10),
            'swing_joint': self.node.create_publisher(Float64, f'{robot_ns}swing/cmd', 10),
            'boom_joint': self.node.create_publisher(Float64, f'{robot_ns}boom/cmd', 10),
            'bucket_joint': self.node.create_publisher(Float64, f'{robot_ns}bucket/cmd', 10)
        }
        return publishers

    def publish_joint_command_for_dump_truck(self, robot_id):
        robot_ns = self.parse_robot_id(robot_id, format_type='namespace')
        publishers = {
            'vessel_joint': self.node.create_publisher(Float64, f'{robot_ns}vessel/cmd', 10)
        }
        return publishers

    def send_joint_command(self, robot_id, joint_name, angle):
        msg = Float64()
        msg.data = float(degrees_to_radians(angle))  # Convert degrees to radians
        if robot_id not in self.joint_publishers:
            self.create_publishers_for_robot(robot_id)
        self.joint_publishers[robot_id][joint_name].publish(msg)

    def create_publishers_for_robot(self, robot_id):
        robot_type, _ = self.parse_robot_id(robot_id, format_type='numeric')
        if robot_type == "excavator":
            self.joint_publishers[robot_id] = self.publish_joint_command_for_excavator(robot_id)
        elif robot_type == "dump_truck":
            self.joint_publishers[robot_id] = self.publish_joint_command_for_dump_truck(robot_id)

    async def display_waiting_and_output_task_progress(self, remaining_time, task_name, robot_id):
        for i in range(remaining_time, 0, -1):
            self.node.get_logger().info(f"\033[1;36mRobot {robot_id} - {task_name}: {i} [s] remaining...\033[0m")

            await asyncio.sleep(1)
