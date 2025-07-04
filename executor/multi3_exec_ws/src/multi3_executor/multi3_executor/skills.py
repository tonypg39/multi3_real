#!/usr/bin/env python3
import rclpy
import time
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from threading import Event
import json
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# It includes always wait and send as base skills
#FIXME: Add a meta-class for all the Skills to have a common interface (SE-FIX)

def estimate_mov_time(pos_a, pos_b, velocity):
    dist = math.sqrt((pos_a[0] - pos_b[0])**2 + (pos_a[0] - pos_b[1])**2)
    t = dist / velocity
    return t


class Nav2Navigator():

    def __init__(self, node, finish_event) -> None:
        self.node = node
        self.finish_event = finish_event
        self.nav_success = False
        self._action_client = ActionClient(self.node, NavigateToPose, f'/{self.node.robot_name}/navigate_to_pose', callback_group=self.node.callback_group)
        self.node.get_logger().info('Initializing the navigator')

    def send_goal(self, tgoal):
        # Set up the goal pose
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        self.node.get_logger().info(f'Sending to Nav: {str(tgoal[0])}|{str(tgoal[1])}')
        goal_pose.pose.pose.position.x = tgoal[0]
        goal_pose.pose.pose.position.y = tgoal[1]
        goal_pose.pose.pose.orientation.z = tgoal[2]
        # goal_pose.pose.pose.orientation.w = spot['orientation']['w']

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_pose,feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info(f'Goal was rejected.{goal_handle}')
            self.finish_event.set()
            return
        self.node.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.nav_success = True
        self.node.get_logger().info(f'Goal result received.')
        self.finish_event.set()
        #rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        self.node.get_logger().info(f"Feedback: {feedback_msg.feedback}")


class NativeNavigator():
    
    def __init__(self, node, finish_event, finish_mock_skill) -> None:
        self.node = node
        self.finish_event = finish_event
        self.finish_mock_skill = finish_mock_skill
        
        if self.node.real_robot_namespace is None:
            self.node.get_logger().fatal("The turtlebot name is NOT assigned for the executor node")
            raise ValueError("The turtlebot name is NOT assigned for the executor node")
        
        self.cmd_pub = self.node.create_publisher(Twist, f'/{self.node.real_robot_namespace}/cmd_vel', 10)
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.odom_sub = self.node.create_subscription(
            Odometry,
            f'/{self.node.real_robot_namespace}/odom',
            self.odom_callback,
            qos,
            callback_group=self.node.callback_group
        )

        # Control parameters
        self.control_params = {
            "k_linear": 0.12,
            "k_angular": 0.34,
            "max_linear": 0.7,
            "dist_threshold": 0.4,
            "sample_period": 0.35
        }

        self.current_pose = None
        self.goal_pose = None
        self.spin_goal = None
        self.spin_timesteps = 0

        self.timer = self.node.create_timer(self.control_params["sample_period"], self.control_loop, callback_group=self.node.callback_group)
        self.node.get_logger().info("Navigator ready for goal commands")


    def set_goal_pose(self, goal):
        # Initialize the cumulative variables
        self.p_ang = 0.0
        self.p_yaw = 0.0
        self.goal_pose = goal
        self.node.get_logger().info(f"Executing go to <goal: {goal}>")
    
    def set_spin_goal(self, spin_goal):
        self.spin_goal = spin_goal # sg = {"spin_speed": 1.5, "time_steps"}

    def odom_callback(self, msg):
        # self.node.get_logger().info(f"Updating pose...")
        self.current_pose = msg.pose.pose
    
    def get_yaw_from_quaternion(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def wrap_angle(self, previous_angle, new_angle):
        """
        Adjust the new_angle based on the previous_angle to maintain continuity.
        Assumes angles are in radians.
        """
        angle_difference = new_angle - previous_angle
        if angle_difference > math.pi:
            new_angle -= 2 * math.pi
        elif angle_difference < -math.pi:
            new_angle += 2 * math.pi
        return new_angle

    def control_loop(self):
        # self.node.get_logger().info(f"Control Step! Goal Pose: {self.goal_pose} || Cur Pose: {self.current_pose}")
        
        if self.spin_goal is not None:
            vel_msg = Twist()
            vel_msg.angular.z = self.spin_goal["angular_speed"]
            if self.spin_timesteps < self.spin_goal["time_steps"]:
                self.node.get_logger().info(f"Task related spinning. Progress = {(self.spin_timesteps/self.spin_goal['time_steps'])*100}%")
                self.cmd_pub.publish(vel_msg)
                self.spin_timesteps += 1
                
            else:
                self.finish_mock_skill.set()
                self.spin_goal = None
                self.spin_timesteps = 0

        if self.goal_pose is None or self.current_pose is None:
            return
        
        # self.node.get_logger().info("Control Step2!")
        
        yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        x,y = self.current_pose.position.x,self.current_pose.position.y

        #Take a single control step
        yaw = self.wrap_angle(self.p_yaw, yaw)
        self.p_yaw = yaw

        # Linear Velocity
        k_linear = self.control_params['k_linear']
        dx = self.goal_pose["x"] - x
        dy = self.goal_pose["y"] - y
        distance = math.hypot(dx, dy)
        linear_speed = min(self.control_params['max_linear'], distance * k_linear)

        # Angular Velocity
        k_angular = self.control_params['k_angular']
        desired_angle_goal = self.wrap_angle(self.p_ang, math.atan2(self.goal_pose["y"]-y,self.goal_pose["x"]-x))
        self.p_ang = desired_angle_goal
        # pos_msg = f"The current angle: {math.degrees(yaw)}  | Desired: {math.degrees(desired_angle_goal)} | Da: {math.degrees(desired_angle_goal-yaw)}"
        # self.node.get_logger().info(pos_msg)
        angular_speed = (desired_angle_goal-yaw)*k_angular

        vel_msg = Twist()
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = angular_speed
        
        
        self.node.get_logger().info(f"The distance to the goal is: {distance}")
        if distance > self.control_params["dist_threshold"]:
            # continue to move the robot
            self.cmd_pub.publish(vel_msg)
        else:
            # stop the robot and send the finish event
            self.goal_pose = None
            self.finish_event.set()

    # helper functions
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def stop_navigator(self):
        self.timer.cancel()



# Skills
class WaitSkill():
    def __init__(self,node, params,finish_event, skill_name="") -> None:
        self.node = node
        self.finish_event = finish_event
        self.params = params
        self.success = False
        #self.subs = self.node.create_subscription(String, '/signal_states', self.update_flags, 10)
        
        wait_str = self.params["target"] 
        self.wait_flags = wait_str.split('&')
        self.wait_for_all = Event()
        self.node.get_logger().info(f"Starting up skill: {self.__class__.__name__}")
        
    
    def wait_for_flags(self):
        while not self.check_flags(self.wait_flags):
            time.sleep(1)
        self.wait_for_all.set()

    def check_flags(self, wf):
        for f in wf:
            if f not in self.node.flags:
                self.node.get_logger().info(f"Did not find flag {f} IN  {self.node.flags}")
                return False
        return True

    def exec(self, virtual_state=None,virtual_effort=None):
        self.wait_for_flags()
        self.wait_for_all.wait()
        self.success = True
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        self.finish_event.set()
        return virtual_state


class SendSkill():
    def __init__(self,node, params, finish_event, skill_name="") -> None:
        self.node = node
        self.finish_event = finish_event
        self.params = params
        self.success = False
    
    def exec(self, virtual_state,virtual_effort=None):
        # It needs params["target"]
        task_id = self.params["target"]
        msg = String()
        msg.data = task_id
        self.node._send_mission_signal(task_id)
        self.success = True
        self.finish_event.set()
        return virtual_state


class BaseSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : base skill
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False

        # Create a navigator obj
        self.wait_for_nav = Event()
        self.wait_for_mock_skill = Event()
        self.nav = NativeNavigator(self.node, self.wait_for_nav, self.wait_for_mock_skill)
        self.node.get_logger().info(f"Starting up skill: {self.__class__.__name__}")
    
    def exec(self,virtual_state=None,virtual_effort=None):
        # It needs: params["room"]["size"]
        print("Received the params: ")
        goal_pos = {
            "x": self.params["location"]["x"],
            "y": self.params["location"]["y"]
        }
        spin_goal = {
            "angular_speed": 0.3,
            "time_steps": 40
        }
        self.nav.set_goal_pose(goal_pos)
        self.wait_for_nav.wait()
        self.nav.set_spin_goal(spin_goal)
        self.wait_for_mock_skill.wait()

        self.success = True
        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        return virtual_state 

class MopSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False

        # Create a navigator obj
        self.wait_for_nav = Event()
        self.wait_for_mock_skill = Event()
        self.nav = NativeNavigator(self.node, self.wait_for_nav, self.wait_for_mock_skill)
        self.node.get_logger().info(f"Starting up skill: {self.__class__.__name__}")
    
    def exec(self,virtual_state=None,virtual_effort=None):
        # It needs: params["room"]["size"]
        print("Received the params: ")
        goal_pos = {
            "x": self.params["location"]["x"],
            "y": self.params["location"]["y"]
        }
        spin_goal = {
            "angular_speed": 0.3,
            "time_steps": 60
        }
        self.nav.set_goal_pose(goal_pos)
        self.wait_for_nav.wait()
        self.nav.set_spin_goal(spin_goal)
        self.wait_for_mock_skill.wait()

        self.success = True
        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        return virtual_state


class VacuumSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False

        # Create a navigator obj
        self.wait_for_nav = Event()
        self.wait_for_mock_skill = Event()
        self.nav = NativeNavigator(self.node, self.wait_for_nav, self.wait_for_mock_skill)
        self.node.get_logger().info("Starting up skill: Vacuum")
    
    def exec(self,virtual_state=None,virtual_effort=None):
        # It needs: params["room"]["size"]
        print("Received the params: ")
        goal_pos = {
            "x": self.params["location"]["x"],
            "y": self.params["location"]["y"]
        }
        spin_goal = {
            "angular_speed": 0.8,
            "time_steps": 60
        }
        self.nav.set_goal_pose(goal_pos)
        self.wait_for_nav.wait()
        self.nav.set_spin_goal(spin_goal)
        self.wait_for_mock_skill.wait()
        self.nav.stop_navigator()
        self.success = True

        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        return virtual_state

class PolishSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False

        # Create a navigator obj
        self.wait_for_nav = Event()
        self.wait_for_mock_skill = Event()
        self.nav = NativeNavigator(self.node, self.wait_for_nav, self.wait_for_mock_skill)
        self.node.get_logger().info("Starting up skill: Polish")
    
    def exec(self,virtual_state=None,virtual_effort=None):
        # It needs: params["room"]["size"]
        print("Received the params: ")
        goal_pos = {
            "x": self.params["location"]["x"],
            "y": self.params["location"]["y"]
        }
        spin_goal = {
            "angular_speed": -0.5,
            "time_steps": 60
        }
        self.nav.set_goal_pose(goal_pos)
        self.wait_for_nav.wait()
        self.nav.set_spin_goal(spin_goal)
        self.wait_for_mock_skill.wait()

        self.success = True
        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        return virtual_state

# Virtual Skills
#SE-FIX: Create a single class VirtualNavigate skill, and since the skill manager instantiates them,
#        you can provide a specific skill name in the constructor

class VMopSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False
        self.node.get_logger().info(f"Starting up skill: {self.__class__.__name__}")

    
    def exec(self, virtual_state=None, virtual_effort=None):
        goal_pos = [self.params["location"]["x"],self.params["location"]["y"],0.0]
        vpos = [virtual_state["x"],virtual_state["y"],virtual_state["z"]]
        
        time_to_goal = estimate_mov_time(vpos, goal_pos, velocity=0.3)
        self.node.get_logger().info(f"Simulating going from [{str(vpos[0])},{str(vpos[1])}] to [{str(goal_pos[0])},{str(goal_pos[1])}]... [{time_to_goal} secs]")
        time.sleep(time_to_goal)
        self.node.get_logger().info(f"Simulating effort in {self.__class__.__name__}  [{virtual_effort} secs]")
        time.sleep(virtual_effort)
        self.success = True
        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        virtual_state = {
            "x": goal_pos[0],
            "y": goal_pos[1],
            "z": goal_pos[2]
        }
        return virtual_state

class VVacuumSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False
        self.node.get_logger().info(f"Starting up skill: {self.__class__.__name__}")

    
    def exec(self, virtual_state=None, virtual_effort=None):
        goal_pos = [self.params["location"]["x"],self.params["location"]["y"],0.0]
        vpos = [virtual_state["x"],virtual_state["y"],virtual_state["z"]]

        time_to_goal = estimate_mov_time(vpos, goal_pos, velocity=0.3)

        self.node.get_logger().info(f"Simulating going from [{str(vpos[0])},{str(vpos[1])}] to [{str(goal_pos[0])},{str(goal_pos[1])}]... [{time_to_goal} secs]")
        time.sleep(time_to_goal)
        self.node.get_logger().info(f"Simulating effort in {self.__class__.__name__}  [{virtual_effort} secs]")
        time.sleep(virtual_effort)
        self.success = True
        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        virtual_state = {
            "x": goal_pos[0],
            "y": goal_pos[1],
            "z": goal_pos[2]
        }
        return virtual_state

class VPolishSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False
        self.node.get_logger().info(f"Starting up skill: {self.__class__.__name__}")

    
    def exec(self, virtual_state=None, virtual_effort=None):
        goal_pos = [self.params["location"]["x"],self.params["location"]["y"],0.0]
        vpos = [virtual_state["x"],virtual_state["y"],virtual_state["z"]]
        time_to_goal = estimate_mov_time(vpos, goal_pos, velocity=0.3)
        self.node.get_logger().info(f"Simulating going from [{str(vpos[0])},{str(vpos[1])}] to [{str(goal_pos[0])},{str(goal_pos[1])}]... [{time_to_goal} secs]")
        time.sleep(time_to_goal)
        self.node.get_logger().info(f"Simulating effort in {self.__class__.__name__}  [{virtual_effort} secs]")
        time.sleep(virtual_effort)
        self.success = True
        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        virtual_state = {
            "x": goal_pos[0],
            "y": goal_pos[1],
            "z": goal_pos[2]
        }
        return virtual_state
    
class VirtualSKill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.skill_name = skill_name
        self.finished_event = finish_event
        self.success = False
        self.node.get_logger().info(f"Starting up skill: {self.skill_name}")
    
    def exec(self, virtual_state=None, virtual_effort=None):
        self.node.get_logger().info(f"Simulating effort in {self.skill_name}  [{virtual_effort} secs]")
        time.sleep(virtual_effort)
        self.success = True
        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.skill_name}")
        return virtual_state



# Skill Manager
# SE-FIX: Move the instantiation of the classes here, to avoid the need to rebring up the navigators
class SkillManager():
    def __init__(self, skill_mask, virtual_mode=True) -> None:
        """
        skill_mask: if specified, then only those skills are created inside the SKmanager
        """
        self.sk_map = {
            "wait_until": WaitSkill,
            "send_signal": SendSkill,
            "mop": VMopSkill if virtual_mode else MopSkill,
            "vacuum": VVacuumSkill if virtual_mode else VacuumSkill,
            "polish": VPolishSkill if virtual_mode else PolishSkill,
            "declutter": VirtualSKill if virtual_mode else BaseSkill,
            "sanitize": VirtualSKill if virtual_mode else BaseSkill,
        }
        general_skills = ["scan_perimeter", "thermal_scan","record_video", "analyze_surveillance_data", "alert_security","follow_movement","capture_image","drone_scan","soil_moisture_analysis","data_analysis","apply_treatment","fertilizer_application","pest_control_spray","irrigation_adjustment","drone_recheck","soil_nutrient_test"]
        for skill in general_skills:
            self.sk_map[skill] = VirtualSKill

        
        self.sk_map = self.filter_skills(self.sk_map,skill_mask)
    
    def filter_skills(self, sk_map, mask):
        if mask == "-": # If no skill mask provided, we left it intact
            return sk_map
        new_sk_map = {}
        sk_list = mask.split(",")
        for k,v in sk_map.items():
            if k in sk_list or k=="send_signal" or k=="wait_until":
                new_sk_map[k] = v
        return new_sk_map

    def skill_map(self):
        return self.sk_map

