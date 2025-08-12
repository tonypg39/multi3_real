import os
import logging
import rclpy
import random
from datetime import datetime
from flask import Flask, request, jsonify
import threading
from sensor_msgs.msg import BatteryState
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from irobot_create_msgs.srv import ResetPose
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from .skills import SkillManager
from threading import Event
import requests
import json
import time
from ament_index_python import get_package_prefix


COORDINATOR_URL = os.getenv("COORDINATOR_URL", "http://coordinator:5000")
ROBOT_NAME = os.getenv("ROBOT_NAME","robotx")
EXECUTOR_PORT = os.getenv("ROBOT_PORT","6000")
TB_ID = os.getenv("TB_ID","")
TEST_ID = os.getenv("TEST_ID","")
VIRTUAL_MODE = os.getenv("VIRTUAL_MODE","0") == "1"


class FragmentExecutor(Node):
    def __init__(self) -> None:
        super().__init__(f'fragment_exec_node')
        self.robot_name = "robot1"
        self.declare_parameter("skill_list", "-")
        # self.declare_parameter("name", "robot")
        self.declare_parameter("mode", "real")
        self.declare_parameter("test_id", "")
        self.declare_parameter("sample_id", "")

        # self.wait_for_start_trigger()

        self.callback_group = ReentrantCallbackGroup()
        self.skill_list = self.get_parameter("skill_list").value
        # self.robot_name = self.get_parameter("name").value
        self.robot_name = ROBOT_NAME
        self.exec_port = EXECUTOR_PORT
        test_id = self.get_parameter("test_id").value
        sample_id = self.get_parameter("sample_id").value[1:]
        self.virtual_mode = self.get_parameter("mode").value == "virtual" or VIRTUAL_MODE
        self.env_states = None
        self.get_logger().info(f"Starting an exec node [{self.robot_name}] with skills: " + self.skill_list)
        self.flags = []
        #FIXME: Add to json
        self.settings = {
            "heartbeat_period": 3.0
        }
        self.info_task = ""
        self.battery_history = []

        if test_id == "" or sample_id == "":
            # self.get_logger().fatal("Test Id and Sample Id needed in Virtual mode")
            # return
            if TEST_ID == "":
                raise ValueError("Test ID was not received in the Executor Node")
            test_id = TEST_ID
            sample_id = 0
        if self.virtual_mode:
            self.virtual_state = {
                "x": .0,
                "y": .0,
                "z": .0
            }
            self.signal_pub_timer = self.create_timer(5.,self.virtual_battery_callback)
            self.env_states = self.read_env_states(test_id, int(sample_id))
        else:
            self.virtual_state = None
            self.real_robot_namespace = TB_ID
            self.initial_position = self.read_initial_position(test_id)
            self.start_reset_srv()
            self.reset_odometry(self.initial_position)
            

            # Battery recording
            self.get_logger().info("Starting battery subscription...")
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )
            self.subscription = self.create_subscription(
                BatteryState,
                f'/{self.real_robot_namespace}/battery_state',
                self.battery_callback,
                qos
            )

        sk_mg = SkillManager(skill_mask=self.skill_list, virtual_mode=self.virtual_mode)
        self.sk_map = sk_mg.skill_map()

        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        self.app = Flask(__name__)
        self.setup_routes()
        threading.Thread(target=self.run_flask, daemon=True).start()
        self.signal_pub_timer = self.create_timer(self.settings['heartbeat_period'],self._send_heartbeat)
        self.busy = False
    
    def read_initial_position(self, test_id):
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        # print(package_path)
        with open(f"{package_path}/multi3_tests/tests/{test_id}/initial_positions.json") as f:
            positions = json.load(f)
        initial_pos = positions[self.robot_name]
        # self.get_logger().info(f"Positions read = {positions} || Init_pos = {initial_pos} || RobotName = {self.robot_name}")
        return initial_pos
    
    def start_reset_srv(self):
        qos_profile = QoSProfile(
            depth=10,  # increase depth to allow larger history buffer
            history=HistoryPolicy.KEEP_LAST
        )
        service_name = f'/{self.real_robot_namespace}/reset_pose'
        self.reset_pose_client = self.create_client(ResetPose, service_name, qos_profile=qos_profile)

        #Debugging service discovery
        # self.list_services()
        services = self.get_service_names_and_types()
        self.get_logger().info(f'Waiting for service {service_name}...')
        while not self.reset_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {service_name} not available, waiting...')

    def reset_odometry(self, initial_position):
        request = ResetPose.Request()
        quat = quaternion_from_euler(0.0, 0.0, initial_position["yaw"])
        initial_pose = Pose()
        initial_pose.position.x = initial_position["x"]
        initial_pose.position.y = initial_position["y"]
        initial_pose.position.z = 0.0
        initial_pose.orientation.x = quat[0]
        initial_pose.orientation.y = quat[1]
        initial_pose.orientation.z = quat[2]
        initial_pose.orientation.w = quat[3]
        request.pose = initial_pose

        self.get_logger().info(f'Calling {self.real_robot_namespace}/reset_pose with initial pose {request}...')

        future = self.reset_pose_client.call_async(request)

        # Spin until the future is done
        time.sleep(2)
    

    def wait_for_start_trigger(self):
        flag_path = "/tmp/start.flag"
        self.get_logger().info("Waiting for the trigger...")
        rate = self.create_rate(1)
        while not os.path.exists(flag_path):
            rate.sleep()
        
        self.get_logger().info("Trigger Flag detected")


    def setup_routes(self):
        @self.app.route('/get_battery_evolution', methods=['GET'])
        def battery():
            battery_evol = {
                "robot_id" : self.robot_name,
                "evolution": self.battery_history
            }
            return jsonify(battery_evol)

        @self.app.route('/get_signal_states', methods=['POST'])
        def receive_mission_signals():
            data = request.get_json()
            # self.get_logger().info(f"Received signals: {data}")
            self.flags = data
            if "_SHUTDOWN_" in self.flags:
                self.destroy_node()
            return jsonify({"status": "received"})

        @self.app.route('/get_fragment', methods=['POST'])
        def receive_data():
            data = request.get_json()
            self.get_logger().info(f"Received Fragment: {data}")
            response = self.exec(data)
            return jsonify({"status": response})
        
    def run_flask(self):
        self.app.run(host="0.0.0.0", port=self.exec_port, debug=False, use_reloader=False)


    def read_env_states(self, test_id, sample_id):
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        # print(package_path)
        with open(f"{package_path}/multi3_tests/tests/{test_id}/env_states.json") as f:
            envs = json.load(f)
        env_states = envs[sample_id]
        return env_states
    
    
    def _send_execution_response(self, response):
        payload = {
            "response": response
        }
        requests.post(f"{COORDINATOR_URL}/execution_response", json=payload)


    def _send_mission_signal(self, mission_signal):
        payload = {
            "signal": mission_signal
        }
        requests.post(f"{COORDINATOR_URL}/mission_signal", json=payload)

    def _send_heartbeat(self):
        # m = String()
        state = "idle" if not self.busy else "busy " + self.info_task
        data = self.robot_name + "=" + state
        # self.get_logger().info(f"\n\n\nSending Heartbeat!! = Current Task: {self.info_task}"
        payload = {
                "data": data
        }
        requests.post(f"{COORDINATOR_URL}/heartbeat", json=payload)

    
    def battery_callback(self, msg):
        battery_point = {
            "voltage": msg.voltage,
            "pct": msg.percentage,
            "current_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        # self.get_logger().info(f"Battery recorded: {battery_point}")
        self.battery_history.append(battery_point)
    
    def virtual_battery_callback(self):
        battery_point = {
            "voltage": random.random() * 12,
            "pct": random.random() * 100,
            "current_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        self.battery_history.append(battery_point)


    def exec(self, fragment):
        # self._stop_srv()
        self.busy = True
        self._send_heartbeat()
        failure = False
        frag = fragment
        for t in frag["tasks"]:
            self.info_task = t["id"]
            sep = t["id"].find("|")
            mi_sep = t["id"].find("^")
            virtual_effort = None
            if sep > -1: # if it a wait or send, augment the call with the target task(s)
                t["vars"]["target"] = t["id"][sep+1:]
                t["id"] = t["id"][:sep]
            
            elif mi_sep > -1:
                core_task = t["id"][:mi_sep]
                if self.virtual_mode:
                    self.get_logger().info(f"The actual core task is: {core_task}")
                    self.get_logger().info(f"The id is: {int(t['id'][mi_sep+1:])}")
                    self.get_logger().info(f"The env_states are is: {self.env_states}")
                    virtual_effort = self.env_states[core_task][int(t["id"][mi_sep+1:])]
                t["id"] = core_task

            elif t["id"] in self.env_states:
                virtual_effort = self.env_states[t["id"]]
            wait_for_skill = Event()
            self.get_logger().info(f"Starting the skill {t['id']}  with params {t['vars']} and v. effort {virtual_effort}")
            sk = self.sk_map[t["id"]](self, t["vars"],wait_for_skill,skill_name=self.info_task)

            self.virtual_state = sk.exec(self.virtual_state, virtual_effort)
            wait_for_skill.wait()
            failure |= sk.success

        response = {"fragment_id" : frag["fragment_id"]}
        response["execution_code"] = 0 if not failure else 1
        self.busy = False
        self.info_task = ""
        self._send_execution_response(response)
        return response
    
    

def main(args=None):
    rclpy.init(args=args)
    bot_exec = FragmentExecutor()
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(bot_exec)
    mt_executor.spin()
    bot_exec.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
