import os
import logging
import rclpy
import random
from datetime import datetime
from flask import Flask, request, jsonify
import threading
from sensor_msgs.msg import BatteryState
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
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

class FragmentExecutor(Node):
    def __init__(self) -> None:
        super().__init__(f'fragment_exec_node')
        self.robot_name = "robot1"
        self.declare_parameter("skill_list", "-")
        # self.declare_parameter("name", "robot")
        self.declare_parameter("mode", "virtual")
        self.declare_parameter("test_id", "")
        self.declare_parameter("sample_id", "")
        self.callback_group = ReentrantCallbackGroup()
        self.skill_list = self.get_parameter("skill_list").value
        # self.robot_name = self.get_parameter("name").value
        self.robot_name = ROBOT_NAME
        self.exec_port = EXECUTOR_PORT
        test_id = self.get_parameter("test_id").value
        sample_id = self.get_parameter("sample_id").value[1:]
        self.virtual_mode = self.get_parameter("mode").value == "virtual"
        self.env_states = None
        self.get_logger().info(f"Starting an exec node [{self.robot_name}] with skills: " + self.skill_list)

        #FIXME: Add to json
        self.settings = {
            "heartbeat_period": 3.0
        }
        self.info_task = ""
        if self.virtual_mode:
            self.virtual_state = {
                "x": .0,
                "y": .0,
                "z": .0
            }
            if test_id == "" or sample_id == "":
                # self.get_logger().fatal("Test Id and Sample Id needed in Virtual mode")
                # return
                test_id = "test_2_2_t_cleaning"
                sample_id = 0
            self.env_states = self.read_env_states(test_id, int(sample_id))
        else:
            self.virtual_state = None
        

        sk_mg = SkillManager(skill_mask=self.skill_list)
        self.sk_map = sk_mg.skill_map()
        
        # Battery recording 
        
        self.battery_history = []
        if self.virtual_mode:
            # VIRTUAL MODE
            self.signal_pub_timer = self.create_timer(5.,self.virtual_battery_callback)
        else:
            self.subscription = self.create_subscription(
                BatteryState,
                '/battery_state',
                self.battery_callback,
                10
            )

        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        self.app = Flask(__name__)
        self.setup_routes()
        threading.Thread(target=self.run_flask, daemon=True).start()
        self.signal_pub_timer = self.create_timer(self.settings['heartbeat_period'],self._send_heartbeat)
        self.busy = False



    def setup_routes(self):
        # @self.app.route('/ping', methods=['GET'])
        # def ping():
        #     return jsonify({"status": "alive"})
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
            self.exec(data)
            return jsonify({"status": "received"})
        
    def run_flask(self):
        self.app.run(host="0.0.0.0", port=self.exec_port, debug=False, use_reloader=False)

    # def check_signals(self, msg):
    #     self.flags = json.loads(msg.data)
    #     if "_SHUTDOWN_" in self.flags:
    #         self.destroy_node()

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
    # rclpy.spin(bot_exec)
    bot_exec.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
