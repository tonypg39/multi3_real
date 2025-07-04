# Coordinator Tasks
# - Listen to /mission_comms and update the flag_table
# assign the available fragment to idle robots
import json
import logging
from datetime import datetime
import os
from flask import Flask, request, jsonify
import threading
import rclpy
import requests
from std_msgs.msg import String
from rclpy.node import Node
from concurrent.futures import ThreadPoolExecutor
import time
from ament_index_python import get_package_prefix
import sys

COORDINATOR_URL = os.getenv("COORDINATOR_URL", "http://localhost:5000")
TEST_ID = os.getenv("TEST_ID", "")
MODE = os.getenv("MODE", "")

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__("multi3_coordinator")
        executors = {
            "robot_1": "http://localhost:6001",
            "robot_2": "http://localhost:6002",
            "robot_3": "http://localhost:6003",
        }
        self.coord_settings = {
            "signal_states_period": 1.0,
            "assignment_period": 1.5
        }
        self.shutdown_count = -1
        self.idle_robots = {}
        self.robot_states = {}
        self.signal_states = ['SYSTEM_START']
        
        # self.wait_for_start_trigger()

        # Declare Parameters
        self.declare_parameter("test_id", "")
        self.declare_parameter("mode", "")

        test_id = self.get_parameter("test_id").value
        mode = self.get_parameter("mode").value
        
        
        if test_id == "":
            if TEST_ID == "":
                raise ValueError("Test ID was not received in the Coordinator Node")
            test_id = TEST_ID
            # self.get_logger().fatal("No test_id specified!!")
            # return
        
        if mode == "":
            if TEST_ID == "":
                raise ValueError("MODE (multi3 |bl_0 |...) was not received in the Coordinator Node")
            mode = MODE
        
        self.get_logger().info("$$**MISSION_START**$$")
        self.get_logger().info(f"Starting the Coordinator node with params ==> test_id = {test_id} || mode = {mode}")
        self.test_id = test_id
        self.robot_count = int(self.test_id.split("_")[1])
        self.mode = mode
        self.executors = self.get_active_executors(executors)
        self.robot_inventory = self.read_inventory(test_id)
        self.start_ready = False
        fragments = self.read_fragments(test_id, mode)
        
        self.signal_pub_timer = self.create_timer(self.coord_settings['signal_states_period'],self.broadcast_signal_states)
        self.assignment_timer = self.create_timer(self.coord_settings['assignment_period'],self.assign)
        self.fragments = self.load_fragments(fragments)
        self.executor_pool = ThreadPoolExecutor(max_workers=8)
        # self.get_logger().info(f"The loaded fragments: {self.fragments}")
        # Dict to keep track of the states of executed fragments 
        # (For a frag to have a key here, the fragment must've been sent )
        self.fragments_futures = {} 
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        self.app = Flask(__name__)
        self.setup_routes()
        threading.Thread(target=self.run_flask, daemon=True).start()
    
    def get_active_executors(self, execs):
        robot_count = int(self.test_id.split("_")[1])
        d = list(execs.items())
        active_execs = {}
        for i in range(robot_count):
            active_execs[d[i][0]] = d[i][1]
        return active_execs 


    def wait_for_start_trigger(self):
        flag_path = "/tmp/start.flag"
        self.get_logger().info("Waiting for the trigger...")
        rate = self.create_rate(1)
        while os.path.exists(flag_path) and False:
            rate.sleep()
        
        self.get_logger().info("Trigger Flag detected")
    
    # Communication Methods
    def send_signal_states(self, message_data):
        # self.get_logger().info("Sending the signal states...")
        for robot,url in self.executors.items():
            try:
                resp = requests.post(f"{url}/get_signal_states", json=self.signal_states)
                results = resp.json()
            except Exception as e:
                results = {"error": str(e)}

    
    def send_fragment_to_robot(self, robot, fragment):
        self.get_logger().info(f"Sending Fragment to Robot {robot}")
        url = self.executors[robot]
        # fragment = json.dumps(fragment)

        try:
            resp = requests.post(f"{url}/get_fragment", json=fragment)
            results = resp.json()
        except Exception as e:
            results = {"error": str(e)}
            self.get_logger().error(str(e))
        # self.get_logger.info(results)
    
    def setup_routes(self):
        @self.app.route('/mission_signal', methods=['POST'])
        def receive_mission_signals():
            data = request.get_json()
            new_signal = data["signal"]
            if new_signal not in self.signal_states:
                self.signal_states.append(new_signal)
            # self.get_logger().info(f"Received HTTP data: {data}")
            return jsonify({"status": "received"})

        @self.app.route('/heartbeat', methods=['POST'])
        def heartbeat():
            data = request.get_json()
            # self.get_logger().info(f"Received HTTP data: {data}")
            self.update_hb(data)
            return jsonify({"status": "received"})
        
        @self.app.route('/execution_response', methods=['POST'])
        def execution_response():
            data = request.get_json()["response"]
            # self.get_logger().info(f"Received HTTP data: {data}")
            self.fragments_futures[data["fragment_id"]] = data["execution_code"]
            return jsonify({"status": "received"})


    def run_flask(self):
        self.app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
    

    def read_fragments(self, test_id, mode):
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        # print(package_path)
        with open(f"{package_path}/multi3_tests/tests/{test_id}/tasks_{mode}.json") as f:
            frags = json.load(f)
        return frags

    def read_inventory(self, test_id):
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        # print(package_path)
        with open(f"{package_path}/multi3_tests/tests/{test_id}/inventory.json") as f:
            inventory = json.load(f)
        return inventory
    


    def load_fragments(self, fragments):
        c = 0
        F = {}
        for fr in fragments:
            fr_obj = {
                "status": "waiting",
                "fragment_id": f"fr_{c}",
                "age": 0,
            }
            fr_obj.update(fr)
            F[fr_obj["fragment_id"]] = fr_obj
            c += 1
        return F

    def update_hb(self, msg):
        st = msg["data"].split("=")
        if not st[0] in self.robot_inventory:
            self.get_logger().warning(f"Ignoring robot '{st[0]}' as it is not in inventory")
            return 
        if st[1] == "idle":
            self.idle_robots[st[0]] = True
        else:
            self.idle_robots[st[0]] = False
        
        self.robot_states[st[0]] = st[1]
        if not self.start_ready:
            inactive_robot = False
            for robot in self.robot_inventory.keys():
                if robot not in self.robot_states:
                    inactive_robot = True
            if not inactive_robot:
                self.get_logger().info("STARTING the mission since all robots are active...")
                self.start_ready = True
        

    
    # Publish the signal_states periodically using a Timer object
    def broadcast_signal_states(self):
        # message = String()
        signal_list = self.signal_states
        if self.shutdown_count > -1:
            if self.shutdown_count == 0:
                self.get_logger().info("$$*MISSION_STOPPED*$$")
                self.assignment_timer.cancel()
                self.signal_pub_timer.cancel()
                self.shutdown_count = 5
                self.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
            signal_list.append("_SHUTDOWN_")
            self.shutdown_count -= 1


        message_data = json.dumps(signal_list)
        self.send_signal_states(message_data)
        # self.get_logger().info(f"Sending signals info: {message.data}")
        # self.signal_publisher.publish(message)
    
    def check_active_fragments(self):
        active_frags = []
        for frag in self.fragments:
            if frag['status'] == "waiting":
                active_frags.append(frag)
            if frag['status'] == "blocked":
                w_flags = frag["initial_wait"].split('&')
                missing_flag = False
                for f in w_flags:
                    if f not in self.signal_states:
                        missing_flag = True
                if not missing_flag:
                    self.fragments[frag["fragment_id"]]["status"] = "waiting"
                    active_frags.append(frag)
        return active_frags
    
    def check_eligibility(self, robot,fragment):
        able = True        
        for t in fragment["tasks"]:
            if t["id"].find("|") > -1: # If it is either a signal or a wait 
                continue
            if self.get_core_task(t["id"]) not in self.robot_inventory[robot]:
                able = False
        self.get_logger().info(f"Checking elegibility for {robot} and tasks {fragment['tasks']} = {able}")
        return able
    
    def get_core_task(self, task):
        sep = task.find("^")
        if sep > -1:
            return task[:sep]
        return task
    
    def generate_assigments(self, robots, fragments):
        assignment_dict = {}
        sorted_frags = sorted(fragments, key= lambda x: x["age"], reverse=True)
        used_robots = set()
        for f in sorted_frags:
            # self.get_logger().warning(f"Inside sorted frags {robots}")
            if "robot" in f.keys():
                # STATIC ASSIGNMENT, we expect the keyword robot associated with each fragment
                # The fragments are a construct with all of the tasks assigned to a specific robot
                if f["robot"] in robots:
                    self.fragments[f["fragment_id"]]["status"] = "executed"
                    assignment_dict[f["robot"]] = f.copy()
                continue
            for r in robots:
                if r in used_robots:
                    continue
                if self.check_eligibility(r,f):
                    self.fragments[f["fragment_id"]]["status"] = "executed"
                    assignment_dict[r] = f.copy()
                    used_robots.add(r)
                    break
        return assignment_dict
            

        
    def get_idle_robots(self):
        # get available services 
        ir = []
        for k,v in self.idle_robots.items():
            if v:
                ir.append(k)
        return ir
    
    def get_active_fragments(self):
        active_frags = []
        for frag in self.fragments.values():
            
            if frag["status"] == "waiting":
                w_flags = frag['initial_wait'].split('&')
                missing_signal = any(fl not in self.signal_states for fl in w_flags)
                # self.get_logger().info(f"The fragment {frag['fragment_id']} has the missing {missing_signal}")
                if not missing_signal:
                    active_frags.append(frag)
        return active_frags
    
        
    def log_fragments(self, fragments, label):
        for f in fragments:
            tasks = []
            for t in f["tasks"]:
                tasks.append(t["id"])
            # print(tasks)
            self.get_logger().info(f'{label} [{f["fragment_id"]}] ==> [{",".join(tasks)}]')

    def log_futures(self):
        st = "Futures: "
        for k,v in self.fragments_futures.items():
            st += f"Fragment: {k} | State: {v.done()}||"
        self.get_logger().info(st)

    def log_robots(self):
        s = "$$**ROBOTS_STATE**$$|| "
        for k,v in self.robot_states.items():
            s += f"{k} => {v} || "
        self.get_logger().info(s)

    def check_finished(self):
        # return False
        for f_id,frag in self.fragments.items():
            executed = frag["status"] == "executed"
            finished = False
            if executed and f_id in self.fragments_futures:
                finished = True
            # self.get_logger().info(f"\n{f_id} -> executed = {str(executed)} | finished = {str(finished)}")
            if not executed or not finished:
                # self.get_logger().info(f"{f_id} -> executed = {str(executed)} | finished = {str(finished)}")
                return False
        return True
        
    def collect_battery_info(self):
        battery_evol = {}
        for _, url in self.executors.items():
            resp = requests.get(url + "/get_battery_evolution")
            evolution = resp.json()
            battery_evol[evolution["robot_id"]] = evolution
        
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        self.get_logger().info(f"The package path is: {package_path}")
        path = f"{package_path}/multi3_tests/results/"
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{ts}_{self.test_id}_{self.mode}.json"
        self.get_logger().info(f"Publishing battery results to {filename}...")
        # Export the json to the path
        with open(path + filename, "w") as f:
            json.dump(battery_evol,f)
            
            


    def assign(self):
        if self.shutdown_count > -1:
            return
        if not self.start_ready:
            self.get_logger().info("Waiting for all robots to be active...")
            return
        if self.check_finished():
            self.get_logger().info("$$*MISSION_COMPLETED*$$")
            self.collect_battery_info()
            self.shutdown_count = 8
            # self.destroy_node()
        self.get_logger().info("------Assignment window------")
        self.get_logger().info(f"Mission signals so far:{self.signal_states} ")
        robots = self.get_idle_robots()
        fragments = self.get_active_fragments()
        
        self.log_fragments(fragments, "Active fragment")
        # self.log_futures()
        self.log_robots()
        # self.get_logger().info(f"The active robots are: {robots}")
        assignments = self.generate_assigments(robots, fragments)
        # self.get_logger().info(f"Assigments: {assignments}")


        if len(assignments) > 0:
            for robot, fragment in assignments.items():
                self.executor_pool.submit(self.send_fragment_to_robot,robot, fragment)
            
            # with ThreadPoolExecutor(max_workers=len(assignments)) as executor:
            #     futures = []
            #     for robot, fragment in assignments.items():
            #         futures.append(executor.submit(self.send_fragment_to_robot,robot, fragment))
            
        # Increase the age of the unpicked fragments
        for f in fragments:
            if self.fragments[f["fragment_id"]]["status"] == "waiting":
                self.fragments[f["fragment_id"]]["age"] += 1
        


# def read_fragments():
#     package_path = get_package_prefix("multi3_coordinator").replace("install","src")
#     # print(package_path)
#     with open(f"{package_path}/multi3_coordinator/tasks.json") as f:
#         frags = json.load(f)
#     return frags

def main(args=None):
    rclpy.init(args=args)
    # fragments = read_fragments()
    coord = CoordinatorNode()
    rclpy.spin(coord)
    coord.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()