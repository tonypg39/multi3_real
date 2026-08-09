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
# MODE = os.getenv("MODE", "")

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__("multi3_coordinator")
        self.coord_settings = {
            "signal_states_period": 1.0,
            "assignment_period": 1.5
        }
        self.shutdown_count = -1
        self.idle_robots = {}
        self.robot_states = {}
        self.fragments = {}
        self.signal_states = ['SYSTEM_START']
        self.first_assignment = True
        
        # Declare Parameters

        # Updates for multi-mission experimentation
        self.declare_parameter("test_id", "")            
        test_id = self.get_parameter("test_id").value
        if test_id == "":
            if TEST_ID == "":
                raise ValueError("Test ID was not received in the Coordinator Node")
            test_id = TEST_ID
            # self.get_logger().fatal("No test_id specified!!")
            # return

        self.get_logger().info(f"Received test_id parameter: {test_id}")
        self.multi_mission = test_id.startswith("test-multi")

        if self.multi_mission:
            self.get_logger().info("Testing multi-mission mode.")
            self.test_folder = "multi_mission_tests"
        else:
            self.get_logger().info("Testing single-mission mode.")
            self.test_folder = "tests"

        self.test_id = test_id
        self.robot_count = int(self.test_id.split("_")[1])
        self.mode = self.test_id.split("_")[-1]
        self.executors = self.get_active_executors()
        
        self.get_logger().info("$$**MISSION_START**$$")
        self.get_logger().info(f"Starting the Coordinator node with params ==> test_id = {test_id} || mode = {self.mode}")
        self.robot_inventory = self.read_inventory(test_id)
        self.start_ready = False
        fragments = self.read_fragments(test_id)

        if self.multi_mission:
            self.declare_parameter("mission_income_time", 30)
            self.mission_income_time = self.get_parameter("mission_income_time").value
            self.fragment_update_timer = self.create_timer(self.mission_income_time, self.add_new_mission)
        
        self.signal_pub_timer = self.create_timer(self.coord_settings['signal_states_period'],self.broadcast_signal_states)
        self.assignment_timer = self.create_timer(self.coord_settings['assignment_period'],self.assign)
        self.fragments = self.load_fragments(fragments)
        self.total_number_signals = self.get_total_of_signals()
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
    
    def get_active_executors(self):
        robot_count = int(self.test_id.split("_")[1])
        active_execs = {}
        for i in range(robot_count):
            active_execs["robot_"+str(i+1)] = f"http://localhost:{6000+i+1}"
        return active_execs 
    
    # Communication Methods
    def send_signal_states(self, message_data):
        # self.get_logger().info("Sending the signal states...")
        for robot,url in self.executors.items():
            try:
                resp = requests.post(f"{url}/get_signal_states", json=self.signal_states)
                results = resp.json()
            except Exception as e:
                results = {"error": str(e)}

    # This method reads the fragments from the dry_mission.json file and updates the self.fragments dictionaty by adding the new fragments to it. It also updates the self.total_number_signals variable to reflect the total number of signals in the updated fragments.
    def add_new_mission(self):
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        with open(f"{package_path}/multi3_tests/{self.test_folder}/{self.test_id}/additional_tasks.json") as f:
            new_frags = json.load(f)

        new_fragments = self.load_fragments(new_frags)
        # Mission incoming at maximum priority
        for fragment in new_fragments.values():
            fragment["priority"] = 999
        self.fragments.update(new_fragments)

        # Update the total number of signals
        self.total_number_signals = self.get_total_of_signals()
        self.get_logger().info(f"Updated fragments: {self.fragments}")
        self.get_logger().info(f"Total number of signals: {self.total_number_signals}")
        self.fragment_update_timer.cancel()  # Stop the timer after adding the new mission
    
    def send_fragment_to_robot(self, robot, fragment):
        self.get_logger().info(f"Sending Fragment {fragment['fragment_id']} to Robot {robot}")
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
            # self.get_logger().info(f"Received signal: {new_signal}")
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
    

    def read_fragments(self, test_id):
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        # print(package_path)
        with open(f"{package_path}/multi3_tests/{self.test_folder}/{test_id}/tasks.json") as f:
            frags = json.load(f)
        return frags

    def read_inventory(self, test_id):
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        # print(package_path)
        with open(f"{package_path}/multi3_tests/{self.test_folder}/{test_id}/inventory.json") as f:
            inventory = json.load(f)
        return inventory
    


    def load_fragments(self, fragments):
        c = len(self.fragments)
        F = {}
        for fr in fragments:
            fr_obj = {
                "status": "waiting",
                "fragment_id": f"fr_{c}",
                "age": 0,
                "priority": 0
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
        
    def send_finish_signal(self):
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        self.get_logger().info(f"The package path is: {package_path}")
        path = f"{package_path}/multi3_tests/results/"
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.test_id}_finished.json"
        self.get_logger().info(f"Writing finished signal in {filename}...")
        # Export the json to the path
        with open(path + filename, "w") as f:
            json.dump({"test":"success"},f)
            
    
    # Publish the signal_states periodically using a Timer object
    def broadcast_signal_states(self):
        # message = String()
        signal_list = self.signal_states
        if self.shutdown_count > -1:
            if self.shutdown_count == 0:
                self.get_logger().info("$$*MISSION_STOPPED*$$")
                self.send_finish_signal()
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
        eligible = True        
        for t in fragment["tasks"]:
            if t["id"].find("|") > -1: # If it is either a signal or a wait 
                continue
            if self.get_core_task(t["id"]) not in self.robot_inventory[robot]:
                eligible = False
        # self.get_logger().info(f"Checking elegibility for {robot} and tasks {fragment['tasks']} = {able}")
        return eligible

    def get_total_of_signals(self):
        if self.mode == "multi3":
            return len(self.fragments)
        cnt = 0
        for f in self.fragments.values():
            for t in f["tasks"]:
                if t["id"].find("send_signal") >-1:
                    cnt+=1
        return cnt

    def get_core_task(self, task):
        sep = task.find("^")
        if sep > -1:
            return task[:sep]
        return task

    def get_fragment_pi(self, fragment):
        # Given a fragment, this method returns a priority, computed as priority + 0.01 age
        return fragment["priority"] + 0.01 * fragment["age"]

    def generate_assigments(self, idle_robots, executable_fragments):
        assignment_dict = {}
        # sorted_frags = sorted(fragments, key= lambda x: x["priority"], reverse=True)
        sorted_frags = sorted(executable_fragments, key= lambda x: self.get_fragment_pi(x), reverse=True)
        new_assigned_robots = set()

        if self.mode != "multi3": # Baseline assignment
            assigned_fragments = set()

            # If in the baseline there is a fragment in self.fragments that is waiting, and has "robot" = robot, then the robot is not eligible to pick up any other fragment
            if self.first_assignment:
                # Check if in the sorted fragments there are fragments without the "robot" key, and if so, assign them to the first available robot
                for f in sorted_frags:
                    if "robot" not in f.keys():
                        for r in idle_robots:
                            if r not in new_assigned_robots and self.check_eligibility(r,f):
                                self.fragments[f["fragment_id"]]["status"] = "executed"
                                assignment_dict[r] = f.copy()
                                new_assigned_robots.add(r)
                                assigned_fragments.add(f["fragment_id"])
                                break
            for f in sorted_frags:
                if "robot" in f.keys():
                    # STATIC ASSIGNMENT, we expect the keyword robot associated with each fragment
                    # The fragments are a construct with all of the tasks assigned to a specific robot
                    if f["robot"] in idle_robots and f["robot"] not in new_assigned_robots and f["fragment_id"] not in assigned_fragments and self.check_eligibility(f["robot"],f):
                            self.fragments[f["fragment_id"]]["status"] = "executed"
                            assignment_dict[f["robot"]] = f.copy()
                            new_assigned_robots.add(f["robot"])
                            assigned_fragments.add(f["fragment_id"])

            # If there are fragments waiting, while eligible robots do not have further fragments waiting to be assigned (i.e., they do not appear in the robot field), then assign them. This happens when a robot has finisched its workload, but there are still fragments waiting to be assigned. In this case, we can assign the waiting fragments to the eligible robots that have already finished their workload.
            for r in idle_robots:
                if r not in new_assigned_robots:
                    finished_workload = True
                    for f in self.fragments.values():
                        # If there's a fragment that is waiting for a specific robot
                        if f["status"] == "waiting" and "robot" in f.keys() and f["robot"] == r:
                            finished_workload = False
                    if finished_workload:
                        self.get_logger().info(f"Robot {r} has finished its workload, checking for waiting fragments from further missions")
                        for f in sorted_frags:
                            if f["fragment_id"] not in assigned_fragments and self.check_eligibility(r,f):
                                self.get_logger().info(f"Assigning fragment {f['fragment_id']} to robot {r}")
                                self.fragments[f["fragment_id"]]["status"] = "executed"
                                assignment_dict[r] = f.copy()
                                new_assigned_robots.add(r)
                                assigned_fragments.add(f["fragment_id"])
                                break

        else:
            for f in sorted_frags:
                # self.get_logger().warning(f"Inside sorted frags {robots}")
                # self.get_logger().info(f"Checking fragment {f['fragment_id']} with priority {self.get_fragment_pi(f)}")
                    for r in idle_robots:
                        if r in new_assigned_robots:
                            # self.get_logger().info(f"Robot {r} is in use (skip)")
                            continue
                        else:
                            # self.get_logger().info(f"Robot {r} is not in use, checking if eligible for the fragment")
                            if self.check_eligibility(r,f):
                                # self.get_logger().info("Is eligible!")
                                self.fragments[f["fragment_id"]]["status"] = "executed"
                                assignment_dict[r] = f.copy()
                                new_assigned_robots.add(r)
                                break
                            # else:
                                # self.get_logger().info("Is not eligible!")
        self.first_assignment = False
        return assignment_dict
        
    def get_idle_robots(self):
        # get available services 
        ir = []
        for k,v in self.idle_robots.items():
            if v:
                ir.append(k)
        return ir
    
    def get_executable_fragments(self):
        active_frags = []
        for frag in self.fragments.values():
            
            if frag["status"] == "waiting":
                w_flags = frag['initial_wait'].split('&')
                missing_signal = any(fl not in self.signal_states for fl in w_flags)
                if not missing_signal:
                    active_frags.append(frag)
                # else:
                #     self.get_logger().info(f"The fragment {frag['fragment_id']} has the missing {missing_signal}")
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
            # self.collect_battery_info()
            self.shutdown_count = 8
            # self.destroy_node()
        self.get_logger().info("------Assignment window------")

        # Timestamp for the start of this assignment window
        assignment_start_time = time.perf_counter()

        # self.get_logger().info(f"Mission signals so far:{self.signal_states} ")
        progress = round(len(self.signal_states)*100/self.total_number_signals+1,2)
        self.get_logger().info(f"Progress: {progress}")
        idle_robots = self.get_idle_robots()
        executable_fragments = self.get_executable_fragments()

        # self.get_logger().info(f"Fragments: {self.fragments}")
        # self.get_logger().info(f"Active fragments: {fragments}")
        
        # self.log_fragments(fragments, "Active fragment")
        # self.log_futures()
        self.log_robots()
        # self.get_logger().info(f"The active robots are: {robots}")
        assignments = self.generate_assigments(idle_robots, executable_fragments)
        # self.get_logger().info(f"Assigments: {assignments}")

        if len(assignments) > 0:
            for robot, fragment in assignments.items():
                self.executor_pool.submit(self.send_fragment_to_robot,robot, fragment)
            
            # with ThreadPoolExecutor(max_workers=len(assignments)) as executor:
            #     futures = []
            #     for robot, fragment in assignments.items():
            #         futures.append(executor.submit(self.send_fragment_to_robot,robot, fragment))
            
        # Increase the age of the unpicked fragments
        for f in executable_fragments:
            if self.fragments[f["fragment_id"]]["status"] == "waiting":
                self.fragments[f["fragment_id"]]["age"] += 1

        assignment_end_time = time.perf_counter()
        assignment_time = (assignment_end_time - assignment_start_time) * 1000  # Convert to milliseconds
        if len(assignments) > 0:
            self.get_logger().info(f"Fragments assigned. Time: {assignment_time:.3f} milliseconds")
        else:
            self.get_logger().info(f"No fragments assigned. Time: {assignment_time:.3f} milliseconds")


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