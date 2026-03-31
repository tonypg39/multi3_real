import pandas as pd
import os
import json
from math import isclose

def compute_mission_times(lines, markers):
    # detect start time
    start_time = None
    end_time = None
    for l in lines:
        if l.find(markers["start_label"]) > -1:
            start_time = float(l.split()[3][1:-1])
        if l.find(markers["end_label"]) > -1:
            end_time = float(l.split()[3][1:-1])
    
    mission_times = {
        "start_time": start_time,
        "end_time": end_time,
        "completed_time": end_time - start_time
    }
    return mission_times


def compute_idle_times(lines, markers, mission_times, robot_count):
    idle_evolution = {}
    idle_times = {}
    for i in range(robot_count):
        robot_name = f"robot_{i+1}"
        idle_evolution[robot_name] = [(True, mission_times["start_time"], "")]
    
    for l in lines:
        if l.find(markers["status_label"]) > -1:
            sections = l.split("||")
            if sections[1] == " \n":
                continue
            for sec in sections[1:-1]:
                ws = sec.split()
                is_idle = (ws[2] == "idle") or (len(ws) == 4 and ws[3].find("wait_until") > -1)
                if is_idle:
                    task = ""
                else:
                    task = ws[3] if len(ws) > 3 else ""
                idle_evolution[ws[0]].append((is_idle, float(l.split()[3][1:-1]), task))
    
    for i in range(robot_count):
        robot_name = f"robot_{i+1}"
        idle_evolution[robot_name].append((True, mission_times["end_time"], ""))
    
    for robot, idle_evol in idle_evolution.items():
        idle_time_total = 0.0
        for i in range(1,len(idle_evol)):
            if idle_evol[i-1][0] and idle_evol[i][0]:
                idle_time_total += (idle_evol[i][1] - idle_evol[i-1][1])
        idle_times[robot] = idle_time_total
    return idle_times, idle_evolution

def process_metrics(metrics):
    data = {
        "test_id": [],
        "robot_count": [],
        "mission_size": [],
        "inventory_id": [],
        "scenario": [],
        "mode": [],
        "makespan": [],
        "robot_id":[],
        "robot_idle_time": []
    }

    for k,v in metrics.items():
        name = k[:k.find(".")]
        labels = name.split("_")
        test_id = name
        robot_count = int(labels[1])
        mission_size = int(labels[2])
        inv_id = int(labels[5][1])
        scenario = labels[4]
        mode = labels[6]

        makespan = v["mission_times"]["completed_time"]

        idle_times = v["idle_times"]
        for robot_id, idle_time in idle_times.items():
            data["test_id"].append(test_id)
            data["robot_count"].append(robot_count)
            data["mission_size"].append(mission_size)
            data["inventory_id"].append(inv_id)
            data["scenario"].append(scenario)
            data["mode"].append(mode)
            data["makespan"].append(makespan)
            data["robot_id"].append(robot_id)
            data["robot_idle_time"].append(idle_time)
    return data

def build_robot_timeline(samples, mission_start, mission_end):
    if not samples:
        return {
            "non_idle_intervals": [],
            "idle_intervals": [],
            "computed_idle_time": 0.0
        }

    rel_samples = [(bool(state), ts - mission_start, task) for state, ts, task in samples]
    mission_end_rel = mission_end - mission_start

    n = len(rel_samples)

    stable_idle = [False] * n
    for i in range(n-1):
        if rel_samples[i][0] and rel_samples[i+1][0]:
            stable_idle[i] = True

    elementary = []

    # intevals between consecutive samples
    for i in range(n - 1):
        t0 = rel_samples[i][1]
        t1 = rel_samples[i + 1][1]
        state = "idle" if stable_idle[i] else "non_idle"
        task = rel_samples[i][2]
        elementary.append((t0, t1, state, task))

    # last interval
    t_last = rel_samples[-1][1]
    if t_last < mission_end_rel:
        state = "idle" if stable_idle[-1] else "non_idle"
        task = rel_samples[-1][2]
        elementary.append((t_last, mission_end_rel, state, task))

    # merging consecutive intervals with the same state (idle or non-idle)
    merged = []
    for t0, t1, state, task in elementary:
        if not merged:
            merged.append([t0, t1, state, task])
            # print(f"Appending to the merged (initial): {task}")
        else:
            prev_t0, prev_t1, prev_state, prev_task = merged[-1]
            if prev_state == state and isclose(prev_t1, t0, abs_tol=1e-9) and prev_task == task:
                merged[-1][1] = t1
                # print(f"Merging with previous: {prev_task}")
            else:
                merged.append([t0, t1, state, task])
                # print(f"Appending to the merged: {task}")

    idle_intervals = []
    non_idle_intervals = []

    for t0, t1, state, task in merged:
        duration = t1 - t0
        if state == "idle":
            idle_intervals.append((t0, duration, task))
        else:
            non_idle_intervals.append((t0, duration, task))

    computed_idle_time = sum(duration for _, duration, _ in idle_intervals)

    return {
        "non_idle_intervals": non_idle_intervals,
        "idle_intervals": idle_intervals,
        "computed_idle_time": computed_idle_time
    }


def process_robot_idle_times(global_results, tolerance=1e-6):
    result = {}

    for mission_id, mission_data in global_results.items():
        test_name = mission_id[:mission_id.find(".")]
        mission_start = mission_data["mission_times"]["start_time"]
        mission_end = mission_data["mission_times"]["end_time"]
        mission_duration = mission_data["mission_times"]["completed_time"]

        result[test_name] = {
            "mission_duration": mission_duration,
            "robots": {}
        }

        idle_evolution = mission_data["idle_evolution"]
        idle_times_json = mission_data["idle_times"]

        for robot_name, samples in idle_evolution.items():
            robot_intervals = build_robot_timeline(samples, mission_start, mission_end)

            measured_idle_time = idle_times_json.get(robot_name, None)
            computed_idle_time = robot_intervals["computed_idle_time"]

            match = (
                measured_idle_time is not None and
                isclose(computed_idle_time, measured_idle_time, abs_tol=tolerance)
            )

            result[test_name]["robots"][robot_name] = {
                "non_idle_intervals": robot_intervals["non_idle_intervals"],
                "idle_intervals": robot_intervals["idle_intervals"],
                "computed_idle_time": computed_idle_time,
                "measured_idle_time": measured_idle_time,
                "idle_time_match": match,
                "idle_time_diff": None if measured_idle_time is None else computed_idle_time - measured_idle_time
            }

    return result

def main():
    markers = {
        "start_label": "$$**MISSION_START**$$",
        "end_label": "MISSION_COMPLETED",
        "status_label": "ROBOTS_STATE",
    }
    
    results_path = "../output/test_logs/"

    global_results = {}
    for filename in os.listdir(results_path):
        file_path = os.path.join(results_path, filename)
        if os.path.isfile(file_path):  # Check if it's a file
            print(f"Processing File: {filename}")
            result_file = filename
            with open(results_path + result_file, "r") as f:
                log_lines = f.readlines()
            
            robot_count = int(result_file.split("_")[1])
            mission_times = compute_mission_times(log_lines,markers)
            idle_times, idle_evolution = compute_idle_times(log_lines,markers,mission_times,robot_count)
            global_results[filename] = {
                "mission_times": mission_times,
                "idle_times": idle_times,
                "idle_evolution": idle_evolution
            }

    mission_data = process_metrics(global_results)
    df = pd.DataFrame(mission_data)
    df.to_csv("../output/mission_data.csv",index=False)

    idle_data = process_robot_idle_times(global_results)
    json.dump(idle_data, open("../output/robot_data.json","w"))

if __name__ == "__main__":
    main()