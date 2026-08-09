import re
import json
from collections import defaultdict

task_precedences = { # List of tasks that must be completed before executing each task
    "soil_moisture_analysis": ["drone_scan"],
    "data_analysis": ["soil_moisture_analysis"],
    "fertilizer_application": ["data_analysis"],
    "pest_control_spray": ["fertilizer_application"],
    "irrigation_adjustment": ["pest_control_spray"],
    "drone_recheck": ["irrigation_adjustment"],
    "soil_nutrient_test": ["irrigation_adjustment"],
    "mop": ["vacuum"],
    "polish": ["mop"],
    "pick_parcel": ["locate_parcel"],
    "scan_barcode": ["pick_parcel"],
    "categorize_by_destination": ["scan_barcode"],
    "check_weight_and_size": ["scan_barcode"],
    "label_parcel": ["check_weight_and_size"],
    "move_to_loading_zone": ["label_parcel"],
    "verify_manifest": ["move_to_loading_zone"],
    "thermal_scan": ["scan_perimeter"],
    "record_video": ["thermal_scan"],
    "analyze_surveillance_data": ["record_video"],
    "alert_security": ["analyze_surveillance_data"],
    "follow_movement": ["analyze_surveillance_data"],
    "capture_image": ["follow_movement"],
}

def parse_task_name(task_name):
    task_name_regex = re.compile(r"^(?P<base>.+?)(?:\^(?P<idx>\d+))?$")
    match = task_name_regex.match(task_name)
    if not match:
        return task_name, None

    base = match.group("base")
    idx = match.group("idx")
    multi_index = int(idx) if idx is not None else None
    return base, multi_index


def flatten_mission_tasks(test_data):
    flat_tasks = []
    robots = test_data.get("robots", {})
    for _, robot_data in robots.items():
        intervals = robot_data.get("non_idle_intervals", [])
        for interval in intervals:
            if interval[2] != "":
                start, duration, task_name = interval
                base_name, multi_index = parse_task_name(task_name)
                current_task = {
                    "task": task_name,
                    "base_name": base_name,
                    "multi_index": multi_index,
                    "start": float(start),
                    "end": float(start) + float(duration),
                }
                flat_tasks.append(current_task)
    return flat_tasks


def extract_tasks_multi_info(flat_tasks):
    tasks_multi_info = {}
    for current_task in flat_tasks:
        if current_task["base_name"] not in tasks_multi_info:
            tasks_multi_info[current_task["base_name"]] = {
                "is_multi": current_task["multi_index"] is not None,
                "instances": []
            }
        tasks_multi_info[current_task["base_name"]]["instances"].append(current_task["task"])
    return tasks_multi_info


def check_task_precedence_constraints(current_task, flat_tasks, task_precedences, tasks_multi_info):

    current_task_name = current_task["task"]
    current_task_base = current_task["base_name"]
    current_task_multi_index = current_task["multi_index"]

    required_tasks = []

    ## Check here if there is a precedence constraint for the current task
    if current_task_base in task_precedences:
        required_precedence_bases = task_precedences[current_task_base]
        is_current_task_multi_instanced = tasks_multi_info[current_task_base]["is_multi"]

        # Check here: if the task is multi instanced
        if is_current_task_multi_instanced:
            for required_precedence_base in required_precedence_bases:
                if required_precedence_base in tasks_multi_info:
                    is_required_multi_instanced = tasks_multi_info[required_precedence_base]["is_multi"]

                    # If the required task is multi instanced as well:
                    if is_required_multi_instanced:
                        # look for the tasks with the same index
                        found_tasks = [flat_task for flat_task in flat_tasks if flat_task["base_name"] == required_precedence_base and flat_task["multi_index"] == current_task_multi_index]
                        required_tasks.extend(found_tasks)

                    # Otherwise (the required task is not multi instanced):
                    else:
                        # look for the task with only base name
                        found_tasks = [flat_task for flat_task in flat_tasks if flat_task["base_name"] == required_precedence_base]
                        required_tasks.extend(found_tasks)

        # If the task is not multi instanced
        else:
            # look for all the previous tasks with matching base names, no matter if multi instanced or not
            for required_precedence_base in required_precedence_bases:
                found_tasks = [flat_task for flat_task in flat_tasks if flat_task["base_name"] == required_precedence_base]
                required_tasks.extend(found_tasks)


    # Check now for all the required tasks:
    current_task_start = current_task["start"]
    violations = []
    for required_task in required_tasks:
        if required_task["end"] > current_task_start:
            violations.append({
                "task": current_task_name,
                "task_start": current_task_start,
                "required_completions": required_tasks
            })
    return violations
            

def check_precedence_constraints_for_single_test(test_data, task_precedences):

    flat_tasks = flatten_mission_tasks(test_data)
    tasks_multi_info = extract_tasks_multi_info(flat_tasks)

    num_checks = 0
    num_passes = 0
    num_failed = 0
    violations = []

    for current_task in flat_tasks:
        if current_task["base_name"] in task_precedences:
            num_checks += 1
            task_violations = check_task_precedence_constraints(current_task, flat_tasks, task_precedences, tasks_multi_info)
            if len(task_violations) > 0:
                violations.extend(task_violations)
                num_failed += 1
            else:
                num_passes += 1

    return {
        "this_test_checks": num_checks,
        "test_ok": num_failed == 0,
        "checks_passed": num_passes,
        "checks_failed": num_failed,
        "violation_list": violations
    }


def check_precedence_constraints_for_all_tests(all_tests_data, task_precedences):
    results = {
        "test_missions": 0,
        "successes": 0,
        "failures": 0,
        "mission_details": {}
    }

    for test_id in all_tests_data.keys():
        test_data = all_tests_data[test_id]

        test_results = check_precedence_constraints_for_single_test(test_data, task_precedences)

        results["test_missions"] += 1
        if test_results["test_ok"]:
            results["successes"] += 1
        else:
            results["failures"] += 1    
        results["mission_details"][test_id] = test_results
    return results


def print_result_summary(results, config):
    print(f"--- Mission execution check report ({config}) ---")
    print(f"Missions checked: {results['test_missions']}")
    print(f"Successes: {results['successes']}")
    print(f"Failures: {results['failures']}")

    if results['failures'] > 0:
        print("\nDetail of failures per mission:")
        for test_id, test_result in results["mission_details"].items():
            if not test_result["test_ok"]:
                print(f"\n- Mission {test_id}: failed {test_result['checks_failed']} checks over {test_result['this_test_checks']} total checks")
                for violation in test_result["violation_list"]:
                    print(f"\t- Task {violation['task']} starts at {violation['task_start']} before completion of:")
                    for required_completion in violation["required_completions"]:
                        print(f"\t\t- {required_completion['task']} (ended at {required_completion['end']})")

    print("\n")

configurations = ["single-mission", "multi-mission/atstart-income", "multi-mission/online-income"]

for config in configurations:
    with open(f"../output/{config}/robot_data.json", "r") as f:
        data = json.load(f)

    results = check_precedence_constraints_for_all_tests(data, task_precedences)

    json.dump(results, open(f"../output/{config}/execution_check_results.json", "w"), indent=4)

    print_result_summary(results, config)
