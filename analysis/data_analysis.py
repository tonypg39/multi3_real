import pandas as pd
import json

def process_metrics(metrics):
    data = {
        "test_id": [],
        "robot_count": [],
        "mission_size": [],
        "inventory_id": [],
        "scenario": [],
        "mode": [],
        "completion_time": [],
        "average_idle_time": []
    } 

    for k,v in metrics.items():
        name = k[:k.find(".")]
        labels = name.split("_")
        # print(f"Labels: {labels}")
        test_id = name
        robot_count = int(labels[1])
        mission_size = int(labels[2])
        inv_id = int(labels[5][1])
        scenario = labels[4]
        mode = labels[6]#[:labels[6].find(".")]
        
        completion_time = v["mission_times"]["completed_time"]
        total_idle = 0
        for ri in v["idle_times"].values():
            total_idle += ri
        avg_idle = total_idle/len(v["idle_times"])

        data["test_id"].append(test_id)
        data["robot_count"].append(robot_count)
        data["mission_size"].append(mission_size)
        data["inventory_id"].append(inv_id)
        data["scenario"].append(scenario)
        data["mode"].append(mode)
        data["completion_time"].append(completion_time)
        data["average_idle_time"].append(avg_idle)
        # return
    
    df = pd.DataFrame(data)
    df.to_csv("data.csv",index=False)


if __name__ == "__main__":
    with open("metrics.json", "r") as f:
        metrics = json.load(f)
    process_metrics(metrics)
    
 