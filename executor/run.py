from flask import Flask, request, jsonify
import os
import threading
import requests
import time

COORDINATOR_URL = os.getenv("COORDINATOR_URL", "http://coordinator:5000")
EXECUTOR_ID = os.getenv("EXECUTOR_ID", "executorX")
ROS_ENV = os.getenv("ROS_ENV", "ros_default")

app = Flask(__name__)

@app.route("/receive", methods=["POST"])
def receive():
    data = request.json
    print(f"{EXECUTOR_ID} received message: {data}")
    return jsonify({"status": "message received", "executor": EXECUTOR_ID}), 200

def background_task():
    while True:
        try:
            r = requests.get(f"{COORDINATOR_URL}/info")
            print(f"{EXECUTOR_ID} got response: {r.json()}")

            payload = {
                "executor_id": EXECUTOR_ID,
                "ros_env": ROS_ENV
            }
            requests.post(f"{COORDINATOR_URL}/data", json=payload)
        except Exception as e:
            print(f"{EXECUTOR_ID} error contacting coordinator:", e)

        time.sleep(5)

if __name__ == "__main__":
    threading.Thread(target=background_task, daemon=True).start()
    app.run(host="0.0.0.0", port=6000)
