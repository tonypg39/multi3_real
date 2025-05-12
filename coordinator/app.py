from flask import Flask, request, jsonify
import requests
import logging

app = Flask(__name__)

# Static list of executor URLs (could be loaded from config or registration endpoint)
executors = {
    "executor1": "http://executor1:6000",
    "executor2": "http://executor2:6000"
}

@app.route("/broadcast", methods=["POST"])
def broadcast():
    message = request.json.get("message", "Default message")
    results = {}
    for name, url in executors.items():
        try:
            resp = requests.post(f"{url}/receive", json={"from": "coordinator", "message": message})
            results[name] = resp.json()
        except Exception as e:
            results[name] = {"error": str(e)}
    return jsonify(results), 200

@app.route("/info", methods=["GET"])
def info():
    return jsonify({"message": "Hello Executor!", "status": "success"})

@app.route("/data", methods=["POST"])
def receive_data():
    data = request.json
    print(f"Received data: {data}")
    return jsonify({"status": "received", "data": data}), 403

if __name__ == "__main__":
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    app.run(host="0.0.0.0", port=5000)
