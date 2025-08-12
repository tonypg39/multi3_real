import subprocess
import os
import yaml
import time

test_path = "./coordinator/multi3_coord_ws/src/multi3_tests/multi3_tests/tests/"
base_compose = {
    'services': {
        'coordinator': {
            'build': './coordinator',
            'environment': [
                'ROS_DOMAIN_ID=55',
                'MODE=bl_0'
            ],
            'volumes': [
                './output:/multi3_coord_ws/src/multi3_tests/multi3_tests/results'
            ],
            'container_name': 'coordinator',
            'network_mode': 'host'
        }
    }
}

def start_docker_compose(compose_file_path='docker-compose.yml'):
    try:
        subprocess.run(
            ["docker", "compose", "-f", compose_file_path, "up", "-d", "--build"],
            check=True
        )
        print("Docker Compose started successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to start Docker Compose: {e}")

def stop_docker_compose(compose_file_path='docker-compose.yml'):
    try:
        subprocess.run(
            ["docker", "compose", "-f", compose_file_path, "down"],
            check=True
        )
        print("Docker Compose stopped successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to stop Docker Compose: {e}")




def generate_executor(idx,test_id):
    return {
            'build': './executor',
            'container_name': f'executor{idx}',
            'environment': [
                'DISPLAY=${DISPLAY}',
                f'ROBOT_PORT={6000+idx}', 
                f'EXECUTOR_ID=executor{idx}',
                f'ROBOT_NAME=robot_{idx}',
                'VIRTUAL_MODE=1',
                'ROS_ENV=ros_humble',
                f'ROS_DOMAIN_ID={idx}',
                f'TEST_ID={test_id}',
                'COORDINATOR_URL=http://localhost:5000'
            ],
            'volumes': [
                '/tmp/.X11-unix:/tmp/.X11-unix'
            ],
            'depends_on': [
                'coordinator'
            ],
            'network_mode': 'host',
            'privileged': True
        }

def generate_docker_compose(test_id):
    compose = {
        'services': {
            'coordinator': {
                'build': './coordinator',
                'environment': [
                    'ROS_DOMAIN_ID=55',
                    'MODE=bl_0'
                ],
                'volumes': [
                    './output:/multi3_coord_ws/src/multi3_tests/multi3_tests/results'
                ],
                'container_name': 'coordinator',
                'network_mode': 'host'
            }
        }
    }
    compose["services"]["coordinator"]["environment"].append(f"TEST_ID={test_id}")
    robot_count = int(test_id.split("_")[1])
    print(f"Robot Count: {robot_count} | Mission size: {int(test_id.split('_')[2])}")
    for r in range(1, robot_count+1):
        compose["services"]["executor"+str(r)] = generate_executor(r,test_id)
    with open(f"docker-compose-{test_id}.yaml","w") as f:
        yaml.dump(compose,f,sort_keys=False)


def check_progress(test_id):
    cmd = f"docker compose -f docker-compose-{test_id}.yaml logs coordinator"
    coord_logs = os.popen(cmd).read().splitlines()
    progress = "Progress: Unread!"
    for l in coord_logs:
        if l.find("Progress") > -1:
            progress = l
    return progress



def main():
    files = os.listdir(test_path)
    for i,test_id in enumerate(files):
        print(f"\n\n***\Running the Test {test_id} || {i+1}/{len(files)}***")
        generate_docker_compose(test_id)
        start_docker_compose(f"docker-compose-{test_id}.yaml")
        time.sleep(2)
    
        while not os.path.exists(f"./output/{test_id}_finished.json"):
            print(check_progress(test_id))
            time.sleep(2)
        print("Detected finish of mission execution...\nPostprocessing...")
        cmd = f"docker compose -f docker-compose-{test_id}.yaml logs coordinator >> ./test_logs/{test_id}.log"
        os.popen(cmd)
        time.sleep(1)
        stop_docker_compose(f"docker-compose-{test_id}.yaml")
        
        

if __name__ == "__main__":
    main()
    # generate_docker_compose("test_5_10_t_cleaning_i0_bl1")