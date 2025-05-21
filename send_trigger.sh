#!/bin/bash

# List of container names
containers=("coordinator" "executor1" "executor2")

# Loop through each container and send the signal
for container in "${containers[@]}"; do
  echo "Sending start signal to $container"
  docker exec "$container" touch /tmp/start.flag
done
