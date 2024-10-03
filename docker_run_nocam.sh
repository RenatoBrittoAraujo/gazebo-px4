#!/bin/bash

source /opt/ros/humble/setup.bash

DOCKER_VOLUMES="$(pwd)/volumes"

if [ ! -d "$DOCKER_VOLUMES" ]; then
    # Create the folder if it doesn't exist
    mkdir -p "$DOCKER_VOLUMES"
fi

touch volumes/.env
echo "HOST_USERNAME=$(whoami)" > volumes/.env

sudo docker compose -f docker-configuration/docker-compose.yaml up -d ros-px4-humble

xhost local:docker

# Wait for the container to be ready
until sudo docker exec ros-px4-humble /bin/bash -c "echo 'Container is ready'" &> /dev/null; do
    echo "Waiting for the container to be ready..."
    sleep 2
done

# Start a new tmux session
tmux new-session -d -s my_session

tmux split-window -v

# Navigate to the first directory and execute the first command in the left pane
tmux select-pane -t 0
tmux send-keys "sudo docker exec -it ros-px4-humble /bin/bash -c \"cd PX4-Autopilot && sudo make px4_sitl gz_x500_depth\"" C-m
# Allow time for the container to open bash (optional sleep to ensure the container is ready)
# tmux send-keys "sleep 2" C-m
# tmux send-keys "cd PX4-Autopilot" C-m
# tmux send-keys "sudo make px4_sitl gz_x500_depth" C-m

tmux split-window -h

# Move to the right pane and execute the second command
tmux select-pane -t 1
tmux send-keys "sudo docker exec -it ros-px4-humble /bin/bash -c "source /opt/ros/humble/setup.bash; source ws/install/setup.bash"" C-m
# tmux send-keys "cd ws" C-m
# tmux send-keys "source install/setup.bash" C-m
# tmux send-keys "colcon build --packages-select fase_1" C-m
# tmux send-keys "ros2 run fase_1 fase1_script" C-m

# Split the right pane horizontally and resize it to create a minimized pane at the bottom

# Resize the new bottom pane to be as small as possible
tmux resize-pane -D 20  # Adjust the 20 to control the pane size (fewer lines = smaller pane)

# Move to the bottom pane and run the first command
tmux select-pane -t 2
# tmux send-keys "MicroXRCEAgent udp4 -p 8888" C-m

# Attach to the session
tmux attach-session -t my_session