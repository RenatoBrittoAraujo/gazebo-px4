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

until sudo docker exec ros-px4-humble /bin/bash -c "echo 'Container is ready'" &> /dev/null; do
    echo "Waiting for the container to be ready..."
    sleep 2
done

tmux new-session -d -s my_session

tmux split-window -v
tmux split-window -v

tmux select-pane -t 0
tmux send-keys "sudo docker exec -it ros-px4-humble /bin/bash -c \"cd PX4-Autopilot && sudo make px4_sitl gz_x500_depth\"" C-m

tmux split-window -h

tmux select-pane -t 1
tmux send-keys "sudo docker exec -it ros-px4-humble /bin/bash -c \"cd $(pwd)/ws; source /opt/ros/humble/setup.bash; source install/setup.bash; colcon build --packages-select fase_3; ros2 run fase_3 fase3_script\"" C-m

tmux resize-pane -D 20  

tmux select-pane -t 2
tmux split-window -h

tmux select-pane -t 3
tmux send-keys "sudo docker exec -it ros-px4-humble /bin/bash -c \"MicroXRCEAgent udp4 -p 8888 \"" C-m

tmux select-pane -t 2
tmux send-keys "./QGroundControl.AppImage" C-m

tmux select-pane -t 4
tmux split-window -h
tmux select-pane -t 4
tmux send-keys "sudo docker exec -it ros-px4-humble /bin/bash -c \"cd $(pwd)/ws; source install/setup.bash; ros2 run ros_gz_image image_bridge /camera\"" C-m

tmux select-pane -t 5
tmux send-keys "sudo docker exec -it ros-px4-humble /bin/bash -c \"cd $(pwd)/ws/src/fase_3/fase_3; source /opt/ros/humble/setup.bash; source ../../../install/setup.bash; python3 get_image_feed.py\"" C-m

# Attach to the session
tmux attach-session -t my_session