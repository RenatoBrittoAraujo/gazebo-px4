tmux attach-session -t my_session
#!/bin/bash

source /opt/ros/humble/setup.bash

# Start a new tmux session
tmux new-session -d -s my_session

tmux split-window -v
tmux split-window -v

# Navigate to the first directory and execute the first command in the left pane
tmux select-pane -t 0
tmux send-keys "cd PX4-Autopilot" C-m
tmux send-keys "make px4_sitl gz_x500_depth" C-m

tmux split-window -h

# Move to the right pane and execute the second command
tmux select-pane -t 1
tmux send-keys "cd ws" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "colcon build --packages-select fase_3; ros2 run fase_3 fase3_script" C-m

# Split the right pane horizontally and resize it to create a minimized pane at the bottom

# Resize the new bottom pane to be as small as possible
tmux resize-pane -D 20  # Adjust the 20 to control the pane size (fewer lines = smaller pane)

# Move to the bottom pane and run the first command
tmux select-pane -t 2
tmux split-window -h

tmux select-pane -t 3
tmux send-keys "MicroXRCEAgent udp4 -p 8888" C-m

tmux select-pane -t 2
tmux send-keys "./QGroundControl.AppImage" C-m

tmux select-pane -t 4
tmux send-keys "cd ws" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "ros2 run ros_gz_image image_bridge /camera" C-m

tmux select-pane -t 5
tmux send-keys "cd ws/src/fase_1/fase_1" C-m
tmux send-keys "python3 imagem.py" C-m

# Attach to the session
tmux attach-session -t my_session