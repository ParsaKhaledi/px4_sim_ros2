#!/bin/bash

# Name of the tmux session
SESSION_NAME="PX4_sim"

# Create a new tmux session (detached)
tmux new-session -d -s $SESSION_NAME

# Split the window into 6 panes
tmux split-window -h -t $SESSION_NAME       # Split the first pane horizontally
tmux split-window -v -t $SESSION_NAME:0.0   # Split the top-left pane vertically
tmux split-window -v -t $SESSION_NAME:0.1   # Numbers update each time terminal splits  
tmux split-window -v -t $SESSION_NAME:0.3 
tmux split-window -v -t $SESSION_NAME:0.4 
tmux select-layout -t $SESSION_NAME tiled   # Arrange panes in a tiled layout

# Run specific commands in each pane
## RUN
tmux send-keys -t $SESSION_NAME:0.0 "docker exec -it px4_sim /px4/volume/startFiles/gz_start_px4_gz_sim.sh"       Enter
tmux send-keys -t $SESSION_NAME:0.1 "docker exec -it px4_sim /px4/volume/startFiles/gz_start_ros2_gz_bridge.sh"   Enter
tmux send-keys -t $SESSION_NAME:0.2 "docker exec -it px4_sim /px4/volume/startFiles/gz_start_state_publisher.sh"  Enter
tmux send-keys -t $SESSION_NAME:0.3 "echo $CameraType && sleep inf" Enter
tmux send-keys -t $SESSION_NAME:0.4 "docker exec -it px4_sim /px4/volume/startFiles/gz_start_rtabmap_rgbd.sh " Enter
tmux send-keys -t $SESSION_NAME:0.5 "bash" Enter

# Attach to the session
tmux attach -t $SESSION_NAME