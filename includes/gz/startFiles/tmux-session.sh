#!/bin/bash

echo Starting Tmux Session

# Name of the tmux session
SESSION_NAME="px4_sim"
# Decision 
Kill_Session_Decision="None"
Decision_Create_Session="None"

##############################################################
# Create Functions
Tmux_create_session_run_CMDs(){
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
    tmux send-keys -t $SESSION_NAME:0.3 "docker exec -it px4_sim /px4/volume/startFiles/start_nav2.sh" Enter
    tmux send-keys -t $SESSION_NAME:0.4 "docker exec -it px4_sim /px4/volume/startFiles/gz_start_rtabmap.sh " Enter
    tmux send-keys -t $SESSION_NAME:0.5 "docker exec -it px4_sim /px4/volume/startFiles/start_nav2_rviz.sh" Enter
    echo Session Created

    # Attach to the session
    tmux attach -t $SESSION_NAME
}

Tmux_kill_session(){
    echo Killing session
    tmux kill-session -t $SESSION_NAME
    echo $SESSION_NAME session killed 
}

Tmux_attach_session(){
    tmux attach-session -t $SESSION_NAME
}

###############################################################
# Creating Tmux Session and Run Commands
# Check if already is created 
output=$(tmux ls | grep px4_sim)
# Check if session is present
if [[ -n "$output" ]]; then
    echo Session Already Exsit 
    while true; do
        read -p "Kill Session First ? " Decision_Kill_Session
        case "$Decision_Kill_Session" in
            [Yy] | [Yy][Ee][Ss] | yess )
                # Kill session
                Tmux_kill_session 
                while true; do
                    read -p "Create session? " Decision_Create_Session
                    if [[ "$Decision_Create_Session" == "Y"   || "$Decision_Create_Session" == "y"   || \
                          "$Decision_Create_Session" == "Yes" || "$Decision_Create_Session" == "yes" || \
                          "$Decision_Create_Session" == "yess"   ]]; then
                        Tmux_create_session_run_CMDs
                        break
                    elif [[ "$Decision_Create_Session" == "N" || "$Decision_Create_Session" == "n" ]]; then
                        exit 0
                    else 
                        echo "Invalid input. Please try again."
                    fi
                done
                break
                ;;
            [Nn] | [Nn][Oo] )
                # Attach to session 
                Tmux_attach_session
                break
                ;;
            *)
                echo "Invalid input. Please try again."
                ;;
        esac
    done
else 
    Tmux_create_session_run_CMDs
fi