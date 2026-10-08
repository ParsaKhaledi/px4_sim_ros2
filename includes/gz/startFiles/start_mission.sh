#!/bin/bash
# Legacy entry point. The monolithic microxrce_offboard.py script has been
# removed. This starts the px4_control node; mission scripts then use Drone.
exec "$(cd "$(dirname "$0")" && pwd)/gz_start_px4_control.sh"
