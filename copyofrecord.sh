#!/bin/bash

SESSION="record"

tmux kill-session -t "$SESSION" 2>/dev/null

# Preserve access to the desktop display for OBS.
DISPLAY_VALUE="${DISPLAY:-:0}"
XAUTHORITY_VALUE="${XAUTHORITY:-$HOME/.Xauthority}"

# Window 1: OBS command ready.
tmux new-session -d -s "$SESSION" -n obs
tmux send-keys -t "$SESSION:obs" \
"DISPLAY=$DISPLAY_VALUE XAUTHORITY=$XAUTHORITY_VALUE LIBGL_ALWAYS_SOFTWARE=1 obs"

# Window 2: rosbag command ready.
tmux new-window -t "$SESSION" -n rosbag
tmux send-keys -t "$SESSION:rosbag" \
#'rosbag record -O images.bag /depthai_node/image/compressed'
rosbag record -O images_$(date +%Y%m%d_%H%M%S).bag /depthai_node/image/compressed

# Window 3: command ready to close the entire recording session.
tmux new-window -t "$SESSION" -n kill
tmux send-keys -t "$SESSION:kill" \
'tmux kill-session -t record'

tmux select-window -t "$SESSION:obs"
tmux attach-session -t "$SESSION"