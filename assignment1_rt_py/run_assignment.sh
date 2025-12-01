#!/bin/bash

SESSION="assignment_py"
WORKSPACE="/home/ubuntu/ros2_workshop"

# Chiudi sessioni precedenti
tmux kill-session -t $SESSION 2>/dev/null

#######################################
# Crea nuova sessione tmux
#######################################
tmux new-session -d -s $SESSION -c "$WORKSPACE"

#######################################
# Layout 2x2
#
#  ┌───────────────┬───────────────┐
#  │  P1 (0,0)     │  P2 (0,1)     │
#  ├───────────────┼───────────────┤
#  │  P3 (1,0)     │  P4 (1,1)     │
#  └───────────────┴───────────────┘
#######################################

# Dividi orizzontalmente → due pannelli
tmux split-window -h -t $SESSION:0

# Dividi verticalmente il pannello sinistro → crea P3
tmux select-pane -t $SESSION:0.0
tmux split-window -v

# Dividi verticalmente il pannello destro → crea P4
tmux select-pane -t $SESSION:0.1
tmux split-window -v

# Applica layout tiled
tmux select-layout -t $SESSION tiled

#######################################
# LANCIA I PROCESSI
#######################################

### Pane 1 (alto-sinistra): turtlesim GUI
tmux select-pane -t $SESSION:0.0
tmux send-keys "cd $WORKSPACE" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "ros2 run turtlesim turtlesim_node" C-m

### Pane 2 (alto-destra): spawn turtle2
tmux select-pane -t $SESSION:0.1
tmux send-keys "cd $WORKSPACE" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "ros2 service call /spawn turtlesim/srv/Spawn '{x: 2.0, y: 2.0, theta: 0.0, name: \"turtle2\"}'" C-m
tmux send-keys "bash" C-m

### Pane 3 (basso-sinistra): distance_node (Python)
tmux select-pane -t $SESSION:0.2
tmux send-keys "cd $WORKSPACE" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "ros2 run assignment1_rt_py distance_node" C-m

### Pane 4 (basso-destra): ui_node (Python)
tmux select-pane -t $SESSION:0.3
tmux send-keys "cd $WORKSPACE" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "ros2 run assignment1_rt_py ui_node" C-m

#######################################
# Entra nella sessione
#######################################
tmux attach -t $SESSION
