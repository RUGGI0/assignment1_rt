#!/bin/bash

SESSION="assignment"
WORKSPACE="/home/ubuntu/ros2_workshop"

# Chiudi eventuali sessioni precedenti
tmux kill-session -t $SESSION 2>/dev/null

#######################################
# Crea sessione nuova
#######################################
tmux new-session -d -s $SESSION -c "$WORKSPACE"

#######################################
# Layout 2x2:
#
#  ┌───────────────┬───────────────┐
#  │  P1 (0,0)     │  P2 (0,1)     │
#  ├───────────────┼───────────────┤
#  │  P3 (1,0)     │  P4 (1,1)     │
#  └───────────────┴───────────────┘
#######################################

# Dividi orizzontalmente → 2 pannelli in alto
tmux split-window -h -t $SESSION:0

# Dividi verticalmente il pannello sinistro → crea P3
tmux select-pane -t $SESSION:0.0
tmux split-window -v

# Dividi verticalmente il pannello destro → crea P4
tmux select-pane -t $SESSION:0.1
tmux split-window -v

# Applica layout 2x2
tmux select-layout -t $SESSION tiled

#######################################
# ORA MANDIAMO I COMANDI AI PANNELLI
#######################################

### Pane 1 (alto-sinistra): turtlesim
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

### Pane 3 (basso-sinistra): distance_node
tmux select-pane -t $SESSION:0.2
tmux send-keys "cd $WORKSPACE" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "ros2 run assignment1_rt distance_node" C-m

### Pane 4 (basso-destra): ui_node
tmux select-pane -t $SESSION:0.3
tmux send-keys "cd $WORKSPACE" C-m
tmux send-keys "source install/setup.bash" C-m
tmux send-keys "ros2 run assignment1_rt ui_node" C-m

#######################################
# Entra nella sessione
#######################################
tmux attach -t $SESSION
