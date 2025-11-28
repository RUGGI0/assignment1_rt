#!/bin/bash

SCRIPT_DIR="$(dirname "$0")"
RUN="$SCRIPT_DIR/run_assignment.sh"
SESSION="assignment"

# Gradient colors
C1='\e[31m'   # red
C2='\e[33m'   # yellow
C3='\e[32m'   # green
C4='\e[36m'   # cyan
C5='\e[34m'   # blue
RESET='\e[0m'

while true; do
    clear

    echo -e "${C1}                           ___           ___           ___                    ${RESET}"
    echo -e "${C1}                          /\  \         /\  \         /\  \                   ${RESET}"
    echo -e "${C1}                         /::\  \       /::\  \       /::\  \                  ${RESET}"
    echo -e "${C2}                        /:/\:\  \     /:/\:\  \     /:/\:\  \                 ${RESET}"
    echo -e "${C2}                       /::\ \:\  \   /:/  \:\  \   /:/  \:\  \                ${RESET}"
    echo -e "${C3}                      /:/\:\ \:\__\ /:/__/_\:\__\ /:/__/ \:\__\               ${RESET}"
    echo -e "${C3}                      \/_|::\/:/  / \:\  /\ \/__/ \:\  \ /:/  /               ${RESET}"
    echo -e "${C4}                         |:|::/  /   \:\ \:\__\    \:\  /:/  /                ${RESET}"
    echo -e "${C4}                         |:|\/__/     \:\/:/  /     \:\/:/  /                 ${RESET}"
    echo -e "${C5}                         |:|  |        \::/  /       \::/  /                  ${RESET}"
    echo -e "${C5}                          \|__|         \/__/         \/__/                   ${RESET}"



    echo ""        
    echo "                            =============================="
    echo "                                  ASSIGNMENT TOOL"
    echo "                            =============================="
    echo ""        
    echo "                                  S = Start"
    echo "                                  R = Restart"
    echo "                                  A = Annull"
    echo "                                  Q = Quit"
    echo ""        
    echo "                            =============================="

    read -n1 -s KEY

    case "$KEY" in
        S|s)
            tmux kill-session -t "$SESSION" 2>/dev/null
            x-terminal-emulator -e "$RUN" &
            ;;
        R|r)
            tmux kill-session -t "$SESSION" 2>/dev/null
            x-terminal-emulator -e "$RUN" &
            ;;
        A|a)
            tmux kill-session -t "$SESSION" 2>/dev/null
            ;;
        Q|q)
            clear
            exit 0
            ;;
    esac
done