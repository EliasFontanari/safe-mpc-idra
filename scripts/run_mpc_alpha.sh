#!/bin/bash

MPC_SCRIPT="mpc.py"

# Define the arguments for the Python script
HORIZON_ARG=("15" "20" "25" "30" "35" "40" "45" "50")

LOG_PREFIX="$2"
LOG_MPC="${LOG_PREFIX}_mpc_sm_"$1"rebuild_alpha.txt"
> "$LOG_MPC"

counter=0

echo "$LOG_PREFIX"
    for ARG_H in "${HORIZON_ARG[@]}"; do
        # MPC
        echo "Running $MPC_SCRIPT with argument alpha $1 horizon $ARG_H" | tee -a "$LOG_MPC"
        python3 "$MPC_SCRIPT" --alpha=$1 -c=$2 --horizon="$ARG_H" >> "$LOG_MPC" 2>&1
        echo "Completed execution" | tee -a "$LOG_MPC"
        echo "----------------------------------------" | tee -a "$LOG_MPC"
        counter=$((counter + 1))
    done

echo "All executions completed. Log written to $LOG_MPC"