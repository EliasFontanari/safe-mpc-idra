#!/bin/bash

MPC_SCRIPT="mpc.py"

# Define the arguments for the Python script
ALPHA_ARG=("10" "20" "30" "40" "50")

HORIZON_ARG=("50" "15" "20" "25" "30" "35" "40" "45" )

LOG_PREFIX="$1"
LOG_MPC="${LOG_PREFIX}_mpc_sm.txt"
> "$LOG_MPC"

counter=0

echo "$LOG_PREFIX"
for ARG_H in "${HORIZON_ARG[@]}"; do
    for ARG in "${ALPHA_ARG[@]}"; do
        # MPC
        echo "Running $MPC_SCRIPT with argument alpha $ARG  horizon $ARG_H" | tee -a "$LOG_MPC"
        if [ $counter -eq 0 ]; then
            python3 "$MPC_SCRIPT"  -c=$1 --alpha=$ARG --horizon="$ARG_H" >> "$LOG_MPC" 2>&1
        else
            python3 "$MPC_SCRIPT" -c=$1 --alpha=$ARG --horizon="$ARG_H" >> "$LOG_MPC" 2>&1
        fi
        echo "Completed execution" | tee -a "$LOG_MPC"
        echo "----------------------------------------" | tee -a "$LOG_MPC"
        counter=$((counter + 1))
    done
done

echo "All executions completed. Log written to $LOG_MPC"