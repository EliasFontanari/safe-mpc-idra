#!/bin/bash

# Argument --> controller type

# Define the Python script path
SCRIPT="compute_all_costs.py"

LOG_METRICS="opt_costs_computation_{$1}.txt"
> "$LOG_METRICS"

# Define the arguments for the Python script
ALPHA_ARG=( "10" "20" "30" "40" "50")  # "10" "20" 
HORIZON_ARG=("15" "20" "25" "30" "35" "40" "45" "50")

# Loop through the arguments
#for ARG in "${ALPHA_ARG[@]}"; do
for ARG_H in "${HORIZON_ARG[@]}"; do
    # Guess
    echo "Computing costs with alpha $1 horizon $ARG_H" 
    python3 "$SCRIPT" --alpha="$1" --horizon="$ARG_H" >> "$LOG_METRICS" 2>&1
done
#done

echo "All executions completed."