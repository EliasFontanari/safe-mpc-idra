#!/bin/bash

# Argument --> controller type

# Define the Python script path
SCRIPT="metrics_count_fails.py"

LOG_METRICS="data_results_{$1}.txt"
> "$LOG_METRICS"

#NOISE=("2" "5" "8" "10" "15" "20")
NOISE=("2.5" "5.0" "10.0" "15.0" "20.0")
#NOISE=("7.5" "12.5" "17.5")
#ALPHA_ARG=("10" "20" "30" "40" "50")
ALPHA_ARG=("20" )
#HORIZON_ARG=("15" "20" "25" "30" "35" "40" "45" "50")
HORIZON_ARG=("25")

MARGIN_JOINTS=("0.1" "0.2" "0.4" "0.6" "0.8"  "1.0" "1.3" "2.6" "5")
MARGIN_COLLISION=("0.001" "0.001" "0.0015" "0.0015" "0.002" "0.003" "0.004" "0.006" "0.008")

# # Loop through the arguments
# #for ARG in "${ALPHA_ARG[@]}"; do
# for ARG_H in "${HORIZON_ARG[@]}"; do
#     for ARG_N in "${NOISES[@]}"; do
#         echo "Running metrics.py with argument alpha $1 horizon $ARG_H noise $ARG_N" | tee -a $LOG_METRICS
#         python3 "$SCRIPT" --alpha="$1" --horizon="$ARG_H" --noise="$ARG_N" >> "$LOG_METRICS" 2>&1
#     done
# done
# #done

length=${#MARGIN_JOINTS[@]}

# Loop through the arguments
for ARG in "${ALPHA_ARG[@]}"; do
    for ARG1 in "${HORIZON_ARG[@]}"; do
        for ARG2 in "${NOISE[@]}"; do
            for ((i=0; i<length; i++)); do
                # MPC
                ARG_JOINTS=${MARGIN_JOINTS[i]}
                ARG_COLLISIONS=${MARGIN_COLLISION[i]}
                echo "Running $SCRIPT with controller "$1" horizon "$ARG1" alpha "$ARG" noise $ARG2 joint_bounds_margin=$ARG_JOINTS collision_margin=$ARG_COLLISIONS"  | tee -a $LOG_METRICS
                python3 "$SCRIPT" --horizon=$ARG1 --alpha=$ARG --noise=$ARG2 --joint_bounds_margin=$ARG_JOINTS --collision_margin=$ARG_COLLISIONS >> "$LOG_METRICS" 2>&1
                counter=$((counter + 1))
            done
        done
    done
done


echo "All executions completed."