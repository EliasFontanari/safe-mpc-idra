#!/bin/bash

# Argument --> controller type

# Define the Python script path
SCRIPT="mpc.py"

#NOISE=("2" "5" "8" "10" "15" "20")
NOISE=("10.0")
#ALPHA_ARG=("10" "20" "30" "40" "50")
ALPHA_ARG=("20")
#HORIZON_ARG=("15" "20" "25" "30" "35" "40" "45" "50")
HORIZON_ARG=("45")

# MARGIN_JOINTS=("0.0" "0.1" "2.6" "5" "10" )
# MARGIN_COLLISION=("0.0" "0.005" "0.01" "0.02" "0.03" )

MARGIN_JOINTS=("1.3")
MARGIN_COLLISION=("0.008")


LOG_METRICS="results_mpc_with_noise_"$1"_hor_"$2"_alpha_"$3"__2.txt"
> "$LOG_METRICS"

counter=0
length=${#MARGIN_JOINTS[@]}

# Define the arguments for the Python script
CONTR_ARG=( "naive" "zerovel" "st" "htwa" "receding" "parallel2")
# Loop through the arguments
for ARG in "${ALPHA_ARG[@]}"; do
    for ARG1 in "${HORIZON_ARG[@]}"; do
        for ARG2 in "${NOISE[@]}"; do
            for ((i=0; i<length; i++)); do
                # MPC
                ARG_JOINTS=${MARGIN_JOINTS[i]}
                ARG_COLLISIONS=${MARGIN_COLLISION[i]}
                echo "Running $SCRIPT with controller "$1" horizon "$ARG1" alpha "$ARG" noise $ARG2 joint_bounds_margin=$ARG_JOINTS collision_margin=$ARG_COLLISIONS"  | tee -a $LOG_METRICS
                nohup python3 "$SCRIPT" -b -c=$1 --horizon=$ARG1 --alpha=$ARG --noise=$ARG2 --joint_bounds_margin=$ARG_JOINTS --collision_margin=$ARG_COLLISIONS >> "$LOG_METRICS" 2>&1
                counter=$((counter + 1))
            done
        done
    done
done
echo "All executions completed."