#!/bin/bash

# Argument --> controller type

# Define the Python script path
SCRIPT="mpc.py"

#NOISE=("2" "5" "8" "10" "15" "20")
NOISE=( "5.0")
#NOISE=("7.5" "12.5" "17.5")
#ALPHA_ARG=("10" "20" "30" "40" "50")
ALPHA_ARG=("20" )
#HORIZON_ARG=("15" "20" "25" "30" "35" "40" "45" "50")
HORIZON_ARG=("25" )

MARGIN_JOINTS=(  "0.6")
MARGIN_COLLISION=( "0.0015")




compact_date=$(date +"%Y_%m_%d_%H_%M_%S")
echo $compact_date

LOG_METRICS="../logs/results_mpc_with_noise_"$1"_hor_"$2"_alpha_"$3"_"$compact_date".txt"
> "$LOG_METRICS"

counter=0
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
                if [ "$counter" -eq 0 ]; then
                    python3 "$SCRIPT"  -c=$1 --horizon=$ARG1 --alpha=$ARG --noise=$ARG2 --joint_bounds_margin=$ARG_JOINTS --collision_margin=$ARG_COLLISIONS >> "$LOG_METRICS" 2>&1
                else
                    python3 "$SCRIPT" -c=$1 --horizon=$ARG1 --alpha=$ARG --noise=$ARG2 --joint_bounds_margin=$ARG_JOINTS --collision_margin=$ARG_COLLISIONS >> "$LOG_METRICS" 2>&1
                fi

                # collisions=$?
                # if [ $collisions -eq 0 ]; then
                #     echo "Skipping to the next noise, that is $ARG2 %"
                #     i=$length
                # fi
                counter=$((counter + 1))
            done
        done
    done
done
echo "All executions completed."