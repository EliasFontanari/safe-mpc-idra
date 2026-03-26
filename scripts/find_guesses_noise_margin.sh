#!/bin/bash

# Argument --> controller type

# Define the Python script path
SCRIPT="guess_acados.py"

#NOISE=("2" "5" "8" "10" "15" "20")
NOISE=("2.5" "5.0" "7.5" "10.0" "12.5" "15.0" "17.5" "20.0" "25.0" "30.0")
# NOISE=("7.5" "12.5" "17.5")
#ALPHA_ARG=("10" "20" "30" "40" "50")
ALPHA_ARG=("20")
#HORIZON_ARG=("15" "20" "25" "30" "35" "40" "45" "50")
HORIZON_ARG=("35" "40" )

MARGIN_JOINTS=("0.1" "0.1" "0.2" "0.4" "0.6" "0.8"  "1.0" "1.3" "2.6" "5")
MARGIN_COLLISION=("0.001" "0.002" "0.001" "0.0015" "0.0015" "0.002" "0.003" "0.004" "0.006" "0.008")

# MARGIN_JOINTS=("1.3")
# MARGIN_COLLISION=("0.008")

LOG_METRICS="results_guess_with_noise_"$1"_hor_"$2"_alpha_"$3".txt"
> "$LOG_METRICS"

counter=0
length=${#MARGIN_JOINTS[@]}
echo $length
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
                echo "Running $SCRIPT with controller receding horizon "$ARG1" alpha "$ARG" noise $ARG2 joint_bounds_margin=$ARG_JOINTS collision_margin=$ARG_COLLISIONS"  | tee -a $LOG_METRICS
                if [ "$counter" -eq 0 ]; then
                    python3 "$SCRIPT" -b -c=receding --horizon=$ARG1 --alpha=$ARG --joint_bounds_margin=$ARG_JOINTS --collision_margin=$ARG_COLLISIONS >> "$LOG_METRICS" 2>&1
                else
                    python3 "$SCRIPT" -c=receding --horizon=$ARG1 --alpha=$ARG --joint_bounds_margin=$ARG_JOINTS --collision_margin=$ARG_COLLISIONS >> "$LOG_METRICS" 2>&1
                fi
                counter=$((counter + 1))
            done
        done
    done
done 
echo "All executions completed."