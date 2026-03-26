#!/bin/bash

# Argument --> controller type

# Define the Python script path
SCRIPT="mpc.py"

#NOISE=("2" "5" "8" "10" "15" "20")
NOISE=("0.1" "1.3" "2.5" "3.7" "5.0")
#ALPHA_ARG=("10" "20" "30" "40" "50")
ALPHA_ARG=("20" "30" "40")
#HORIZON_ARG=("15" "20" "25" "30" "35" "40" "45" "50")
HORIZON_ARG=("25"  "35"  "45")

LOG_METRICS="results_with_noise_"$1"_hor_"$2"_alpha_"$3".txt"
> "$LOG_METRICS"

counter=0

# Define the arguments for the Python script
CONTR_ARG=( "naive" "zerovel" "st" "htwa" "receding" "parallel2")
# Loop through the arguments
for ARG in "${ALPHA_ARG[@]}"; do
    for ARG1 in "${HORIZON_ARG[@]}"; do
        for ARG2 in "${NOISE[@]}"; do
            # Guess
            echo "Running $SCRIPT with controller "$1" horizon "$ARG1" alpha "$ARG" noise $ARG2"  | tee -a $LOG_METRICS
            if [ $counter -eq 0 ]; then
                python3 "$SCRIPT" -b -c=$1 --horizon=$ARG1 --alpha=$ARG --noise=$ARG2 >> "$LOG_METRICS" 2>&1
            else
                python3 "$SCRIPT" -c=$1 --horizon=$ARG1 --alpha=$ARG --noise=$ARG2 >> "$LOG_METRICS" 2>&1
            fi
            counter=$((counter + 1))
        done
    done
done
echo "All executions completed."