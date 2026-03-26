#!/bin/bash

# Define the Python script path
GUESS_SCRIPT="guess_acados.py"

# Define the arguments for the Python script
ALPHA_ARG=("20" "30" "40")
HORIZON_ARG=( "25"  "35"  "45")

export PYTHONPATH=:$(pwd)/../src


counter=0

# find guesses
LOG_GUESS="log_guesses_sm_hor_$1.txt"
LOG_GUESS_NAIVE="naive_log_guesses_sm_hor_$1.txt"
LOG_GUESS_ZEROVEL="zerovel_log_guesses_sm_hor_$1.txt"
LOG_GUESS_STWA="stwa_log_guesses_sm_$1.txt"

> "$LOG_GUESS"
> "$LOG_GUESS_NAIVE"
> "$LOG_GUESS_ZEROVEL"
> "$LOG_GUESS_STWA"


pwd >> "$LOG_GUESS_STWA"


count=0
for ALPHA in "${ALPHA_ARG[@]}"; do
    for ARG in "${HORIZON_ARG[@]}"; do
        # Guess
        echo "Running $GUESS_SCRIPT with argument alpha $ALPHA horizon $ARG" | tee -a "$LOG_GUESS" "$LOG_GUESS_STWA"    
        python3 "$GUESS_SCRIPT" -c="receding" --alpha=$ALPHA --horizon="$ARG" >> "$LOG_GUESS_STWA" 2>&1 
        echo "Completed execution" | tee -a "$LOG_GUESS"
        echo "----------------------------------------" | tee -a "$LOG_GUESS"
        sleep 1
    done
done