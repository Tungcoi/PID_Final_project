#!/bin/bash

# Default PID parameters
STEER_KP=0.35
STEER_KI=0.001
STEER_KD=0.2
THROTTLE_KP=0.7
THROTTLE_KI=0.1
THROTTLE_KD=0.2

# Check if custom parameters are provided as arguments
if [ "$#" -eq 6 ]; then
  STEER_KP=$1
  STEER_KI=$2
  STEER_KD=$3
  THROTTLE_KP=$4
  THROTTLE_KI=$5
  THROTTLE_KD=$6
elif [ "$#" -ne 0 ]; then
  echo "Invalid number of arguments! Provide either 0 or 6 arguments."
  echo "Usage: ./run_main_pid.sh [STEER_KP STEER_KI STEER_KD THROTTLE_KP THROTTLE_KI THROTTLE_KD]"
  exit 1
fi

# Run the pid_controller with the provided or default PID parameters
./pid_controller/pid_controller $STEER_KP $STEER_KI $STEER_KD $THROTTLE_KP $THROTTLE_KI $THROTTLE_KD &
sleep 1.0

# Run the simulator API and append stdout to the same log file
python3 simulatorAPI.py