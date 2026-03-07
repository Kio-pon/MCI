#!/bin/bash
# Script to fix serial port permissions, kill any blocking processes, and run plot_adc.py

PORT="/dev/ttyUSB0"

# Kill any process holding the serial port (e.g. screen sessions)
echo "Checking for processes on $PORT..."
sudo fuser -k "$PORT" 2>/dev/null && echo "Killed blocking process(es)" && sleep 1

# Fix permissions
echo "Setting permissions on $PORT..."
sudo chmod 666 "$PORT"

# Activate venv and run
echo "Starting plot_adc.py..."
source /home/azyan/MCI_Copy/MCI-Lab07/lab7_env/bin/activate
python plot_adc.py
