#!/usr/bin/env bash
export timestamp=$(date +"%Y-%m-%d_%H-%M-%S")
export LOG_FILE=/home/$USER/network-monitor-recordings/network_logs_${timestamp}.log

mkdir -p /home/$USER/network-monitor-recordings

echo 'Monitoring network stats at '
echo $LOG_FILE

ifstat -t -i eth0 0.5 &> $LOG_FILE

wait
