#!/usr/bin/env bash

echo "Started battery monitoring script"
if ros2 topic echo /raptor_dbw_interface/misc_report_do | grep -m 1 "battery_voltage: 12[0-7]"; then
    while true
    do
	  spd-say "Battery low"
	  sleep 2
    done
fi       
