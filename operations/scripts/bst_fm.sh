#!/usr/bin/env bash
# This is my guess at what the correct args are lol
ROS_DOMAIN_ID=2 RUST_LOG=debug zenoh-bridge-dds -d 2 -m peer -e tcp/10.42.0.200:7447 -s /DO12 -r /DO12/** -w /DO12/** \
--allow "ros_discovery_info|diagnostics|/joystick/.*|rt/lateral_error|lookahead_error|ct_report|misc_report_do|pt_report|/vehicle/.*|rc_to_ct|/raptor_dbw_interface/pt_report"
