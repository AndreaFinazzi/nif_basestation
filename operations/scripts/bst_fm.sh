#!/usr/bin/env bash
# This is my guess at what the correct args are lol
ROS_DOMAIN_ID=42 RUST_LOG=debug zenoh-bridge-dds -d 42 -m peer -e udp/10.42.4.200:7447 -s /DO12 -r /DO12/** -w /DO12/**  --rest-plugin \
--allow "/telemetry/.*|ros_discovery_info|/joystick/command|rc_to_ct"
