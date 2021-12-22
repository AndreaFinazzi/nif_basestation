#!/usr/bin/env bash
# This is my guess at what the correct args are lol
# UHLC_MAX_DELTA_MS=1000 ROS_DOMAIN_ID=42 RUST_LOG=debug zenoh-bridge-dds -d 42 -m peer -e udp/10.42.4.200:7447 -s /DO12 -r /DO12/** -w /DO12/**  --rest-plugin \
# --allow "/nif_telemetry/.*|ros_discovery_info|/joystick/command|rc_to_ct"

UHLC_MAX_DELTA_MS=1000 ROS_DOMAIN_ID=42 RUST_LOG=debug \
    zenoh-bridge-dds \
    --no-multicast-scouting \
    -d 42 \
    -l udp/0.0.0.0:7447 \
    -e udp/10.42.4.200:7447 \
    --allow "/nif_telemetry/.*|ros_discovery_info|/joystick/command|rc_to_ct"
