#!/usr/bin/env bash
# This is my guess at what the correct args are lol
ROS_DOMAIN_ID=1 RUST_LOG=debug dzd -d 1 -m peer -e tcp/10.42.0.200:7447 -s /DO12 -r /DO12/* -w /DO12/**