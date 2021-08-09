#!/usr/bin/env bash
# This is copied from the adlink - not the bst
ROS_DOMAIN_ID=42 RUST_LOG=debug dzd -d 1 -m peer -l tcp/10.42.4.200:7447 -s /DO12 -r /DO12/* -w /DO12/**