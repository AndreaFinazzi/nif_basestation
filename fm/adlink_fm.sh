#!/usr/bin/env bash
# This is copied from the adlink - not the bst
# ROS_DOMAIN_ID=1 RUST_LOG=debug dzd -d 1 -m peer -l tcp/10.42.0.200:7447 -s /DO12 -r /DO12/* -w /DO12/**
ROS_DOMAIN_ID=1 RUST_LOG=debug dzd -d 1 -m peer -l tcp/0.0.0.0:7447 -s /DO12 -r /DO12/* -w /DO12/**