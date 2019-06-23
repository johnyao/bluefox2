#!/bin/bash

# Script to copy over header and library files from the system-wide bluefox driver installation to the ROS driver
# This assumes that the bluefox driver has already been installed system-wide
# This should be run before the first build if the system-wide bluefox driver has a version different from 2.26.0

# Set path to the ROS bluefox package
package_path=/home/nvidia/sandbox/john_sandbox/wet/src/bluefox2
opt_path=/opt/mvIMPACT_Acquire

# Remove existing headers and libraries
rm ${package_path}/mvIMPACT/lib/arm64/*
rm -rf ${package_path}/mvIMPACT/include/*

# Copy over headers and libraries from system-wide bluefox installation
cp -r /opt/mvIMPACT_Acquire/lib/arm64  ${package_path}/mvIMPACT/lib/
cp -r ${opt_path}/common ${opt_path}/mvDeviceManager ${opt_path}/DriverBase ${opt_path}/mvIMPACT_CPP ${opt_path}/mvPropHandling ${package_path}/mvIMPACT/include
