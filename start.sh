#!/bin/bash

# Function to check if a device exists
check_device() {
    if [ -e "$1" ]; then
        echo "Device $1 found."
        return 0
    else
        echo "Device $1 not found."
        return 1
    fi
}

# Function to set device permissions
set_device_permissions() {
    if [ -e "$1" ]; then
        echo "Setting permissions for $1"
        sudo chmod 666 "$1"
    fi
}

# Source ROS 2 setup
source /opt/ros/rolling/setup.bash
source install/setup.bash

# Set permissions for serial devices
for i in {0..9}; do
    set_device_permissions "/dev/ttyUSB$i"
    set_device_permissions "/dev/ttyACM$i"
done

# Check for motor and servo devices
MOTOR_PORT=""
SERVO_PORT=""

# Try to find motor and servo ports
for i in {0..9}; do
    if check_device "/dev/ttyUSB$i"; then
        if [ -z "$MOTOR_PORT" ]; then
            MOTOR_PORT="/dev/ttyUSB$i"
        elif [ -z "$SERVO_PORT" ]; then
            SERVO_PORT="/dev/ttyUSB$i"
        fi
    fi
done

# Use default ports if not found
if [ -z "$MOTOR_PORT" ]; then
    MOTOR_PORT="/dev/ttyUSB0"
    echo "No motor port detected, using default: $MOTOR_PORT"
fi

if [ -z "$SERVO_PORT" ]; then
    SERVO_PORT="/dev/ttyUSB1"
    echo "No servo port detected, using default: $SERVO_PORT"
fi

# Launch the system
ros2 launch arm_bringup arm_control.launch.py motor_port:=$MOTOR_PORT servo_port:=$SERVO_PORT
