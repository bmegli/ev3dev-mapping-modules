#!/usr/bin/env bash

# This script expects:
# -XV11 LIDARS (input 1 and output C)
#
# Script:
# - changes input 1 mode to other-uart for XV11-LIDAR uart communication
# - changes outputs C mode to dc-motor for XV11-LIDAR motor control
# - spins the motor for some time to warm-up the LIDAR (they tend to work unstable when cold starting)

echo 'Changing input 1 mode to other-uart for XV11-LIDAR data'
echo other-uart > /sys/class/lego-port/port0/mode

echo 'Changing output C mode to dc-motor for XV11-LIDAR motor'
echo dc-motor > /sys/class/lego-port/port6/mode

echo 'Waiting for dc-motor device to be created'
sleep 1

echo 'Setting duty cycle setpoint for XV11-LIDAR 1 motor'
echo 44 > /sys/class/dc-motor/motor0/duty_cycle_sp

echo 'Warming up the XV11-LIDAR 1 motor (spinning 20 seconds)'
echo run-direct > /sys/class/dc-motor/motor0/command
sleep 20

echo 'Stopping XV11-LIDAR 1 motor'
echo stop > /sys/class/dc-motor/motor0/command

echo 'Done'
echo ''
echo 'You may now want to call ev3control, e.g.:'
echo './ev3control 8004 500'
