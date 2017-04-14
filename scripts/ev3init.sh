#!/usr/bin/env bash

# This script expects:
# -CruizCore XG1300L gyroscope (input 3)
# -2 XV11 LIDARS (input 1 and 2, and outputs B and C)
# -2 Large Servo Motors (outputs A and D)
#
# Script:
# - loads CruizCore XG1300L driver manually (it can't be autodetected)
# - changes inputs 1 and 2 modes to other-uart for XV11-LIDAR uart communication
# - changes outputs B and C modes to dc-motor for XV11-LIDAR motors control
# - spins the motors for some time to warm-up the LIDARS (they tend to work unstable when cold starting)

echo 'ev3dev-mapping-modules initialization script'
echo '(once after boot to init ports for lidars and gyroscope)'
echo ''

echo 'Loading MicroInfnity CruizCore XG1300L gyroscope I2C driver'
echo mi-xg1300l 0x01 > /sys/bus/i2c/devices/i2c-5/new_device

echo 'Changing input 1 mode to other-uart (XV11-LIDAR 1)'
echo other-uart > /sys/class/lego-port/port0/mode
echo 'Changing input 2 mode to other-uart (XV11-LIDAR 2)'
echo other-uart > /sys/class/lego-port/port1/mode

echo 'Changing output B mode to dc-motor (XV11-LIDAR 1 motor)'
echo dc-motor > /sys/class/lego-port/port5/mode
echo 'Changing output C mode to dc-motor (XV11-LIDAR 2 motor)'
echo dc-motor > /sys/class/lego-port/port6/mode

echo 'Waiting for dc-motor devices to be created'
sleep 1

echo 'Setting duty cycle setpoint for XV11-LIDAR 1 motor'
echo 44 > /sys/class/dc-motor/motor0/duty_cycle_sp
echo 'Setting duty cycle setpoint for XV11-LIDAR 2 motor'
echo 44 > /sys/class/dc-motor/motor1/duty_cycle_sp

echo 'Warming up the XV11-LIDAR 1 motor (spinning 20 seconds)'
echo run-direct > /sys/class/dc-motor/motor0/command
echo 'Warming up the XV11-LIDAR 2 motor (spinning 20 seconds)'
echo run-direct > /sys/class/dc-motor/motor1/command
sleep 20

echo 'Stopping XV11-LIDAR 1 motor'
echo stop > /sys/class/dc-motor/motor0/command
echo 'Stopping XV11-LIDAR 2 motor'
echo stop > /sys/class/dc-motor/motor1/command

echo 'Done'


