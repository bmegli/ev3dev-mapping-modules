#!/usr/bin/env bash

# This script expects:
# -CruizCore XG1300L gyroscope (input 3)
# -2 Large Servo Motors (outputs A and D)
#
# Script:
# - loads CruizCore XG1300L driver manually (it can't be autodetected)

echo 'ev3dev-mapping-modules initialization script'
echo '(once after boot to init gyroscope)'
echo ''

echo 'Loading MicroInfnity CruizCore XG1300L gyroscope I2C driver'
echo '(this may require root privileges)'
echo mi-xg1300l 0x01 > /sys/bus/i2c/devices/i2c-5/new_device

echo 'Done'
echo ''
echo 'You may now want to call ev3control, e.g.:'
echo './ev3control 8004 500'
