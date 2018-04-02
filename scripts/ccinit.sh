#!/usr/bin/env bash

echo 'configuring pins for UART 1'
config-pin p9.24 uart
config-pin p9.26 uart

echo 'configuring pins for UART 4'
config-pin p9.11 uart
config-pin p9.13 uart

echo 'configuring PWM chip 3'
config-pin p9.14 pwm


