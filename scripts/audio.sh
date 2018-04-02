#!/usr/bin/env bash

# dependency: trx installed in system path (tx only)

# argument 1: device (e.g. plughw:1)
# argument 2: bitrate kHz (e.g. 96)
# argument 3: host   (e.g. 192.168.0.100)
# argument 4: port   (e.g. 1350)

echo audio: device=$1 bitrate=$2 host=$3 port=$4
exec tx -d $1 -h $3 -p $4 -b $2 -f 480 -c 1
