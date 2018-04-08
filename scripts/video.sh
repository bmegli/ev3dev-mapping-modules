#!/usr/bin/env bash

# argument 1: device (e.g. /dev/video0)
# argument 2: width  (e.g. 640)
# argument 3: height (e.g. 360)
# argument 4: bitrate (e.g. 3000000)
# argument 5: host   (e.g. 192.168.0.100)
# argument 6: port   (e.g. 8877)

# prerequisities: gstreamer installed
# script was written for Logitech C920 (the line with exposure)


echo video: device=$1 width=$2 height=$3 bitrate=$4 host=$5 port=$6

echo video: forcing Logitech C920 to capture at 30 FPS
v4l2-ctl --set-ctrl=exposure_auto_priority=0
v4l2-ctl --set-parm=30

echo video: starting gstreamer
exec gst-launch-1.0 uvch264src initial-bitrate=$4 average-bitrate=$4 iframe-period=4000 device=$1 name=src auto-start=true src.vidsrc ! video/x-h264,width=$2,height=$3,framerate=30/1 ! h264parse ! rtph264pay ! udpsink host=$5 port=$6

