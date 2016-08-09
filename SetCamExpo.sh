#!/bin/bash
clear
echo "Setting Exposure to Low"
v4l2-ctl -c exposure_auto=1
v4l2-ctl -c exposure_absolute=10
echo
