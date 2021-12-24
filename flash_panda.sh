#!/usr/bin/bash

cd /data/openpilot/panda/board
exec ./recover.sh

if [ -f /EON ]; then
  reboot
elif [ -f /TICI ]; then
  sudo reboot
fi
