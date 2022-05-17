#!/bin/sh
#bootjobs.sh
cd /home/pi/noip-2.1.9-1
sudo noip2
cd
sleep 60
sudo mount -a
cd
cd /home/pi/ACC
sh filecopy.sh
rm logfile.txt
python3 ACC6.1_4OW.py
