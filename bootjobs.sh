#!/bin/sh
#bootjobs.sh
cd /home/pi/noip 
sudo noip2 
cd
cd /home/pi/ACC
rm logfile.txt
python3 ACC4.7.py
