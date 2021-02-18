#!/bin/sh
#bootjobs.sh
cd /home/pi/noip 
sudo noip2 
cd
cd /home/pi/ACC
sh filecopy.sh
rm logfile.txt
python3 ACC5.0.py
