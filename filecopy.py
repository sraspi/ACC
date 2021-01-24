import time
from shutil import copyfile
Datum = time.strftime("%Y_%m_%d")
dateiname = ("/home/pi/data/" + Datum + ".txt")
copyfile("/home/pi/ACC/logfile.txt", dateiname)
