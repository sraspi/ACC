import time
from shutil import copyfile
Datum = time.strftime("%Y_%m_%d_%H_%M")
dateiname = ("/home/pi/data/" + Datum + ".txt")
copyfile("/home/pi/ACC/logfile.txt", dateiname)
