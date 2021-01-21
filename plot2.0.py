import time
from matplotlib import pyplot as plt
import numpy as np
import os
import sys

Startzeit = time.time() #Versuchsstartzeit
x1=np.genfromtxt('G:\\Dropbox\\Dokumente etc\\Raspi\\ACC\\test.txt',skip_header=3,usecols=(3))
y1=np.genfromtxt('G:\\Dropbox\\Dokumente etc\\Raspi\\ACC\\test.txt',skip_header=3,usecols=(5))
y2=np.genfromtxt('G:\\Dropbox\\Dokumente etc\\Raspi\\ACC\\test.txt',skip_header=3,usecols=(7))
y3=np.genfromtxt('G:\\Dropbox\\Dokumente etc\\Raspi\\ACC\\test.txt',skip_header=3,usecols=(9))
y4=np.genfromtxt('G:\\Dropbox\\Dokumente etc\\Raspi\\ACC\\test.txt',skip_header=3,usecols=(11))
y5=np.genfromtxt('G:\\Dropbox\\Dokumente etc\\Raspi\\ACC\\test.txt',skip_header=3,usecols=(13))
y6=np.genfromtxt('G:\\Dropbox\\Dokumente etc\\Raspi\\ACC\\test.txt',skip_header=3,usecols=(15))
y7=np.genfromtxt('G:\\Dropbox\\Dokumente etc\\Raspi\\ACC\\test.txt',skip_header=3,usecols=(17))
y8=np.genfromtxt('G:\\Dropbox\\Dokumente etc\\Raspi\\ACC\\test.txt',skip_header=3,usecols=(19))
y9=np.genfromtxt('G:\\Dropbox\\Dokumente etc\\Raspi\\ACC\\test.txt',skip_header=3,usecols=(21))
y10=np.genfromtxt('G:\\Dropbox\\Dokumente etc\\Raspi\\ACC\\test.txt',skip_header=3,usecols=(23))

xwerte = []
ywerte = []

print("Zeit [h]", x1)
print("BME_temp [°C]", y1)
print("BME_pressure: ", y2)
print("BME_hunidity: ", y3)
print("T1: ", y4)
print("T2: ", y5)
print("A0: ", y6)
print("A1: ", y7)
print("A2: ", y8)
print("Vg: ", y9)
print("CPU_temp: ", y10)


Vg = 0

plt.ion()

x = np.linspace(0, 100)
y = np.linspace(0, 500)
figure, ax = plt.subplots(figsize=(10,8))



try:
    for a in range(1,1000000):
        Endzeit = time.time()
        delta = (Endzeit - Startzeit)/60/60 # Zeit in Stunden seit Versuchsstart
        
        xwerte.append(delta)
        ywerte.append(Vg)
        plt.title("TGZ",fontsize=25)
        plt.xlabel("Zeit seit Versuchsstart in Stunden",fontsize=20)
        plt.ylabel("Gasvolumen in mL",fontsize=20)
        
        plt.plot(xwerte, ywerte)
        figure.canvas.flush_events()
        Vg = Vg + 2.5
        time.sleep(1)
        
except KeyboardInterrupt:
    print("keyboardInterrupt")
    timestr = time.strftime("%Y%m%d_%H%M%S")
    print("\nBye")
    sys.exit()
