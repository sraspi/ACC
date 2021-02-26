from matplotlib import pyplot as plt
import numpy as np
import smbus2
import bme280
import threading
import subprocess
from queue import Queue
import RPi.GPIO as GPIO
import time
import sys
import Nokia_LCD as LCD
import Adafruit_GPIO.SPI as SPI
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import os
from gpiozero import CPUTemperature
import smtplib, ssl
import email
from email import encoders
from email.mime.base import MIMEBase
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText


# Import the ADS1115 module.
# Create an ADS1115 ADC (16-bit) instance.
#from ADS1x15 import ADS1115
#adc = ADS1115()


#Kernelmodule laden
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

#1wire-Tempsensoren auslesen:
sensorcount = 2                                            #Festlegen, wieviele Sensoren vorhanden sind
sensors = ['28-3c01d6072b77', '28-3c01d607858e'];          #Array mit den Sensor-IDs
sensorpath = '/sys/bus/w1/devices/'                        #Pfad zum Sensorverzeichnis
sensorfile = '/w1_slave'                                   #Geraetedatei

#Variablen definieren
n = 0
m = 0
o = 0
p = 0

Vg = 0 #aufsummierte Interrupts
V = 0 #Vg/2 wegen >BOTH

Tgn = 0 #aufsummierte Taster_grün
Trt = 0 #aufsummierte Taster_rot

z1 = 0

port = 1 							# init BME280
address = 0x76
#bus = smbus2.SMBus(port)
#calibration_params = bme280.load_calibration_params(bus, address)

#Listen für Messwerte definieren
x1 = []
y1 = []
y2 = []
y3 = []
y4 = []
y5 = []
y6 = []
y7 = []
y8 = []
y9 = []
y10 = []

y1m =[]
y2m =[]
y3m =[]




#Listen für ADSmax-Werte definieren
A0 = [0]*50
A1 = [0]*50
A2 = [0]*50
A3 = [0]*50

#ADS settings
values = [0]*4
channel = 0
GAIN = 1

#allg. settings
timestr = time.strftime("%Y%m%d_%H%M%S")
Dateiname = "/home/pi/ACC/logfile.txt"
Startzeit = time.time() #Versuchsstartzeit

#Settings z.B.Display, GPIOs
GPIO.setmode(GPIO.BCM)
spiSettings = SPI.SpiDev(0, 0, max_speed_hz=4000000)#Settings Nokia LCD
d = LCD.PCD8544(23, 24, spi=spiSettings)            #Settings Nokia LCD
image = Image.new('1', (LCD.LCDWIDTH, LCD.LCDHEIGHT))

GPIO.setup(6, GPIO.IN, pull_up_down = GPIO.PUD_UP) # GPIO 6  als Input definieren und Pullup-Widerstand aktivieren #Tgn
GPIO.setup(25, GPIO.IN, pull_up_down = GPIO.PUD_UP) # GPIO 6  als Input definieren und Pullup-Widerstand aktivieren #Trt
GPIO.setup(13, GPIO.IN, pull_up_down = GPIO.PUD_UP) # GPIO 6  als Input definieren und Pullup-Widerstand aktivieren # TGZ_Vg

GPIO.setup(9, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(26, GPIO.OUT) #power
GPIO.setup(27, GPIO.OUT)

def LED_ON():
    GPIO.output(9, GPIO.HIGH)
    GPIO.output(12, GPIO.HIGH) #BL_on
    GPIO.output(16, GPIO.HIGH)
    GPIO.output(17, GPIO.HIGH)
    GPIO.output(19, GPIO.HIGH)
    GPIO.output(20, GPIO.HIGH)
    GPIO.output(21, GPIO.HIGH)
    GPIO.output(22, GPIO.HIGH)
    GPIO.output(26, GPIO.HIGH) #power_on
    GPIO.output(27, GPIO.HIGH)

def LED_OFF():
    GPIO.output(9, GPIO.LOW)
    GPIO.output(12, GPIO.LOW)
    GPIO.output(16, GPIO.LOW)
    GPIO.output(17, GPIO.LOW)
    GPIO.output(19, GPIO.LOW)
    GPIO.output(20, GPIO.LOW)
    GPIO.output(21, GPIO.LOW)
    GPIO.output(22, GPIO.LOW)
    GPIO.output(27, GPIO.LOW)
    GPIO.output(26, GPIO.LOW)


def TGZ_sim1():
    a = 0
    while (a < 10):
        LED_ON()
        time.sleep(0.025)
        LED_OFF()
        time.sleep(0.025)
        a = a +1

def Input():
    header = ("ACC5.1.py started at: " +timestr + '\n' + '\n' + "Zeit ," + "t[h] ," + "BME_temp ,"  + "BEM_pressure ," + "BME_humidity ," + "T1 ," + "T2 ," + "A0 ," + "A1 ," + "A2 ," + "V ," + "CPU_temp, " '\n')
    data = open(Dateiname, "a")
    data.write(str(header))
    data.close()


def Nokia():
    d.begin(contrast=60)
    d.clear()
    d.display()
    time.sleep(0.15)
    #GPIO.output(12, GPIO.HIGH) #BL_ON
    draw = ImageDraw.Draw(image)
    draw.rectangle((0,0,84,48), outline=255, fill=255)
    #draw.ellipse((2,2,27,22), outline=0, fill=255)
    #draw.rectangle((35,2,54,22), outline=0, fill=255)
    #draw.polygon([(63,22), (73,2), (83,22)],
      #          outline=0, fill=255)
    font = ImageFont.load_default()
    draw.text((1,1),  Text0, font=font)
    draw.text((1,10), Text1, font=font)
    draw.text((1,19), Text2, font=font)
    draw.text((1,28), Text3, font=font)
    draw.text((1,37), Text4, font=font)
    d.image(image)
    d.display()
    time.sleep(0.1)



def ads(): # Read all the ADC channel values in a list.
    global A0max
    global A1max
    global A2max
    global A3max

    for n in range(50):
        # Read the specified ADC channel using the previously set gain value.
        A0[n] = adc.read_adc(0, gain=GAIN)
        A1[n] = adc.read_adc(1, gain=GAIN)
        A2[n] = adc.read_adc(2, gain=GAIN)
        A3[n] = adc.read_adc(3, gain=GAIN)
    A0max=round(((max(A0)-13230)*4.096/32768),0)
    A1max=round(((max(A1)-13365)*4.096/32768)*10*240,2)
    A2max=round(((max(A2)-13230)*4.096/32768),0)
    A3max=round(((max(A3)-13230)*4.096/32768)*10*240,2)

def callsensor(sensor):

    f = open(sensorpath + sensor + sensorfile, 'r')       #Pfad, Sensor-ID und Geraetedatei zusammensetzen, Datei im Lesemodus oeffnen
    lines = f.readlines()                     #Inhalt der Datei in lines schreiben
    f.close()                         #Datei schliessen
    temp_line = lines[1].find('t=')               #Den Index (Position) von t= in temp_line schreiben
    temp_output = lines[1].strip() [temp_line+2:]       #Index 1 (Zeile 2) strippen und die Zeichen nach t= in temp_output schreiben
    temp_celsius = float(temp_output) / 1000            #Tausendstelgrad durch 1000 teilen und so in Grad Celsius wandeln, in temp_celsius
    return temp_celsius

Input() #Nutzereingaben

# Callback-Funktionen:

def V_g(channel):
    global Vg
    global V
    global Vt_start
    global Vt_diff

    Vt_end = time.time()
    Vt_diff = (Vt_end - Vt_start)
    Vg = Vg + 1
    V = Vg*2.5

def T_gn(channel):
    global Tgn
    Tgn = Tgn + 1

def T_rt(channel):
    global Trt
    Trt = Trt + 1


#Display-Ausgaben konfigurieren:

def txt1():                            #Text Messwerte
    global Text0
    global Text1
    global Text2
    global Text3
    global Text4

    Text0 = ("T1 " + str(T1))
    Text1 = ("T2 " + str(T2))
    Text2 = ("V" + str(V))
    Text3 = ("--------")
    Text4 = ("--------")

def txt2():                            #Abfrage boot/reboot?
    global Text0
    global Text1
    global Text2
    global Text3
    global Text4

    Text0 = ("REBOOT?   2*RT")
    Text1 = ("SHUTDOWN? 3*RT")
    Text2 = ("Tgn:" + str(Tgn))
    Text3 = ("Trt:" + str(Trt))
    Text4 = ("wait 4 sec.")


def txt3():                            #boot text
    global Text0
    global Text1
    global Text2
    global Text3
    global Text4
    timestr = time.strftime("%Y%m%d_%H:%M")

    Text0 = ("ACC5.1.py")
    Text1 = ("LCD-check")
    Text2 = ("BL-check")
    Text3 = (str(timestr))
    Text4 = ("          ")
    



def txt4():                            # reboot text
    global Text0
    global Text1
    global Text2
    global Text3
    global Text4
    timestr = time.strftime("%Y%m%d_%H:%M")

    Text0 = ("Tgn:" + str(Tgn))
    Text1 = ("Trt:" + str(Trt))
    Text2 = ("reboot in 4 sec")
    Text3 = (str(timestr))
    Text4 = ("  BYE!!! ")


def txt5():                           # shutdown text
    global Text0
    global Text1
    global Text2
    global Text3
    global Text4
    timestr = time.strftime("%Y%m%d_%H:%M")
    print("shutdown in 4 sec")
    Text0 = ("Tgn:" + str(Tgn))
    Text1 = ("Trt:" + str(Trt))
    Text2 = ("shutdown 4 sec")
    Text3 = (str(timestr))
    Text4 = ("  BYE!!!    ")
    time.sleep(4)



def bme():
    global bme_temp_m
    global bme_pressure_m
    global bme_humidity_m

    global bme_temp
    global bme_pressure
    global bme_humidity


    data = bme280.sample(bus, address, calibration_params)

    bme_temp_m = (round((data.temperature - 1.4),2))
    bme_temp = str(round((data.temperature - 1.4),2))

    bme_pressure_m = (round((data.pressure +21), 0))
    bme_pressure = str(round((data.pressure +21), 0))

    bme_humidity_m = (round((data.humidity + 5),0))
    bme_humidity = str(round((data.humidity + 5),0))



def DS1820():
    global T1
    global T2
    s1 = sensors[0]
    s2 = sensors[1]
    #s3 = sensors[2]
    #s4 = sensors[3]
    #T1 = round(((callsensor(s1)),1)
    #T2 = round(((callsensor(s2)),1)
    T1 = round(((callsensor(s1)) - 1), 2)
    T2 = round(((callsensor(s2))  - 1), 2)

t = threading.Thread(target=TGZ_sim1)
t.start()
time.sleep(1.5)                          #time.sleep notwendig um auf thread-Ende zu warten!
GPIO.output(12, GPIO.HIGH)
txt3()                                 #LCD-Ausgabe von boot-text
Nokia()
time.sleep(5)
GPIO.output(12, GPIO.LOW)


Startzeit = time.time()                #Versuchsstartzeit
Vt_start = time.time()                 #Startzeit fuer Ermittlung von Vt_diff

# Interrupt-Event hinzufuegen
GPIO.add_event_detect(25, GPIO.RISING, callback = T_rt, bouncetime = 250)
GPIO.add_event_detect(6, GPIO.RISING, callback = T_gn, bouncetime = 250)
GPIO.add_event_detect(13, GPIO.RISING, callback = V_g, bouncetime = 25)




try:

    while True:
        print("Tgn:", Tgn)
        print("Trt:", Trt)
        if (Trt > 0):
            GPIO.output(12, GPIO.HIGH)
            time.sleep(4)
            GPIO.output(12, GPIO.LOW)
            Trt = 0

        if (Tgn < 1):                            #Mittelwertbildung aus 15 Werten zur Speicherung, LCD-Ausgabe alle 4 sec.
            Endzeit = time.time()
            delta = (Endzeit - Startzeit)/60/60  # Zeit in Stunden seit Versuchsstart
            cpu = CPUTemperature()
            cput = float(cpu.temperature)
            #bme()                                #BME-Werte abfragen
            #ads()                                # ADS-Sensorwerte abfragen
            DS1820()                             #1W-Werte abfragen
            #GPIO.output(26, GPIO.HIGH) # ON
            txt1()                               #LCD-Ausgabe von ADS und Temp
            Nokia()

            Datum=time.strftime("%Y-%m-%d %H:%M:%S")
            print(time.strftime("%Y-%m-%d %H:%M:%S") + " t: " + str(delta) +  ' Sensor' + str(1) + ': ' + str(T1) + '  ' + '    Sensor' +  str(2) + ':' + str(T2) +  "    Vg:" + str(V))
            print()

            if z1 > 15:
                #y1m = sum(y1m)/5
                #y2m = sum(y2m)/5
                #y3m = sum(y3m)/5
                z1 = 0

                fobj_out = open(Dateiname,"a" )
                #bme_temp_m = str(bme_temp_m)
                #bme_pressure_m = str(bme_pressure_m)
                #bme_humidity_m = str(bme_humidity_m)


                fobj_out.write(Datum + " , " + str(delta) + " , "  +  str(T1) +  ' , ' + str(T2) + " , " + str(V) + " , " + str(cput) + '\n' )
                fobj_out.close()
                y1m = []
                y2m = []
                y3m = []


            else:
                #y1m.append(bme_temp_m)
                #y2m.append(bme_pressure_m)
                #y3m.append(bme_humidity_m)
                z1 = z1 + 1

            #Messwerte an Listen für späteren Graphen anhängen
            #x1.append(delta)
            #y1.append(bme_temp)
            #y2.append(bme_pressure)
            #y3.append(bme_humidity)
            #y4.append(T1)
            #y5.append(T2)
            #y6.append(A0max)
            #y7.append(A1max)
            #y8.append(A2max)
            #y9.append(Vg)
            #y10.append(cput)

            # hier kann spaltenweise aus Dateien auslesen
            #x1=np.genfromtxt(Dateiname,skip_header=3,usecols=(3))
            #y1=np.genfromtxt(Dateiname,skip_header=3,usecols=(5))
            #y2=np.genfromtxt(Dateiname,skip_header=3,usecols=(7))
            #y3=np.genfromtxt(Dateiname,skip_header=3,usecols=(9))
            #y4=np.genfromtxt(Dateiname,skip_header=3,usecols=(11))
            #y5=np.genfromtxt(Dateiname,skip_header=3,usecols=(13))
            #y6=np.genfromtxt(Dateiname,skip_header=3,usecols=(15))
            #y7=np.genfromtxt(Dateiname,skip_header=3,usecols=(17))
            #y8=np.genfromtxt(Dateiname,skip_header=3,usecols=(19))
            #y9=np.genfromtxt(Dateiname,skip_header=3,usecols=(21))
            #y10=np.genfromtxt(Dateiname,skip_header=3,usecols=(23))
            
            time.sleep(0.01)



        else:
            print()
            print("reboot?")
            GPIO.output(12, GPIO.HIGH)
            txt2()
            Nokia()
            time.sleep(4)
            print("Trt",Trt)

            if (Trt > 3):
                GPIO.output(12, GPIO.HIGH)
                print('shutdown!!!')
                print("Trt: " + str(Trt))
                txt5()                                 #LCD-Ausgabe Trt Tgn und reboot
                Nokia()
                subprocess.call('/home/pi/ACC/shutdown.sh')
                time.sleep(4)
            
            if (Trt > 1):
                t = threading.Thread(target=TGZ_sim1)
                t.start()
                time.sleep(2)
                GPIO.output(12, GPIO.HIGH)
                print('reboot')
                print("Trt: " + str(Trt))
                txt4()                                 #LCD-Ausgabe Trt Tgn und reboot
                Nokia()
                time.sleep(4)
                subprocess.call('/home/pi/ACC/reboot.sh')
                time.sleep(4)


            else:
                Trt = 0
                Tgn = 0
                GPIO.output(12, GPIO.LOW)
       
        

except KeyboardInterrupt:
    print("keyboardInterrupt")
    timestr = time.strftime("%Y%m%d_%H%M%S")
    d = open(Dateiname, "a")
    d.write("KeyboardInterrupt at: " + timestr + ": Vgas:" + str(Vg) + "\n")
    d.close()
    GPIO.cleanup()
    print("\nBye")
    sys.exit()








