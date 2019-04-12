#!/usr/bin/env python
#TEST COMPASS WORKING: sudo python compasstest.py 
#TEST GPS WORKING: sudo python gpstest.py

from __future__ import print_function
import os
import sys
import argparse
import time
import Adafruit_LSM303
import math
import time
from datetime import datetime, date
import serial
import string
import pynmea2
import RPi.GPIO as gpio
from Adafruit_LSM303DLHC import LSM303DLHC
from subprocess import call
import shlex




# Create a LSM303 instance.
#lsm303 = Adafruit_LSM303.LSM303()
lsm303 = LSM303DLHC(0x19, 0x1E, False)

#TODO: function to get GPS coords from GPS Module in raspberry pi
#currently returns test data
def get_coordinates():
    gpio.setmode(gpio.BCM)
    port = "/dev/ttyAMA0" # the serial port to which the pi is connected.
    #create a serial object
    ser = serial.Serial(port, baudrate = 9600, timeout = 0.5)
#now reading from serial object
    while 1: #infinite loop
        try: #to get data
                data = ser.readline()  #reads serial line. line 35: if data does not = none and
        except: #if it doesnt happen
                print("loading")
                #wait for the serial port to churn out data
        if data != None and data[0:6] == '$GPGGA': # the long and lat data are always contained
                #in the GPGGA string of the NMEA data, #first 6 chars of data equal to gpgga will
                #go to line 20 goes inside, give lat/lon
                msg = pynmea2.parse(data)
                print(msg)
                #parse the latitude and print (using a library) store in message object
                latval = 0
                try:
                    latval = float(msg.lat)/100 #take lat from message object, convert to float, turn into numeric form
                except:
                    print("gps not ready")
                    continue
                concatlat = "lat:" + str(latval) #rn a float, turns into a string, cant concat
                #parse the longitude and print
                longval = float(msg.lon)/100
                concatlong = "long:"+ str(longval)
                print (concatlong)
                print (concatlat)
                print(msg.lat_dir) #https://github.com/Knio/pynmea2
                if msg.lat_dir == "S":
                    latval = latval * -1
                print(msg.lon_dir)
                if msg.lon_dir == "W":
                    longval = longval * -1
                time.sleep(0.5)#wait a little before picking the next data.
                return [latval, longval]
        
    #TODO: function which get degrees to qibla from current GPS coordinates
    #currently returns test data
#def get_qibla_degrees(coords):
    #lat = coords[0]
    #lon = coords[1]
def get_qibla_degrees(coords): #equation function
    print("Inside Qibla degrees")
    coords = [coords[0]*(math.pi/180), coords[1]*(math.pi/180)] #added this to equasian.py and https://github.com/Knio/pynmea2 for +/- coordinate direction
    Xp = coords[0] #lat of current data
    Yp = coords[1] #lon of current data
    #get qibla degrees for above lat lon
    Xm = 21.3891*(math.pi/180) #poi, convert to radians latitude Mecca
    Ym = 39.8579*(math.pi/180) #poi, convert to radians longitude Mecca
    a = math.sin(Ym-Yp) #sin(lon Mecca- lon data)
    b = math.cos(Xp) #cos(lat data)
    c = math.tan(Xm) #tan(lat Mecca)211
    d = math.sin(Xp) #sin(lat data)
    e = math.cos(Ym-Yp) #cos(lon Mecca-lon data)
    f = b*c 
    g = d*e
    qibla_degrees = math.atan2((a),(f-g)) #eqn to solve angle to Mecca from tru north
    
    return math.degrees(qibla_degrees) #math.degrees converts rads to degs

    coords = [Xp*(math.pi/180), Yp*(math.pi/180)] #converts lat data, lon data into radians
    qibla_degrees = get_qibla_degrees(coords) 
    print(qibla_degrees)
    
    #get qibla degrees for above lat lon

    #function to get current compass direction LSM303 Module in raspberry pi
    #Accel X=-59, Accel Y=4, Accel Z=1013, Mag X=150, Mag Y=-176, Mag Z=-1127
    #currently returns test data
#def get_compass_direction():
    #print("Inside Compass direction")
    # Read the X, Y, Z axis acceleration values and print them.
    #accel, mag = lsm303.read()
    # Grab the X, Y, Z components from the reading and print them out.
    #accel_x, accel_y, accel_z = accel
    #mag_x, mag_y, mag_z = mag
    #return [accel_x, accel_y, accel_z, mag_x, mag_y, mag_z]

#def get_heading(direction):
    #print("Inside Get Heading")
    #magX = direction[4]*1.0
    #magY = direction[3]*1.0

    #print("magX: %s, magY: %s"%(magX, magY))
    #heading = (180 * math.atan2(magY,magX)*1.0)/math.pi*1.0
    #if heading < 0:
     #   heading += 360
    #heading = lsm303.readMagneticHeading() 
	 
    #print("Heading:")
    #print(heading)
    #return heading
     
def get_compass_heading():
    lsm = LSM303DLHC(0x19, 0x1E, False)
    time.sleep(0.25)
    accel = lsm.readAccelerationsG()
    #mag gain here
    lsm.setMagnetometerRange(1.3)
    #
    #mag data rate
    lsm.setMagDataRate(.75)
    #
    mag = lsm.readMagneticsGauss()
    temp = lsm.readTemperatureCelsius()
    heading = lsm.readMagneticHeading()
    if heading < 0:
        heading += 360
    print ("Timestamp: %s" % datetime.now().isoformat()) #strftime('%Y-%m-%dT%H:%M:%S(%Z)')
    print ("Accel X: %6.3f G,     Y: %6.3f G,     Z: %6.3f G" % (accel.x, accel.y, accel.z))
    #print "Mag   X: %6.3f gauss, Y: %6.3f gauss, Z: %6.3f gauss" % (mag.x, mag.y, mag.z)
    print ("Mag   X: %6.3f gauss, Y: %6.3f gauss, Z: %6.3f gauss" % (mag.x, mag.z, mag.y))
    print ("Temp:    %6.3f C" % (temp))
    print ("Heading: %6.3f" % (heading))
    return heading

	
def get_final_heading(qibla_degrees, heading):
    
	if (((qibla_degrees > 0) and (heading > 0)) or ((qibla_degrees < 0) and (heading < 0))):
		return (qibla_degrees - heading)
	else:
		angle = qibla_degrees + heading
		if(qibla_degrees > 0):
                    return angle
		else:
                    return (-1*angle)
import geomag
def get_declination(coords):
    declination = geomag.declination(coords[0], coords[1])
    return declination

coords = []

def get_gps():
    global coords
    gps_mode = raw_input("Please chose a gps mode:\n1.Preprogrammed Data\n2.Real GPS Data\n>")
    if gps_mode == "2":
	    #get current GPS coordinates from GPS Module
	    coords = get_coordinates()#return value from get coordinates and send to coords
	    print("Current coords %s"%(coords,))
    if gps_mode == "1":
            coords = [40.566350, -74.388440]

def calibrate():
    lsm = LSM303DLHC(0x19, 0x1E, False)
    xMin=1000
    xMax=-1000
    yMin=1000
    yMax=-1000
    zMin=1000
    zMax=-1000
    bias=[0,0,0]
    rage=[0,0,0]
    for i in range(0,5000):
        mag=lsm.readMagneticsGauss()
        print("X: ",mag.x,"Y: ",mag.y,"Z: ",mag.z)
        if(mag.x<xMin):
            xMin=mag.x
        if(mag.x>xMax):
            xMax=mag.x
        if(mag.y<yMin):
            yMin=mag.y
        if(mag.y>yMax):
            yMax=mag.y
        if(mag.z<zMin):
            zMin=mag.z
        if(mag.z>zMax):
            zMax=mag.z
    bias[0]=(xMax+xMin)/2
    bias[1]=(yMax+yMin)/2
    bias[2]=(zMax+zMin)/2
    #range[0]=(xMax-xMin)/2
    #range[1]=(yMax-yMin)/2
    #range[2]=(zMax-zMin)/2
    lsm.setOffset(bias[0],bias[1],0)

def main():
    print("Starting now")
    
    #get current compass direction from LSM303 Module
    direction = get_compass_heading()
    #heading = get_heading(direction)
    print("Current compass direction %s"%(direction,))
    
    #get declination
    declination = get_declination(coords)
    print("current declination: %s"%(declination,))
    
    #heading with declination added
    new_heading = direction - declination
    print("new heading(with declination): %s"%(new_heading,))
    
    #get degrees to qibla from current GPS coordinates
    qibla_degrees = get_qibla_degrees(coords)
    print("Qibla Degrees from input coords: %s is: %s"%(coords, qibla_degrees))
	
    final_heading= get_final_heading(qibla_degrees, new_heading)
    print("Final angle = ",final_heading)
    return '{"qibla_degrees": '+ str(qibla_degrees) + ',"compass_degrees": ' + str(new_heading) + ',"final_heading": ' + str(final_heading) + "}"
#!/usr/bin/env python
"""
Very simple HTTP server in python.
Usage::
    ./dummy-web-server.py [<port>]
Send a GET request::
    curl http://localhost
Send a HEAD request::
    curl -I http://localhost
Send a POST request::
    curl -d "foo=bar&bin=baz" http://localhost
"""
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import SocketServer

class S(BaseHTTPRequestHandler):
    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
	if self.path.endswith("compass.png"):
            f = open("compass.png") #self.path has /test.html
#note that this potentially makes every file on your computer readable by the internet

            self.send_response(200)
            self.send_header('Content-type',    'image/png')
            self.end_headers()
            self.wfile.write(f.read())
            f.close()
            return
	if self.path.endswith("qibla_pointer_white.png"):
            f = open("qibla_pointer_white.png") #self.path has /test.html
#note that this potentially makes every file on your computer readable by the internet

            self.send_response(200)
            self.send_header('Content-type',    'image/png')
            self.end_headers()
            self.wfile.write(f.read())
            f.close()
            return
        if not self.path.endswith("info"):
            f = open("qibla.html") #self.path has /test.html
#note that this potentially makes every file on your computer readable by the internet

            self.send_response(200)
            self.send_header('Content-type',    'text/html')
            self.end_headers()
            self.wfile.write(f.read())
            f.close()
            return
        output = main()
        self._set_headers()
        self.wfile.write(output)

    def do_HEAD(self):
        self._set_headers()
        
    def do_POST(self):
        # Doesn't do anything with posted data
        self._set_headers()
        self.wfile.write("<html><body><h1>POST!</h1></body></html>")
        
def run(server_class=HTTPServer, handler_class=S, port=80):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print('Starting httpd...')
    httpd.serve_forever()

if __name__ == "__main__":
    from sys import argv
    print("Getting GPS Coordinates first time")
    get_gps()
    #mode = raw_input("Press 1 for Calibrating Compass\n>")
    #if mode == "1":
    #    calibrate()
    mode = raw_input("Please chose a mode:\n1.Test-Mode\n2.Server Mode\n>")
    if mode == "2":
        if len(argv) == 2:
            run(port=int(argv[1]))
        else:
            run()
    if mode == "1":
        main()
    
#if __name__ == '__main__':
#    sys.exit(main(sys.argv[1:]))









"""
Starting now
long:74.235949
lat:40.3499625
N
W
Current coords [40.349962500000004, -74.235949]
Inside get_compass_heading
Timestamp: 2018-11-16T18:09:37.550404
Accel X: -0.088 G,     Y: -0.036 G,     Z:  1.008 G
Mag   X:  3.065 gauss, Y:  3.315 gauss, Z: -3.453 gauss
Temp:    18.000 C
Heading: 47.244
Current compass direction 47.2439843795
Inside Qibla degrees
Qibla Degrees from input coords: [40.349962500000004, -74.235949] is: 58.3452239453
Final angle =  11.1012395658
"""
#Question 1: what degree you facing based on above? 47.243
#Question 2: what is the direction of the qibla in degrees based on your current location? 58.3
#Question 3: How many degrees should the user turn to face the qibla from his current direction? About 11 degrees





























