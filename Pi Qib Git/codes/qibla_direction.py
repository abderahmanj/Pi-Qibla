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
import serial
import string
import pynmea2
import RPi.GPIO as gpio

# Create a LSM303 instance.
lsm303 = Adafruit_LSM303.LSM303()

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
                #parse the latitude and print (using a library) store in message object
                latval = float(msg.lat)/100 #take lat from message object, convert to float, turn into numeric form
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
    coords = [coords[0]*(math.pi/180), coords[1]*(math.pi/180)] #added this to equasian.py and https://github.com/Knio/pynmea2 for +/- coordinate direction
    Xp = coords[0] #lat of current data
    Yp = coords[1] #lon of current data
    #get qibla degrees for above lat lon
    Xm = 21.3891*(math.pi/180) #poi, convert to radians latitude Mecca
    Ym = 39.8579*(math.pi/180) #poi, convert to radians longitude Mecca
    a = math.sin(Ym-Yp) #sin(lon Mecca- lon data)
    b = math.cos(Xp) #cos(lat data)
    c = math.tan(Xm) #tan(lat Mecca)
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
def get_compass_direction():
    # Read the X, Y, Z axis acceleration values and print them.
    accel, mag = lsm303.read()
    # Grab the X, Y, Z components from the reading and print them out.
    accel_x, accel_y, accel_z = accel
    mag_x, mag_y, mag_z = mag
    return [accel_x, accel_y, accel_z, mag_x, mag_y, mag_z]

def get_heading(direction):
    #magX = direction[4]*1.0
    #magY = direction[3]*1.0

    #print("magX: %s, magY: %s"%(magX, magY))
    #heading = (180 * math.atan2(magY,magX)*1.0)/math.pi*1.0
    #if heading < 0:
    #    heading += 360
    heading = lsm.readMagneticHeading() 
	 
    print("Heading:")
    print(heading)
	return heading
	
def get_final_heading
	if (qibla_degrees > 0 && heading > 0)||(qibla_degrees < 0 && heading < 0):
		return angle = qibla_degrees - heading
	else:
		angle = qibla_degrees + heading
		if(qibla_degrees > 0)
		return angle
		else
		return (-1*angle)

def main(arguments):

   #get current GPS coordinates from GPS Module
    coords = get_coordinates()#return value from get coordinates and send to coords
    print("Current coords %s"%(coords,))
    
    #get current compass direction from LSM303 Module
    direction = get_compass_direction()
    heading = get_heading(direction)
    print("Current compass direction %s"%(direction,))
    
    #get degrees to qibla from current GPS coordinates
    qibla_degrees = get_qibla_degrees(coords)
    print("Qibla Degrees from input coords: %s is: %s"%(coords, qibla_degrees))
	
	final_heading= get_final_heading()
	print("Final angle = ",final_heading)


    
    
if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
