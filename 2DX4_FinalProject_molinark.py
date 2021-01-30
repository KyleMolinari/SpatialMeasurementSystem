'''
Kyle Molinari
400136596
2DX4 Final Project

Using Python 3.6.5
Need to pyserial, open3d, numpy, math, and sys libraries
'''

import serial
import math
import numpy
import open3d
import sys

angleOffset = 0 #starting angle in degrees relative to motor position (just corrects for staring orientation of motor)
distanceOffset = 10 #offset of the VL53L1X sensor from the axis of rotation (in millimeters)
s = serial.Serial("COM6", 115200) 
print("Opening: " + s.name)
lines = []


def initialize():
    s.write(b'1') #write 1 to serial port to indicate that its ready to read data
    start = 0 #initialize start flag to 0
    #ignore all serial data (eg startup print statements) until the step angle is given followed by a comma
    while(start == 0):
        firstLine = s.readline().decode().replace(',','') #read serial data but take out any commas
        try: #if the serial data is a number then that means it is the step angle
            stepAngle = int(firstLine)
            start = 1 #set start flag to 1
        except: #if the serial data is not a number
            print(firstLine)

    getData(stepAngle)

def getData(angle):
    f = open("tof_radar.xyz", "w") #create a xyz file to write to
    z = 0     #initial z position
    i = 0
    revs = 0
    global lines
    
    while(i<int(360/angle)):
        
        x = s.readline()        # read one line
        c = x.decode()      # convert to str
        
        if(c != c.replace(',','')): ##if there is a comma in the string that means that it is giving the step angle
            angle = int(c.replace(',','')) #make sure step angle is updated
            x = s.readline() #read the next string
            c = x.decode()
        
        if c=="restart\n": #if emergency stop button is pressed
            print("Goodbye\n")
            f.close() #close xyz file
            sys.exit() #stop the program
        else:
            distance = int(c)+distanceOffset #get serial data and set it as distance. add the distanceOffset to adjust for hardware limitations
            #use unit vector direction & distance to get x and y position
            x=-distance*math.sin(i*math.radians(angle)+math.radians(angleOffset))
            y=distance*math.cos(i*math.radians(angle)+math.radians(angleOffset))
            
            
            data = str(x)+" "+str(y)+" "+str(z)+"\n"
            f.write(data) #write coordinates to xyz file
            print(data)
            ####lines.append([i, (i+1)%(360/angle)])
            if(i == int(360/angle)-1): #once full revolution is completed
                revs = revs + 1
                keepCollecting = input("Press enter to end program and see 3D plot. If you would like to keep collecting data, enter the change in Z coordinate from the last measurement in mm.\n")

                try: #if a number is entered
                    z = z + int(keepCollecting) #update the z coordinate for the next rotation
                    i = -1 #i will be updated to 0 after this if statement and the while loop will run again
                except: #if enter is pressed (or a number is not entered)
                    print("Closing: " + s.name)
                    f.close()
                    visualize(angle, revs)
        i = i + 1
                    
    
    
    
def visualize(angle, revs):
    global lines
    pts = int(360/angle)
    
    #connect lines between pts in each plane
    for i in range(revs): #for each displacement (offset in z direction)
        for j in range(pts): #go through all points in the plane
            if(j+1 == pts): #if its the last point connect it to the first point
                lines.append([j+i*pts, 0+i*pts])
            else: #connect adjacent points in the plane
                lines.append([j+i*pts, j+1+i*pts])

    #connect vertices between planes
    for i in range(revs-1): #go through all planes execpt the last one
        for j in range(pts): #go through all points in that plane              
            lines.append([j+i*pts, j+(i+1)*pts]) #connect the point to the corresponding point in the next plane
    

    pcd = open3d.io.read_point_cloud("tof_radar.xyz", format='xyz')

    line_set = open3d.geometry.LineSet()
    line_set.points = open3d.utility.Vector3dVector(numpy.asarray(pcd.points))
    line_set.lines = open3d.utility.Vector2iVector(lines)
    
    print(pcd)
    print(numpy.asarray(pcd.points))
    open3d.visualization.draw_geometries([line_set]) #mesh visualization
    #open3d.visualization.draw_geometries([pcd]) #point cloud visualization


initialize()

