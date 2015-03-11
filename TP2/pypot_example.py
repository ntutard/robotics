import itertools
import time
import numpy
import json
import pypot.dynamixel
import pypot.dynamixel.conversion
import pypot.robot.robot
from pypot.dynamixel import autodetect_robot
from pypot.robot import from_json
from math import cos, sin, acos, asin
import math

def leg_dk(T1,T2,T3,L1 = 51,L2 = 63.7, L3 =93, a=None, b=None ):
    if a == None:
        a = asin(22.5/L2)
    if b == None:
        b = asin(8.2/L3)
        
    T1f=math.radians(T1)
    T2f=-math.radians(T2)-a
    T3f=math.radians(T3) + b - math.radians(90) + a
    return [cos(T1f) * (L1 + L2*cos(T2f) + L3*cos(T3f+T2f) ),
          sin(T1f) * (L1 + L2*cos(T2f) + L3*cos(T3f+T2f)),
          L2 * sin(T2f) + L3*sin(T3f+T2f)]

def initialisation(ids):
    dxl_io.enable_torque(found_ids)
    initPos = dict(zip(ids, itertools.repeat(0)))
    dxl_io.set_goal_position(initPos)

    print("Waiting motor movement")
        
    for i in ids:
        inited = False
        while not inited:
            if (abs(dxl_io.get_present_position([i])[0]) < 1):
                inited = True
                
    dxl_io.disable_torque(found_ids)
    print("Ready ")

def interchangeId(ids, id1, id2):
    if (id1 == id2):
        print "Already has the good id"
        return ids
    
    newIds = dict()

    if (id2 in ids):
        unusedId = 0
        for i in range(254):
            if (i not in ids):
                newIds[id2] = i
                unusedId = i
                break

        dxl_io.change_id(newIds)

        newIds = dict()
        newIds[id1] = id2
        dxl_io.change_id(newIds)

        newIds = dict()
        newIds[unusedId] = id1
        dxl_io.change_id(newIds) 
    else:

        newIds[id1] = id2
        dxl_io.change_id(newIds)

    for i in range(len(ids)):
        if (ids[i] == id1):
            ids[i] = id2
        elif (ids[i] == id2):
            ids[i] = id1

    print "Motor ", id1, " changed to ", id2
        
    return ids

def changeId(ids):
    
    for i in range(len(ids)):
        idNotGiven = True
        initialisation(ids)
        while (idNotGiven):
            pos = dxl_io.get_present_position(ids)
            for x in range(len(pos)):
                if (abs(pos[x]) > 20):
                    print("Enter id for this motor :")
                    tmp = int( input() )
                    ids = interchangeId(ids, ids[x], tmp )
                    idNotGiven = False
                    print("Motor validated !")
                    raw_input('->')
                    break;

            time.sleep(0.02)
            
    initialisation(ids)
    print(ids)
            
def suivreSinusoide(ids, frequence,amplitude,temps):
    t0=time.time()
    t=0
    while(t<temps):
        t=time.time()-t0
        pos = amplitude * numpy.sin(2 * numpy.pi * frequence * t)
        posArray = dict(zip(ids, itertools.repeat(pos)))
        dxl_io.set_goal_position(posArray)
        time.sleep(0.02)
        
        

if __name__ == '__main__':

    # we first open the Dynamixel serial port
##    with pypot.dynamixel.DxlIO('/dev/ttyUSB0', baudrate=1000000) as dxl_io:
##
##        # we can scan the motors
##        found_ids = dxl_io.scan()  # this may take several seconds
##     #   print 'Detected:', found_ids
##      #  print('Do you want to change all the ids (1 to change 0 to not change) :')
##
##       # answer=input()
##        #if int(answer) == 1:
##        changeId(found_ids)

          #  yesfound_ids = dxl_io.scan()  # this may take several seconds
           # print 'Detected:', yesfound_ids
        
##        spider_robot=autodetect_robot()
##        config = spider_robot.to_config()
##        with open('spider_robot.json', 'wb') as f:
##            json.dump(config, f)
##        spider_robot.close()

        
           
        spider_robot=from_json('spider_robot.json')
        print "petit message"
        for m in spider_robot.thirdMotors :
            m.compliant = False
            m.goal_speed = 150
            m.goal_position = 0
        for m in spider_robot.secondMotors :
            m.compliant = False
            m.goal_speed = 100
            m.goal_position = 0
        for m in spider_robot.firstMotors :
            m.compliant = False
            m.goal_speed = 100
            m.goal_position = 0
            
        time.sleep(2)
        t0 = time.time()
        while (True):
            t = math.sin(2 * math.pi * 0.5 * time.time() - t0) - 0.5
            if (t < 0):
                spider_robot.motor_12.goal_position =  -60 * abs(t)
                if (abs(t) > 0.5):
                    spider_robot.motor_13.goal_position = 150 * (abs(t) - 0.5)
                time.sleep(0.02)
            else:
                spider_robot.motor_42.goal_position =  -60 * abs(t)
                if (abs(t) > 0.5):
                    spider_robot.motor_43.goal_position = 150 * (abs(t) - 0.5)
                time.sleep(0.02)
        

        spider_robot.close()






        
        # we power on the motors
        #dxl_io.enable_torque(found_ids)

        # we get the current positions
        #print 'Current pos:', dxl_io.get_present_position(found_ids)

        # # we create a python dictionnary: {id0 : position0, id1 : position1...}
        # pos = dict(zip(found_ids, itertools.repeat(0)))
        # print 'Cmd:', pos
        # # we send these new positions
        # dxl_io.set_goal_position(pos)
        # time.sleep(1)  # we wait for 1
    
        #changeId(found_ids)
        #suivreSinusoide(found_ids,0.5,30,15)
        # we get the current positions
       # print 'New pos:', dxl_io.get_present_position(found_ids)
##        dxl_io.disable_torque(found_ids)
        #pos = dict(zip(found_ids, [0,30,-67]))
        #dxl_io.set_goal_position(pos)
        #dxl_io.disable_torque(found_ids)
##        while True :
##                angle=dxl_io.get_present_position(found_ids)
##                print 'Position :',leg_dk(angle[0],angle[1],angle[2])
##                time.sleep(0.5)
            
        # we power off the motors
        
        time.sleep(1)  # we wait for 1s

        
