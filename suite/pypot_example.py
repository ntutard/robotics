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
from indirect import *
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
    time.sleep(0.30)
    
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
        
## FONCTIONS UTILES ##

def moveLeg(leg, x, y, z):
    oldPos = []
    for m in leg :
        oldPos.append(m.goal_position)

    co = leg_dk(oldPos[0], oldPos[1], oldPos[2])
    co[0] += x
    co[1] += y
    co[2] += z

    tmp = leg_ik(co[0],co[1],co[2])
    i = 0
    for m in leg :
        if (abs(tmp[i]) < 130):
            m.goal_position = tmp[i]
        i += 1          

def setLeg(spider_robot,x,y,z):
    tmp = leg_ik(x,y,z)
    i = 0
    for m in leg :
        if (abs(tmp[i]) < 130):
            m.goal_position = tmp[i]
        i += 1

def moveLegRepere(sr, leg, x, y, z):
    if (sr.leg1 == leg) :
        moveLeg(leg, x, y, z)
    elif (sr.leg2 == leg) :
        moveLeg(leg, -x, -y, z)
    elif (sr.leg31 == leg or sr.leg32 == leg):
        moveLeg(leg, -y, x, z)
    else:
        moveLeg(leg, y, -x, z)
    
def move(spider_robot, x, y, z) :
    moveLeg(spider_robot.leg1, x, y, z)
    moveLeg(spider_robot.leg2, -x, -y, z)
    moveLeg(spider_robot.leg31, -y, x, z)
    moveLeg(spider_robot.leg32, -y, x, z)
    moveLeg(spider_robot.leg41, y, -x, z)
    moveLeg(spider_robot.leg42, y, -x, z)

def moveTwoLegs(leg1, leg2, x, y, z):
    moveLeg(leg1, x, y, z)
    moveLeg(leg2, x, y, z)

def upAndDownLeg(leg):
    moveLeg(leg, 0, 0, 10)
    time.sleep(0.35)
    for m in leg :
        m.goal_position = 0

def rotate2(spider_robot) :
    waitTime = 0.35
    moveY = -50

    # First phase
    
    moveTwoLegs(spider_robot.leg1, spider_robot.leg2, 0, 0, 20)    
    moveTwoLegs(spider_robot.leg32, spider_robot.leg41, 0,0,-10)
    moveTwoLegs(spider_robot.leg42, spider_robot.leg31, 0,0,20)
    time.sleep(waitTime)

    moveTwoLegs(spider_robot.leg32, spider_robot.leg41, 0,moveY,0)
    moveTwoLegs(spider_robot.leg42, spider_robot.leg31, 0,moveY,0)
    
    time.sleep(waitTime)

    moveTwoLegs(spider_robot.leg32, spider_robot.leg41, 0,0,10)
    moveTwoLegs(spider_robot.leg42, spider_robot.leg31, 0,0,-20)
    moveTwoLegs(spider_robot.leg1, spider_robot.leg2, 0, 0, -20)
    time.sleep(waitTime)

    # Second phase

    moveTwoLegs(spider_robot.leg32, spider_robot.leg41, 0,0,20)
    time.sleep(waitTime)
    
    moveTwoLegs(spider_robot.leg32, spider_robot.leg41, 0,-moveY,0)


    time.sleep(waitTime)

    moveTwoLegs(spider_robot.leg32, spider_robot.leg41, 0,0,-20)
    time.sleep(waitTime)
    moveTwoLegs(spider_robot.leg42, spider_robot.leg31, 0,0,20)
    time.sleep(waitTime)
    moveTwoLegs(spider_robot.leg42, spider_robot.leg31, 0,-moveY,0)
    time.sleep(waitTime)
    moveTwoLegs(spider_robot.leg42, spider_robot.leg31, 0,0,-20)
    time.sleep(waitTime)

    
def setInit(spider_robot) :
    for m in spider_robot.thirdMotors :
            m.compliant = False
            m.goal_speed = 150
            m.goal_position = 0
    for m in spider_robot.secondMotors :
            m.compliant = False
            m.goal_speed = 150
            m.goal_position = 0
    for m in spider_robot.firstMotors :
            m.compliant = False
            m.goal_speed = 150
            m.goal_position = 0


def moveThreeLegs(leg1, leg2, leg3, x, y, z):
    moveLeg(leg1, x, y, z)
    moveLeg(leg2, x, y, z)
    moveLeg(leg3, x, y, z)

def moveLegsRepere(sr, leg, moves, waitTime):
    i = 0
    for m in moves:
        for l in leg:
            moveLegRepere(sr, l, moves[i][0], moves[i][1], moves[i][2])
        time.sleep(waitTime)
        i += 1
    
def spiderWalk(beginWalk, sr):
    waitTime = 0.25
    legs1 = [sr.leg1, sr.leg31, sr.leg41]
    legs2 = [sr.leg2, sr.leg32, sr.leg42]

    coefX = 40
    coefZ = 15
    
    monter=[0,0,coefZ]
    doubleMonter = [0, 0, coefZ * 2]
    descendre=[0,0,-coefZ]
    doubleDescendre=[0, 0, -(coefZ * 2)]
    avancer=[-coefX, 0, 0]
    reculer=[coefX, 0, 0]

    if (beginWalk):
        moveLegsRepere(sr, legs2, [monter], waitTime)
        
    moveLegsRepere(sr, legs1, [avancer], waitTime)
    moveLegsRepere(sr, legs2, [doubleDescendre], waitTime)
    moveLegsRepere(sr, legs1, [reculer], waitTime)
    moveLegsRepere(sr, legs2, [doubleMonter], waitTime)

def scorpionWalk(beginWalk, forward, sr):
        if (forward):
                coefX = 30
        else:
                coefX = -30

        coefZ = 30
        waitFactor = 1.2

        sr.motor_42.goal_position = -50
        sr.motor_43.goal_position = 120

        if (beginWalk):
                moveLegsRepere(sr, [sr.leg42, sr.leg31], [[0, 0, coefZ]], 0)
                moveLegsRepere(sr, [sr.leg32, sr.leg41], [[-coefX, 0, 0]], 0.15*waitFactor)
                moveLegsRepere(sr, [sr.leg42, sr.leg31], [[0, 0, -coefZ]], 0.05*waitFactor)

        moveLegsRepere(sr, [sr.leg32, sr.leg41], [[0, 0, coefZ]], 0)
        moveLegsRepere(sr, [sr.leg42, sr.leg31], [[-coefX, 0, 0]], 0.05*waitFactor)
        moveLegsRepere(sr, [sr.leg32, sr.leg41], [[coefX, 0, 0]], 0.10*waitFactor)
        moveLegsRepere(sr, [sr.leg32, sr.leg41], [[0, 0, -coefZ]], 0.05*waitFactor)

        sr.motor_42.goal_position = -70
        sr.motor_43.goal_position = 135

        moveLegsRepere(sr, [sr.leg31, sr.leg42], [[0, 0, coefZ]], 0)
        moveLegsRepere(sr, [sr.leg41, sr.leg32], [[-coefX, 0, 0]], 0.05*waitFactor)
        moveLegsRepere(sr, [sr.leg31, sr.leg42], [[coefX, 0, 0]], 0.10*waitFactor)
        moveLegsRepere(sr, [sr.leg31, sr.leg42], [[0, 0, -coefZ]], 0.05*waitFactor)
               
def spiderMode(sr):
        setInit(sr)

        time.sleep(1)
        
        move(sr, 0, 0, 35)

        moveLegsRepere(sr, [sr.leg32, sr.leg42], [[50, 0, 0]], 0)
        moveLegsRepere(sr, [sr.leg31, sr.leg41], [[-50, 0, 0]], 0)

def scorpionMode(sr):
        setInit(sr)

        time.sleep(2)
                
        sr.motor_12.goal_position = -60
        sr.motor_13.goal_position = 15
        sr.motor_42.goal_position = -70
        sr.motor_43.goal_position = 135

        time.sleep(0.5)

        moveLegsRepere(sr, [sr.leg32, sr.leg42], [[70, 0, 40]], 0)
        moveLegsRepere(sr, [sr.leg31, sr.leg41], [[-70, 0, 40]], 0)

        time.sleep(0.5)
        sr.motor_42.goal_speed = 40
        sr.motor_43.goal_speed = 35
        sr.motor_42.goal_position = -70
        sr.motor_43.goal_position = 135
       
        
        
if __name__ == '__main__':

        state = "waiting"

        #with pypot.dynamixel.DxlIO('/dev/ttyUSB0', baudrate=1000000) as dxl_io:
        #        found_ids = dxl_io.scan()  # this may take several seconds
        #        changeId(found_ids)

        print ("Initialisation du robot")
        spider_robot=from_json('spider_robot.json')
        scorpionMode(spider_robot)
        time.sleep(1)

        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        ch = ''

        while (ch != 'q' and ch != 'Q'):
                try:
                        tty.setraw(sys.stdin.fileno())  
                        ch = sys.stdin.read(1)
                finally:
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

                        if (ch == 'z' or ch == 'Z'):
                                print("Forward Walking")
                                if (state == "forwardWalking"):
                                        scorpionWalk(False, True, spider_robot)
                                else:
                                        scorpionWalk(True, True, spider_robot)
                                        state = "forwardWalking"
                                print("End Walking")
                        elif (ch == 's' or ch == 'S'):
                                print("Backward Walking")
                                #if (state == "backWardWalking")
                                        

        print("Deconnexion")
        
        spider_robot.close()


        
