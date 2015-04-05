import itertools
import time
import numpy
import json
import pypot.dynamixel
import pypot.dynamixel.conversion
import pypot.robot.robot
import signal
import sys
from parseArduino import *
from threadArduino import *
from pypot.dynamixel import autodetect_robot
from pypot.robot import from_json
from math import cos, sin, acos, asin
from indirect import *
import math
DEFAULT_GOAL_SPEED = 100
## print input results
DEBUG_INPUT = True
WALK_MODE="walkmode"
ROTATE_MODE="rotatemode"
ANGLE_ROTATE_MODE="anglerotatemode"
MOVE_TO_MM="flrkgtrk"
FREE_WALK_MODE="freewalkmode"
CENTER_MODE="centermovemode"
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
                    print("Enter id for this motor (previous ="+str(ids[x])+") :")
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

currentLegPositions = {}
isInited = False
spider_robot = None

def moveLeg(leg, x, y, z):
    oldPos = []
    for m in leg :
        oldPos.append(m.goal_position)

    co = leg_dk(oldPos[0], oldPos[1], oldPos[2])
    co[0] += x
    co[1] += y
    co[2] += z

    if (isInited):
        currentLegPositions[getKeyForLeg(spider_robot, leg)][0] += x
        currentLegPositions[getKeyForLeg(spider_robot, leg)][1] += y
        currentLegPositions[getKeyForLeg(spider_robot, leg)][2] += z

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

def spiderGoToMM(sr,mmX,mmY):
    maxX=30
    maxY=30
    beginWalk=True
        
    while(abs(mmX)!=0 or abs(mmY) !=0 ):
        valueX=mmX
        valueY=mmY
        if(abs(mmX)>=maxX):
            if(mmX<0):
                mmX+=maxX
                valueX-=maxX
            else:
                mmX-=maxX
                valueX=maxX
        else:
            mmX=0
        if(abs(mmY)>maxY):
            if(mmY<0):
                mmY+=maxY
                valueY-=maxY
            else:
                mmY-=maxY
                valueY=maxY
        else:
            mmY=0
        print valueX,valueY

        spiderWalk(beginWalk,"freewalking",sr,valueX,valueY)
        beginWalk=False
        

def spiderAngleRotate(sr,angleRad, clockSense):
    waitTime = 0.28

    while (angleRad != 0):
        tmp = 0
        if (angleRad > math.pi/12):
            tmp = math.pi/12
            angleRad -= math.pi/12
        elif (-angleRad > math.pi/12):
            tmp = -math.pi/12
            angleRad += math.pi/12
        else:
            tmp = angleRad
            angleRad = 0

        print (tmp * 180 / math.pi)
        moveX = 0
        moveY = 50./20. * (tmp * 180 / math.pi)
        moveZ = 20

        print str(moveX) + " " + str(moveY)

        if (clockSense):
            moveY = -moveY
            
        moveThreeLegs(sr.leg1, sr.leg31, sr.leg41, -moveX, -moveY, moveZ)
        moveThreeLegs(sr.leg2, sr.leg32, sr.leg42, moveX, moveY, 0)
        time.sleep(waitTime)
        moveThreeLegs(sr.leg1, sr.leg31, sr.leg41, 0, 0, -moveZ)
        time.sleep(0.1)
        moveThreeLegs(sr.leg2, sr.leg32, sr.leg42, -moveX, -moveY, moveZ)
        time.sleep(waitTime)
        moveThreeLegs(sr.leg1, sr.leg31, sr.leg41, moveX, moveY, 0)
        moveThreeLegs(sr.leg2, sr.leg32, sr.leg42, 0, 0, -moveZ)
        time.sleep(waitTime)
    
def spiderRotate(sr, clockSense) :
    waitTime = 0.25
    moveY = -50
    moveZ = 20

    if (clockSense):
        moveY = -moveY

    # First phase

    moveThreeLegs(sr.leg1, sr.leg31, sr.leg41, 0, -moveY, moveZ)
    moveThreeLegs(sr.leg2, sr.leg32, sr.leg42, 0, moveY, 0)
    time.sleep(waitTime)
    moveThreeLegs(sr.leg1, sr.leg31, sr.leg41, 0, 0, -moveZ)
    time.sleep(0.1)
    moveThreeLegs(sr.leg2, sr.leg32, sr.leg42, 0, -moveY, moveZ)
    time.sleep(waitTime)
    moveThreeLegs(sr.leg1, sr.leg31, sr.leg41, 0, moveY, 0)
    moveThreeLegs(sr.leg2, sr.leg32, sr.leg42, 0, 0, -moveZ)
    time.sleep(waitTime)
    
def setInit(spider_robot) :
    for m in spider_robot.thirdMotors :
            m.compliant = False
            m.goal_speed = DEFAULT_GOAL_SPEED
            m.goal_position = 0
    for m in spider_robot.secondMotors :
            m.compliant = False
            m.goal_speed = DEFAULT_GOAL_SPEED
            m.goal_position = 0
    for m in spider_robot.firstMotors :
            m.compliant = False
            m.goal_speed = DEFAULT_GOAL_SPEED
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
    
def scorpionFreeWalking(beginWalk,direction,sr,arduinoValue):
    return None ## TODO

def spiderFreeWalk(beginWalk,direction,sr,arduinoValue):
    (coefX,coefY)=coeff(arduinoValue)
    defaultX=40
    defaultY=40
    if(DEBUG_INPUT):
        print coefX,coefY
    spiderWalk(True,"freewalking",sr,coefY*defaultX,coefX*defaultY)

def spiderWalk(beginWalk, direction, sr,coefX=40,coefY=40,coefZ=15):    
    
    if (direction =="forwardwalking"):
        coefX = coefX
        coefY = 0
    elif (direction == "backwardwalking"):
        coefX = coefX * -1
        coefY = 0
    elif direction == "leftwalking":
        coefX = 0
        coefY = coefY
    elif direction == "rightwalking":
        coefX = 0
        coefY = coefY * -1
    elif direction == "forwardrightwalking":
        coefX = coefX
        coefY = coefY *-1
    elif (direction == "forwardleftwalking"):
        coefX = coefX
        coefY = coefY
    elif (direction == "backwardrightwalking"):
        coefX = coefX*-1
        coefY = coefY *-1
    elif (direction == "backwardleftwalking"):
        coefX = coefX*-1
        coefY = coefY
    elif (direction != "freewalking"):
        print("MOVEMENT NOT IMPLEMENTED : "+direction+"\n")
        return
    
    waitTime = 0.15
    legs1 = [sr.leg1, sr.leg31, sr.leg41]
    legs2 = [sr.leg2, sr.leg32, sr.leg42]
    
    monter=[0,0,coefZ]
    doubleMonter = [0, 0, coefZ * 2]
    descendre=[0,0,-coefZ]
    doubleDescendre=[0, 0, -(coefZ * 2)]
    avancer=[-coefX, -coefY, 0]
    reculer=[coefX, coefY, 0]

    if (beginWalk):
        legs = [sr.leg1, sr.leg2, sr.leg31, sr.leg32, sr.leg41, sr.leg42]
        for l in legs:
                posLeg = currentLegPositions[getKeyForLeg(sr, l)]
                tmpX = -posLeg[0]
                tmpY = -posLeg[1]
                tmpZ = -posLeg[2]

                moveLeg(l, tmpX, tmpY, tmpZ)

        time.sleep(0.20)
                
        moveLegsRepere(sr, legs2, [monter], waitTime)
        
    moveLegsRepere(sr, legs1, [avancer], waitTime)
    moveLegsRepere(sr, legs2, [doubleDescendre], waitTime)
    moveLegsRepere(sr, legs1, [reculer], waitTime)
    moveLegsRepere(sr, legs2, [doubleMonter], waitTime)

def stabilizeSpiderAfterWalking(sr):
    legs2 = [sr.leg2, sr.leg32, sr.leg42]
    coefZ = 15

    descendre=[0,0,-coefZ]

    moveLegsRepere(sr, legs2, [descendre], 0.15)

def scorpionWalk(previousState, direction, sr):
        coefX=30
        coefY=30
        
        if (direction =="forwardwalking"):
            coefX = coefX
            coefY = 0
        elif (direction == "backwardwalking"):
            coefX = coefX * -1
            coefY = 0
        elif direction == "leftwalking":
            coefX = 0
            coefY = coefY
        elif direction == "rightwalking":
            coefX = 0
            coefY = coefY * -1
        elif direction == "forwardrightwalking":
            coefX = coefX
            coefY = coefY *-1
        elif (direction == "forwardleftwalking"):
            coefX = coefX
            coefY = coefY
        elif (direction == "backwardrightwalking"):
            coefX = coefX*-1
            coefY = coefY *-1
        elif (direction == "backwardleftwalking"):
            coefX = coefX*-1
            coefY = coefY

        else:
            print("MOVEMENT NOT IMPLEMENTED : "+direction+"\n")
            return 
                
        coefZ = 35
        waitFactor = 1.3
        
        sr.motor_42.goal_position = -50
        sr.motor_43.goal_position = 120
        
        if (previousState != direction):
                legs = [sr.leg1, sr.leg2, sr.leg31, sr.leg32, sr.leg41, sr.leg42]
                for l in legs:
                        posLeg = currentLegPositions[getKeyForLeg(sr, l)]
                        tmpX = -posLeg[0]
                        tmpY = -posLeg[1]
                        tmpZ = -posLeg[2]

                        moveLeg(l, tmpX, tmpY, tmpZ)

                time.sleep(0.20)
                
                moveLegsRepere(sr, [sr.leg42, sr.leg31], [[0, 0, coefZ]], 0)
                moveLegsRepere(sr, [sr.leg32, sr.leg41], [[-coefX, -coefY, 0]], 0.15*waitFactor)
                moveLegsRepere(sr, [sr.leg42, sr.leg31], [[0, 0, -coefZ]], 0.05*waitFactor)
        
        moveLegsRepere(sr, [sr.leg32, sr.leg41], [[0, 0, coefZ]], 0)
        moveLegsRepere(sr, [sr.leg42, sr.leg31], [[-coefX, -coefY, 0]], 0.05*waitFactor)
        moveLegsRepere(sr, [sr.leg32, sr.leg41], [[coefX, coefY, 0]], 0.10*waitFactor)
        moveLegsRepere(sr, [sr.leg32, sr.leg41], [[0, 0, -coefZ]], 0.05*waitFactor)

        sr.motor_42.goal_position = -70
        sr.motor_43.goal_position = 135

        moveLegsRepere(sr, [sr.leg31, sr.leg42], [[0, 0, coefZ]], 0)
        moveLegsRepere(sr, [sr.leg41, sr.leg32], [[-coefX, -coefY, 0]], 0.05*waitFactor)
        moveLegsRepere(sr, [sr.leg31, sr.leg42], [[coefX, coefY, 0]], 0.10*waitFactor)
        moveLegsRepere(sr, [sr.leg31, sr.leg42], [[0, 0, -coefZ]], 0.05*waitFactor)

        time.sleep(0.1 * waitFactor)
            
def spiderMode(sr):
        setInit(sr)

        time.sleep(2)
        
        move(sr, 0, 0, 35)

        moveLegsRepere(sr, [sr.leg32, sr.leg42], [[50, 0, 0]], 0)
        moveLegsRepere(sr, [sr.leg31, sr.leg41], [[-50, 0, 0]], 0)

        legs = [sr.leg1, sr.leg2, sr.leg31, sr.leg32, sr.leg41, sr.leg42]
        for l in legs:
            currentLegPositions[getKeyForLeg(spider_robot, l)] = [0] * 3

def scorpionMode(sr):
        setInit(sr)

        time.sleep(2)
                
        sr.motor_12.goal_position = -60
        sr.motor_13.goal_position = 15
        sr.motor_42.goal_position = -70
        sr.motor_43.goal_position = 135

        time.sleep(0.5)

        moveLegsRepere(sr, [sr.leg32, sr.leg42], [[70, 0, 30]], 0)
        moveLegsRepere(sr, [sr.leg31, sr.leg41], [[-70, 0, 30]], 0)

        time.sleep(0.5)
        sr.motor_42.goal_speed = 40
        sr.motor_43.goal_speed = 35
        sr.motor_42.goal_position = -70
        sr.motor_43.goal_position = 135
        
        legs = [sr.leg1, sr.leg2, sr.leg31, sr.leg32, sr.leg41, sr.leg42]
        for l in legs:
            currentLegPositions[getKeyForLeg(spider_robot, l)] = [0] * 3


        
def getNextStateFromArduinoInput(value,currentState,currentMode):
    
    ch = direction(value)

    if ch == "leftwalking":
        if currentMode == ROTATE_MODE:
            nextState="rotateleft"
        else:
            nextState = ch
    elif ch == "rightwalking":
        if currentMode== ROTATE_MODE:
            nextState = "rotateright"
        else:
            nextState = ch
    elif ch == "waiting": # XXX
        nextState = None
    else:
        nextState = ch
    if(DEBUG_INPUT and nextState != None):
        print("DEBUG INPUT FROM ARDUINO : ch = "+ch +"\ncurrentState = "+currentState+"\nnextState = "+nextState+"\n")
    return nextState
        

def getSubModeFromInput(ch,currentMode,inputMode):
    if inputMode == "arduino":
        return getSubModeFromArduinoInput(ch,currentMode)
    elif inputMode == "keyboard":
        return getSubModeFromKeyboardInput(ch,currentMode)

def getSubModeFromKeyboardInput(ch,currentMode):
    if (ch == '5' or ch == 'g' or ch =='G'):
        if currentMode ==  ROTATE_MODE:
            currentMode = WALK_MODE
        else :
            currentMode = ROTATE_MODE
            print "ROTATE MODE"
    elif(ch == 'o' or ch == 'O'):
        if currentMode == ANGLE_ROTATE_MODE:
            currentMode=WALK_MODE
        else:
            print "ANGLE ROTATE MODE"
            currentMode=ANGLE_ROTATE_MODE
    elif(ch == 'p' or ch =='P'):
        if currentMode == MOVE_TO_MM:
            currentMode = WALK_MODE
        else:
            print "TO MM MODE"
            currentMode=MOVE_TO_MM
    elif(ch == 'm' or ch == 'M'):
        if currentMode ==CENTER_MODE:
            currentMode=WALK_MODE
        else:
            print "CENTER_MODE"
            currentMode=CENTER_MODE
    return currentMode
def getSubModeFromArduinoInput(ch,currentMode):
    ##Get the BUTTONS_ROTATE_SWITCH pressed == switch between rotation mode and walk mode
    
    rotate=buttonsSave()[BUTTONS_ROTATE_SWITCH] == 1
    freeWalking=buttonsSave()[BUTTONS_FREE_WALKING]== 1
   
    if rotate or freeWalking:
        time.sleep(3)
    if rotate :
        if currentMode == ROTATE_MODE:
            currentMode= WALK_MODE
        else:
            currentMode = ROTATE_MODE
    elif freeWalking :
        if currentMode == FREE_WALK_MODE:
            currentMode = WALK_MODE
        else:
            currentMode =FREE_WALK_MODE
    if DEBUG_INPUT:
        print "CURRENT_MODE : " +str(currentMode)
    return currentMode
        
def getNextStateFromInput(ch,currentState,currentMode,inputMode):
    if inputMode == "arduino":
        return getNextStateFromArduinoInput(ch,currentState,currentMode)
    elif inputMode == "keyboard":
        return getNextStateFromKeyboardInput(ch,currentState,currentMode)
    else:
        print("Bad inputMode !\n")
        return None

def getNextStateFromKeyboardInput(ch,currentState,currentMode):
    if (ch == '8' or ch == 't' or ch == 'T'):
        nextState="forwardwalking"
      
    elif (ch == '2' or ch == 'b' or ch =='B'):
        nextState="backwardwalking"
    elif (ch == '4' or ch == 'f' or ch == 'F'):
        if currentMode != ROTATE_MODE :
            nextState="leftwalking"
        else:
            nextState="rotateleft"
    elif (ch == '6' or ch == 'h' or ch =='H'):
        if currentMode != ROTATE_MODE :
            nextState="rightwalking"
        else :
            nextState="rotateright"
    elif (ch == '7' or ch == 'r' or ch =='R'):
        nextState="forwardleftwalking"
    elif (ch == '9' or ch == 'y' or ch =='Y'):
        nextState = "forwardrightwalking"
            
    elif (ch == '1' or ch == 'v' or ch =='V'):
        nextState = "backwardleftwalking"
            
    elif (ch == '3' or ch == 'n' or ch =='N'):
        nextState = "backwardrightwalking"
    elif currentMode == ANGLE_ROTATE_MODE:
        nextState = "anglerotate"
    elif currentMode == MOVE_TO_MM :
        nextState= "tomm"
    elif currentMode == CENTER_MODE:
        nextState="centermove"
    else:
        nextState=None
    if(DEBUG_INPUT and nextState != None):
        print("DEBUG INPUT FROM KEYBOARD : ch = "+ch +"\ncurrentState = "+currentState+"\nnextState = "+nextState+"\n")
    return nextState
    
def getKeyForLeg(sr, l):
    if (l == sr.leg1):
        return 'leg1'
    elif (l == sr.leg2):
        return 'leg2'
    elif (l == sr.leg31):
        return 'leg31'
    elif (l == sr.leg32):
        return 'leg32'
    elif (l == sr.leg41):
        return 'leg41'
    elif (l == sr.leg42):
        return 'leg42'
    else:
        return None
        
        
if __name__ == '__main__':
          


        nextState=None
        currentState="waiting"

##        with pypot.dynamixel.DxlIO('/dev/ttyUSB0', baudrate=1000000) as dxl_io:
##                found_ids = dxl_io.scan()  # this may take several seconds
##                print found_ids
##                changeId(found_ids)
                
        #print ("Initialisation du robot")
        #print(currentLegPositions)
        
        spider_robot=from_json('spider_robot.json')
                    
        #print(currentLegPositions)
        setInit(spider_robot)
        spiderMode(spider_robot)
        inputMode = "keyboard"
        mode="spider"
        currentSubMode=WALK_MODE
        #Launch PollArduino thread 
        arduinoThread=PollArduino()
        arduinoThread.start()
        ##
        
        isInited = True
        time.sleep(1)

        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        ch = ''

        while (ch != 'q' and ch != 'Q'):

            ## Get input from arduino or keyboard (default keyboard)
            if (inputMode == "keyboard"):
                try:
                        tty.setraw(sys.stdin.fileno())  
                        ch = sys.stdin.read(1)
                finally:
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            elif (inputMode == "arduino"): 
                ## TODO = keep reading keyboard here in order to stop arduino mode if not responding 
                ##Get the last value read by PollArduino ( no buffering ) and parse it.
                ##ch == None if getValue or parseArduino failed.
                ch=parseArduino(arduinoThread.getValue())
                
                ## If arduino doesn't work properly change mode to keyboard
                ## and go back to while line 
                if ch == None:
                    inputMode ="keyboard"
                    continue
                ##Save buttons
                buttons(ch)
            
            
            ## Input mode change 
            if (ch == '!'):
                inputMode = "arduino"
                time.sleep(1)
                print("Arduino input mode")
                continue
            elif (ch == 'q'): 
                break
            if (inputMode=="arduino" and buttonsSave()[BUTTONS_SWITCH_MODE] == 1):
                inputMode = "keyboard"
                print("keyboard input mode")
                ch = None
            if (inputMode == "arduino" and buttonsSave()[BUTTONS_SWITCH_SPIDER_MODE] == 1):
                print("Changing to spider mode")
                if (mode == "spider"):
                    mode="scorpion"
                    scorpionMode(spider_robot)
                else:
                    mode="spider"
                    setInit(spider_robot)
                    spiderMode(spider_robot)
         
            ## spider_robot mode change
            if (ch == '0' or ch == 'c' or ch =='c'):
                if (mode == "scorpion"):
                    print("Changing to spider mode")
                    mode="spider"
                    spiderMode(spider_robot)
                else :
                    print("Changing to scorpion mode")
                    mode="scorpion"
                    scorpionMode(spider_robot)
          
            ## symbole to direction ( work with arduino and keyboard)
            oldcurrentMode=currentSubMode
            currentSubMode = getSubModeFromInput(ch,currentSubMode,inputMode)
            
            nextState=getNextStateFromInput(ch,currentState,currentSubMode,inputMode)
            ## If symbole is found
            if(nextState != None):

                if (mode == "scorpion") :

                    if(currentSubMode == WALK_MODE):
                        if(currentState != nextState ):
                            scorpionWalk(False,nextState,spider_robot)    
                        else:
                            scorpionWalk(True,nextState,spider_robot)
                    elif(currentState == FREE_WALK_MODE):
                        if currentState != nextState:
                            scorpionFreeWalk(False,nextState,spider_robot,ch)
                        else:
                            scorpionFreeWalk(True,nextState,spider_robot,ch)

                elif (mode=="spider"):

                    if(currentSubMode == WALK_MODE):
                        if "walking" in currentState:
                            spiderWalk(False,nextState,spider_robot)
                        else:
                            spiderWalk(True,nextState,spider_robot)

                    elif(currentSubMode == FREE_WALK_MODE):
                        if "walking" in currentState:
                            spiderFreeWalk(False,nextState,spider_robot,ch)
                        else:
                            spiderFreeWalk(True,nextState,spider_robot,ch)
                    elif(currentSubMode == MOVE_TO_MM):
                        print "Entrer la valeur souhaitee en MM X"
                        mmX=int(input())
                        print "Entrer la valeur souhaitee en MM Y"
                        mmY=int(input())
                        spiderGoToMM(spider_robot,mmX,mmY)
                    elif currentSubMode == CENTER_MODE:
                      moveLeg(spider_robot.leg1,20,20,20)
                      moveLeg(spider_robot.leg2,20,20,20)
                    elif(currentSubMode == ROTATE_MODE and oldcurrentMode != ROTATE_MODE):
                        stabilizeSpiderAfterWalking(spider_robot)
                    if(nextState == "rotateleft"):
                        spiderRotate(spider_robot, False)
                    elif (nextState == "rotateright"):
                        spiderRotate(spider_robot, True)
                    elif(currentSubMode == ANGLE_ROTATE_MODE and inputMode == "keyboard" ):
                        print "Entrer la valeur souhaitee"
                        tmpAngle=float(input())
                        spiderAngleRotate(spider_robot,tmpAngle*math.pi/180.,False)
                ## if nextState != None , update current state .
                currentState = nextState

           
                        

        print("Deconnexion")
        arduinoThread.stop()
        setInit(spider_robot)
        
        spider_robot.close()


        
