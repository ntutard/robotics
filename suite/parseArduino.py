from subprocess import Popen
import fileinput
import math
import subprocess


MAX_JOYSTICK_ARDUINO = 512
UNLOCK_BUTTONS=[True,True,True,True,True]
BUTTONS_NAMES=["Joystick button","switch_down button","switch_left button","switch_up button","switch_right button"]

## [0,6[ , one button <=> one action 
BUTTONS_ROTATE_SWITCH=0
BUTTONS_SWITCH_MODE=1
BUTTONS_FREE_WALKING=2


## Get the tab with 5 buttons values , 1 == pressed . 
## See buttons_names for order .
## Buttons cannot be hold . Only count for 1 time pressed for simplicity (UNLOCK_BUTTONS) .
def buttons(tabValue):
    tabRetour=[0,0,0,0,0]
    assert(len(tabValue)>=7)
    for i in range(2,7):
        assert(tabValue[i]==0 or  tabValue[i] == 1)
        ## Prevent buttons to be pressed multiple time by waiting each one to be released before watching them again
        ## 0011111001 -> 00100000001 
        if(UNLOCK_BUTTONS [i-2]) :
            tabRetour[i-2]=(tabValue[i]+1)%2
        else :
            tabRetour[i-2]=0
        if((tabValue[i]+1)%2 == 0):
            UNLOCK_BUTTONS[i-2]= True
        else:
            UNLOCK_BUTTONS[i-2]= False
    
            
    return tabRetour

def coeff(tabValue):
    assert(len(tabValue)>=2)
    return [abs(tabValue[0])/512.,abs(tabValue[1])/512.]

def angleRad(tabValue):
    assert(len(tabValue)>=2)
    return math.atan2(-tabValue[1],-tabValue[0])


def direction(tabValue):
    directionX=0
    directionY=0
    assert(len(tabValue)>=2)
    if(tabValue[0]> 10 ):
        directionX=1
    elif (tabValue[0]< -10):
        directionX=-1
    if(tabValue[1]>10):
        directionY=1
    elif(tabValue[1]< -10):
        directionY=-1
    tabDirection=[-directionY,-directionX]
    if(tabDirection == [0,0]):
        return "waiting"
    elif (tabDirection == [0,1]):
        return "rightwalking"
    elif (tabDirection == [0,-1]):
        return "leftwalking"
    elif (tabDirection == [1,0]):
        return "forwardwalking"
    elif (tabDirection == [1,1]):
        return "forwardrightwalking"
    elif (tabDirection == [1,-1]):
        return "forwardleftwalking"
    elif (tabDirection == [-1,0]):
        return "backwardwalking"
    elif (tabDirection == [-1,1]):
        return "backwardrightwalking"
    elif (tabDirection == [-1,-1]):
        return "backwardleftwalking"
    else :
        return None
    
def parseArduino(msg):
    msgSplitedInt=[0 for i in range(10)]
    msgsplited=msg.split('#',9)
    if(len(msgsplited)==10):
        for i in range(10):
            msgSplitedInt[i]=int(msgsplited[i])
        return msgSplitedInt
    return None


# while(True):
#     #call('/home/louis/programmation/arduinoProgrammation/src/ezArduinoSerial/testEzArduinoSerial')
#     p=Popen('/home/louis/programmation/arduinoProgrammation/src/ezArduinoSerial/testEzArduinoSerial',stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
#     while(True):
#           value=p.stdout.readline()
#           print value
#           parsed=parseArduino(value)
#           if parsed != None:
#              print direction(parsed)
