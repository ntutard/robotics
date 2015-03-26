from subprocess import Popen
import fileinput
import subprocess


def direction(tabPosition):
    directionX=0
    directionY=0
    if(tabPosition[0]> 10 ):
        directionX=1
    elif (tabPosition[0]< -10):
        directionX=-1
    if(tabPosition[1]>10):
        directionY=1
    elif(tabPosition[1]< -10):
        directionY=-1
    tabDirection=[-directionY,-directionX]
    if(tabDirection == [0,0]):
        return "idle"
    elif (tabDirection == [0,1]):
        return "right"
    elif (tabDirection == [0,-1]):
        return "left"
    elif (tabDirection == [1,0]):
        return "forward"
    elif (tabDirection == [1,1]):
        return "forwardright"
    elif (tabDirection == [1,-1]):
        return "forwardleft"
    elif (tabDirection == [-1,0]):
        return "backward"
    elif (tabDirection == [-1,1]):
        return "backwardright"
    elif (tabDirection == [-1,-1]):
        return "backwardleft"
    
def parseArduino(msg):
    msgsplited=msg.split('#',1)
    if(len(msgsplited)==2 and len(msgsplited[0]) > 0 and len(msgsplited[1]) > 0):
        return [int(msgsplited[0]),int(msgsplited[1])]
    return None

