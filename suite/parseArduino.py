from subprocess import Popen
import fileinput
import subprocess



def buttons(tabValue):
    tabRetour=[0,0,0,0,0]
    assert(len(tabValue)>=7)
    for i in range(2,7):
        assert(tabValue[i]==0 or  tabValue[i] == 1)
        tabRetour[i-2]=(tabValue[i]+1)%2
    return tabRetour

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
        return "idle"
    elif (tabDirection == [0,1]):
        return "right"
    elif (tabDirection == [0,-1]):
        return "left"
    elif (tabDirection == [1,0]):
        return "forward"
    elif (tabDirection == [1,1]):
        return "forwardRight"
    elif (tabDirection == [1,-1]):
        return "forwardLeft"
    elif (tabDirection == [-1,0]):
        return "backward"
    elif (tabDirection == [-1,1]):
        return "backwardRight"
    elif (tabDirection == [-1,-1]):
        return "backwardLeft"
    
def parseArduino(msg):
    msgsplited=msg.split('#',6)
    if(len(msgsplited)==7):
        return [int(msgsplited[0]),int(msgsplited[1]),int(msgsplited[2]),int(msgsplited[3]),int(msgsplited[4]),int(msgsplited[5]),int(msgsplited[6])]
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
