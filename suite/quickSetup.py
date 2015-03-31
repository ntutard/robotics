import pypot.dynamixel
import pypot.dynamixel.conversion
import pypot.robot.robot
import itertools
import time
import numpy


## Raise each leg one after one clockwise
def quickTestSetup(spider_robot):
    coefZ=15
    up=[0,0,coefZ]
    down=[0,0,-coefZ]
    legs=[sr.leg1,sr.leg42,sr.leg41,sr.leg2,sr.leg31,sr.leg.32] ## XXX May not be the good order .
    for leg in legs:
        moveLegsRepere(spider_robot,leg,[up,down],0.5)


    

    
    
    
