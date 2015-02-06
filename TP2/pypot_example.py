import itertools
import time
import numpy
import pypot.dynamixel
import pypot.dynamixel.conversion

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
    print("Ready !")

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
                    ids = interchangeId(ids, ids[x], i)
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
    with pypot.dynamixel.DxlIO('/dev/ttyUSB0', baudrate=1000000) as dxl_io:

        # we can scan the motors
        found_ids = dxl_io.scan()  # this may take several seconds
        print 'Detected:', found_ids

        # we power on the motors
        dxl_io.enable_torque(found_ids)

        # we get the current positions
        print 'Current pos:', dxl_io.get_present_position(found_ids)

        # # we create a python dictionnary: {id0 : position0, id1 : position1...}
        # pos = dict(zip(found_ids, itertools.repeat(0)))
        # print 'Cmd:', pos
        # # we send these new positions
        # dxl_io.set_goal_position(pos)
        # time.sleep(1)  # we wait for 1
    
        changeId(found_ids)
        #suivreSinusoide(found_ids,0.5,30,15)
        # we get the current positions
        print 'New pos:', dxl_io.get_present_position(found_ids)

        # we power off the motors

        time.sleep(1)  # we wait for 1s

        
