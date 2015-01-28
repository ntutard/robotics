#!/usr/bin/python
# -*- coding: utf-8 -*-

import serial

DATA_START = 0xff
INSTRUCTION_WRITE = 0x03
INSTRUCTION_PING = 0x01
INSTRUCTION_READ=0x02
LED=0x19
LED_ON = 0x01
LED_OFF = 0x00

def open_serial(port, baud,timeout=0.05):
    ser = serial.Serial(port=port, baudrate=baud,timeout=timeout)
    if ser.isOpen():
        return ser
    else:
        print 'SERIAL ERROR'


def close(ser):
    ser.close()


def write_data(ser, data):
    ser.write(data)


def read_data(ser, size=1):
    return ser.read(size)


def to_hex(val):
    return chr(val)

def calcChecksum(servoId, length, instruction, params):
    checksum = servoId + length + instruction
    
    for i in range(len(params)):
        checksum+=params[i]
        
    checksum = ~checksum
    checksum = checksum & 0xFF
    
    return to_hex(checksum)

def parseDatas(servoId, l, ins, params):
    data = to_hex(DATA_START) + to_hex(DATA_START) + to_hex(servoId) + to_hex(l) + to_hex(ins)
    for i in range(len(params)):
        data += to_hex(params[i])

    data += calcChecksum(servoId, l, ins, params)

    return data


def instructionRead(servoId,params):
    data_length=0x04
    
    return parseDatas(servoId,data_length,INSTRUCTION_READ,params)

def instructionWrite(servoId, params):
    data_length = 0x03 + (len(params) - 1)

    return parseDatas(servoId, data_length, INSTRUCTION_WRITE, params)

def instructionPing(servoId):
    data_length = 0x02

    return parseDatas(servoId, data_length, INSTRUCTION_PING, []);

def findServo():
    idFound = []
    for i in range(0, 254): # Pas 256 puisque 254-255 = broadcast
        data = instructionPing(i)
    
        # print decode_data(data)
        write_data(serial_port, data)

        # read the status packet (size 6)
        d = read_data(serial_port, 6)

        if (d!=""):
            idFound.insert(0,i)
            print "Servo ID found : " + str(i)

    return idFound
            
def decode_data(data):
    res = ''
    for d in data:
        res += hex(ord(d)) + ' '

    return res

def decode_data_array(data):
    res = []
    for d in data:
        res.insert(len(res), ord(d))

    return res

def sendData(data):
    write_data(serial_port, data)

def readData(size):
    return read_data(serial_port, size)

def sendReadOn(servoId, params):
    data = instructionRead(servoId, params)
    
    sendData(data)
    
    currentData = decode_data_array(readData(4))
    l = 2 + currentData[3]
    currentData = decode_data_array(readData(l))

    return currentData

if __name__ == '__main__':

    serial_port = open_serial('/dev/ttyUSB0', 1000000, timeout=0.1) 

    # # we open the port
    
    # # print findServo()
    data = instructionWrite(0x01, [0x20, 0x60, 0x00])

    # # print decode_data(data)
    write_data(serial_port, data)

    # # read the status packet (size 6)
    d = read_data(serial_port, 6)

    # # print findServo()
    data = instructionWrite(0x01, [0x1E, 0x00, 0x03])

    # # print decode_data(data)
    write_data(serial_port, data)

    # # read the status packet (size 6)
    d = read_data(serial_port, 6)

    print decode_data(d)


    print sendReadOn(0x01, [0x1E, 0x02])
    
