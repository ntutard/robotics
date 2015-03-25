from threading import Thread,Lock
import time
import serial

class PollArduino(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.ser=serial.Serial('/dev/ttyACM0',9600)
        self.shared=""
        self.notStopping=True
    def stop(self):
        self.notStopping=False
    def run(self):
        while(self.notStopping):
            self.shared=self.ser.readline()
            
    def getValue(self):
        s=self.shared
        return s

# p=PollArduino()
# p.start()
# while(True):
#     print p.getValue()
#     time.sleep(0.01)


