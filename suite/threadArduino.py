from threading import Thread,Lock
import time
import serial

class PollArduino(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.ser=serial.Serial('/dev/ttyACM0',9600)
        self.shared=""
        self.notStopping=True
        self.started=False
    def stop(self):
        self.notStopping=False
    def run(self):
        self.started=True
        while(self.notStopping):
            self.shared=self.ser.readline()
            
    def getValue(self):
        if self.started:
            s=self.shared
            return s
        else:
            print "Thread not started !\n"
            return None

# p=PollArduino()
# p.start()
# while(True):
#     print p.getValue()
#     time.sleep(0.01)


