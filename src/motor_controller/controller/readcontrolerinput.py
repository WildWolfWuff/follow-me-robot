
import math
import threading
import serial
from time import sleep
from xboxcontroller import XboxController
from serialmonitor import SerialMonitor
class SerialMonitor(object):
    def __init__(self, ser):
        self.ser=ser
        self.display_thread=threading.Thread(target=self._print_msg,args=())
        self.display_thread.daemon = True
        self.display_thread.start()
    def _print_msg(self):
        while self.ser.open:
            result = self.ser.read_until()
            print(result.decode("ASCII"))

if __name__ == '__main__':
    dec=1
    sep=';'
    ser = serial.Serial('COM5',9600)
    speed=1
    try:
        monitor=SerialMonitor(ser)
        joy = XboxController()
        px=str('0.0')
        py=str('0.0')
        pz=str('0.0')
        while True:
            values=joy.read()
            x=str(round(values[0][1],dec) * speed)
            y=str(-round(values[0][0],dec) * speed)
            z=str(round(values[0][2],dec) * speed)
            if( x != px or y != py or z != pz):
                msg=x+";"+y+";"+z+"\n"
                print(msg)
                ser.write(msg.encode())
                ser.flush()
            px=x
            py=y
            pz=z
    except Exception as e:
        print(e)
    finally:
        ser.close()
        print("Close")
