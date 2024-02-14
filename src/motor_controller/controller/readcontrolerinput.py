
import serial
from time import sleep
import threading
from xboxcontroller import XboxController
from serialmonitor import SerialMonitor

if __name__ == '__main__':
    dec=2
    sep=';'
    speed=1
    print("Starup com connection")
    ser = serial.Serial('COM6',9600)
    print("connected")
    try:
        print("setup monitor")
        monitor=SerialMonitor(ser)
        print("setup joystic")
        joy = XboxController()
        px=str('0.0')
        py=str('0.0')
        pz=str('0.0')
        pressed=False
        while True:
            values=joy.read()
            x=str(round(values[0][1],dec) * speed)
            y=str(-round(values[0][0],dec) * speed)
            # l=round(values[0][4],dec)
            # r=round(values[0][5]*-1,dec)
            # z=str((l+r)*speed)
            z=str(round(values[0][2],dec) * speed)
            if(values[1][0]==1 or values[1][2] == 1):
                if(not pressed):
                    speed= speed + values[1][0] * -1 + values[1][2]
                    pressed=True
                    print(pressed,speed)
            else:
                pressed=False

            # if( and not pressed and speed - 1 > 0):
            #     # speed=speed-1
            #     down=True
            #     print(pressed,speed)
            # elif( and not pressed):
            #     # speed=speed+1
            #     pressed=True
            #     print(pressed,speed)
            # else:

            if( x != px or y != py or z != pz):
                pl=x+";"+y+";"+z
                print(pl)
                msg=pl+"\n"
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
