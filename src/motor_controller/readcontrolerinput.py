from inputs import get_gamepad
import math
import threading
import serial
from time import sleep

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self): # return the buttons/triggers that you care about in this methode
        ly=self.LeftJoystickY
        lx=self.LeftJoystickX
        ry=self.RightJoystickY
        rx=self.RightJoystickX
        lt=self.LeftTrigger
        rt=self.RightTrigger
        # lb=self.LeftBumper
        # rb=self.RightBumper
        a=self.A
        x=self.X
        y=self.Y
        b=self.B
        # lth=self.LeftThumb
        # rth=self.RightThumb
        # back=self.Back
        # start=self.Start
        # ldp=self.LeftDPad
        # rdp=self.RightDPad
        # udp=self.UpDPad
        # dpd=self.DownDPad
        return [[lx,ly,rx,ry,lt,rt],[a,x,y,b]]

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state

if __name__ == '__main__':
    dec=1
    sep=';'
    ser = serial.Serial('COM4',9600)
    speed=100
    try:
        joy = XboxController()
        px=float(0.0)
        py=float(0.0)
        pz=float(0.0)
        while True:
            values=joy.read()
            x=float(values[0][2])
            y=float(values[0][0])
            z=float(values[0][1])
            if( x!=px or y!=py or z != pz):
                print(values)
                s=str(round(x*2.0, dec))+";"+str(round(y*3, dec))+";"+str(round(z*3, dec))+"\n"
                ser.write(s.encode())
                ser.flush()
                # result = ser.read_until()
                # print(result.decode("ASCII"))
            px=x
            py=y
            pz=z
    except Exception as e:
        print(e)
    finally:
        ser.close()
        print("Close")