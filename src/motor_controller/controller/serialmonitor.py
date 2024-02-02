import serial

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