from constants import *

class ArmController():
    def __init__(self, com_port=None, duration=3000):
        self.init_pos = [0.0,50.0,50.0]
        self.com_port = com_port
        self.duration = duration # in ms
        self.move_checker = CheckMoving(duration)

        self.ser = None

        self.ser_init()

        if self.ser is not None:
            self.send_cmd(self.CMD_SET_INTERVAL, duration // self.queue_max)
            print("[ArmController] serial connected")

    def ser_init(self):
        baud = 9600
        baseports = ['/dev/ttyUSB', '/dev/ttyACM', 'COM', '/dev/tty.usbmodem1234']
        self.ser = None

        while not self.ser:
            for baseport in baseports:
                if self.ser:
                    break
                for i in xrange(0, 64):
                    try:
                        port = baseport + str(i)
                        self.ser = serial.Serial(port, baud, timeout=1)
                        print("Monitor: Opened " + port + '\r')
                        break
                    except:
                        self.ser = None
                        pass

            if not self.ser:
                print("[ArmController] Couldn't open a serial port.")

    def send_cmd(self, cmd_id, cmd_val):
        print("[ArmController] sending cmd_id: %d with cmd_val: %d"%cmd_id, cmd_val)
        if self.ser == None:
            return None
        self.ser.write((str(cmd_id)+'/'+str(cmd_val)+'/').encode())