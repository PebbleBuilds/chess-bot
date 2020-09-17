# ChessBotCore
# By Rocco Ruan
# 
# A class that contains functions for specific types of moves (captures, regular moves, or promotions), which are to be
# called by ChessBotCore.This class will then send serial commands to ChessBot_IK.ino on the Arduino Nano to execute those moves.

from constants import *

class CheckMoving():
    def __init__(self, duration):
        self.duration = duration
        self.last_time = time.time() * 1000

    def start_timing(self):
        self.last_time = time.time() * 1000

    def check_if_done(self):
        if time.time() * 1000 - self.last_time + 500 > self.duration:
            return True
        return False

class ArmController():
    def __init__(self, com_port=None, duration=3000):
        self.init_pos = [0.0,50.0,50.0]
        self.com_port = com_port
        self.duration = duration # in ms
        self.move_checker = CheckMoving(duration)

        self.piece_grab_z = -20 # z position in mm that the arm should be in to grab a piece
        self.clearance_z = 0 # z position in mm that the arm should be in to move around without hitting stuff

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

    def remove_piece(self, piece_location):
        (piece_x, piece_y) = piece_location
        