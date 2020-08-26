# Inverse Kinematics Code
# By KiriBuilds
#
# This code:
# - ingests target positions in the form of a list of 3-tuples
# - outputs motor angles to an Arduino

#!/usr/bin/env python
import rospy, serial, sys, time
from IK_lib import *
from geometry_msgs.msg import Point

CMD_SET_BASE = 1
CMD_SET_SHOULDER = 2
CMD_SET_ELBOW = 3
CMD_SET_INTERVAL = 4
CMD_GET_QUEUE_MAX = 5

CFG_CMD_VAL_PLACES = 4

class CheckMoving():
    def __init__(self, duration):
        self.duration = duration
        self.last_time = time.time() * 1000

    def start_timing():
        self.last_time = time.time() * 1000

    def check_if_done():
        if time.time() - self.last_time + 500 > duration:
            return True
        return False

class IK_Arm():
    def __init__(self, com_port=None, duration=3000):
        self.init_pos = (0,50,50)
        self.com_port = com_port
        self.duration = duration # in ms
        self.moving_checker = CheckMoving(duration)
    
        self.d1 = 175 # (in mm)
        self.d2 = 175

        self.ser = None

        if self.com_port is not None:
            self.ser = serial.Serial(com_port, 9800, timeout=1.0)
            send_cmd(GET_QUEUE_MAX, 0)
            try:
                self.queue_max = int(self.ser.read(1))
            except:
                raise("Error: Couldn't get queue_max from Nano")    
        else:
            print("WARNING: No com_port specified")
    
    def find_intermediate_positions(self, init_pos, final_pos):
        curr_pos = init_pos
        increments = init_pos
        intermediate_positions = []
    
        for idx in range(0, len(init_pos)):
            increments[idx] = (final_pos[idx] - init_pos[idx]) / self.queue_max
    
        for n in range(0, self.queue_max):
            for idx in range(0, len(init_pos)):
                curr_pos[idx] += increments[idx]
            intermediate_positions.append(curr_pos)
    
        return intermediate_positions

    def send_cmd(self, cmd_id, cmd_val):
        cmd_id_shift = 10**CFGCMD_VAL_PLACES 
        if cmd_id_shift < val:
            print("Invalid command value received")
            return None
        cmd = cmd_id_shifted * cmd_id + val
        print("cmd:",cmd)
        if ser == None:
            return None
        ser.write(cmd)
    
    def callback(self, received_point):
        rospy.loginfo(rospy.get_caller_id() + "I heard %f, %f, %f" %(received_point.x, received_point.y, received_point.z))
    
        final_pos = (received_point.x, received_point.y, received_point.z)
    
        i_positions = self.find_intermediate_positions(self.init_pos, final_pos)
        i_us_list = []
    
        for pos in i_positions:
            i_us_list.append(angles_to_us(cartesian_to_angles(pos)))
    
        if not move_checker.check_if_done():
            print("Uh oh. The arm wasn't done moving. Try again.")
            return
    
        for pos in i_us_list:
            send_cmd(CMD_SET_BASE, pos[0])
            send_cmd(CMD_SET_SHOULDER, pos[1])
            send_cmd(CMD_SET_ELBOW, pos[2])
        
        move_checker.start_timing()
        self.init_pos = final_pos

#ROS stuff down here
def listener(com_port=None):
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('IK', anonymous=True)    

    arm = IK_Arm(com_port)

    rospy.Subscriber("target_positions", Point, arm.callback)
         
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass