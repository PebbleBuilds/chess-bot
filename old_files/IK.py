# Inverse Kinematics Code
# By KiriBuilds
#
# This code:
# - ingests target positions in the form of a list of 3-tuples
# - outputs motor angles to an Arduino

#!/usr/bin/env python
import rospy, serial, sys, time
from math import sin, cos, acos, atan, asin, sqrt
from geometry_msgs.msg import Point

_QUEUE_MAX = 50

_CMD_SET_BASE = 1
_CMD_SET_SHOULDER = 2
_CMD_SET_ELBOW = 3
_CMD_SET_INTERVAL = 4

_CFG_CMD_VAL_PLACES = 4

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

class IK():
    def cartesian_to_angles(pos):
        (x, y, z) = pos
    
        r = math.sqrt(x**2 + y**2 + z**2)
        base_angle = atan(y/x)
        elbow_angle = -1 * acos( (r**2 - d1**2 * d2**2) / (2*d1*d2))
        shoulder_angle = asin(z/r) + atan( (d2*sin(elbow_angle)) / (d1+d2*cos(elbow_angle)))
        angle_tuple = (base_angle, shoulder_angle, elbow_angle)
        return angle_tuple
    
    def angles_to_us(angle_tuple):
        (base_angle, shoulder_angle, elbow_angle) = angle_tuple
    
        base_us = int(base_angle / 180 * 1000) + 1000
        shoulder_us = int(shoulder_angle / 180 * 1000) + 1000
        elbow_us = int(elbow_angle / 180 * 1000) + 1000
        us_tuple = (base_us, shoulder_us, elbow_us)
        return us_tuple
    
    def find_intermediate_positions(init_pos, final_pos):
        curr_pos = init_pos
        increments = init_pos
        intermediate_positions = []
    
        for idx in range(0, len(init_pos)):
            increments[idx] = (final_pos[idx] - init_pos[idx]) / _QUEUE_MAX
    
        for n in range(0, _QUEUE_MAX):
            for idx in range(0, len(init_pos)):
                curr_pos[idx] += increments[idx]
            intermediate_positions.append(curr_pos)
    
        return intermediate_positions
    
    def us_3tuple_to_cmds(us_3tuple):
        cmd_base = us_3tuple[0] + 10**_CFG_CMD_VAL_PLACES
        cmd_shoulder = us_3tuple[1] + 10**_CFG_CMD_VAL_PLACES
        cmd_elbow = us_3tuple[2] + 10**_CFG_CMD_VAL_PLACES
        return(cmd_base, cmd_shoulder, cmd_elbow)
    
    def duration_to_interval(duration):
        interval = duration 
    
    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
        msg = data.data
        final_pos = (msg.x, msg.y, msg.z)
    
        i_positions = find_intermediate_positions(init_pos, final_pos)
        i_us_list = []
    
        for pos in i_positions:
            i_us = angles_to_us(cartesian_to_angles(pos))
            i_us_list.append(i_us)
    
        if not move_checker.check_if_done():
            print("Uh oh. The arm wasn't done moving. Try again.")
            return
    
        for pos in i_us_list:
            for cmd in us_3tuple_to_cmds(us_3tuple):
                #ser.write(cmd)
                print(cmd)
        
        move_checker.start_timing()
        
    def listener():
    
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('IK', anonymous=True)
    
        rospy.Subscriber("target_positions", Point, callback)
    
        com_port = sys.argv[1]
        #ser = serial.Serial(com_port, 9800, timeout=1)
        moving_checker = CheckMoving(duration)
    
        d1 = 175 # (in mm)
        d2 = 175
    
        init_pos = (d2+30, 0, d1) # x toward opponent, y to the right, z up
        duration = 3000 # in ms
    
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    listener()