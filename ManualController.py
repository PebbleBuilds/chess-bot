#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Point

def talker():
    pub = rospy.Publisher('target_positions', Point, queue_size=10)
    rospy.init_node('manual_controller', anonymous=True)
    while not rospy.is_shutdown():
        position = (raw_input("enter x:"), raw_input("enter y:"), raw_input("enter z:"))
        try:
            for pos in position:
                position = float(pos)
        except TypeError:
            continue

        msg = Point(x=position[0], y=position[1], z=position[2])

        rospy.loginfo(msg)
        pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass