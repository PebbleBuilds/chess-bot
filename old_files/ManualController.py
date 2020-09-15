#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Point

class ROS_ManualController():
    def __init__(self):
        self.pub = rospy.Publisher('target_positions', Point, queue_size=10)
        rospy.init_node('manual_controller', anonymous=True)

    def controller(self):
        while not rospy.is_shutdown():
            position = [raw_input("enter x:"), raw_input("enter y:"), raw_input("enter z:")]
            try:
                for i, val in enumerate(position):
                    print(val)
                    position[i] = float(val)
            except ValueError:
                print("Error: Must be a float")
                continue

            msg = Point(x=position[0], y=position[1], z=position[2])

            rospy.loginfo(msg)
            self.pub.publish(msg)

if __name__ == '__main__':
    try:
        controller = ROS_ManualController()
        controller.controller()
    except rospy.ROSInterruptException:
        pass