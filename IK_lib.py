from math import sin, cos, acos, atan, asin, sqrt
def cartesian_to_angles(pos, d1, d2):
    (x, y, z) = pos
    try:
        r = sqrt(x**2 + y**2 + z**2)
        base_angle = atan(y/x) + 90 #straight fwd is 90 degrees
        elbow_angle = acos( (r**2 - d1**2 - d2**2) / (2*d1*d2)) #straight is 0 degrees
        shoulder_angle = asin(z/r) + atan( (d2*sin(elbow_angle)) / (d1+d2*cos(elbow_angle))) + 90 #straight upwards is 90 degrees
        arm_angles = [base_angle, shoulder_angle, elbow_angle]
    except ValueError as error:
        print("%r, pos was %r" %(error, pos))
        return None
    return arm_angles 
    
def angles_to_us(arm_angles,servo_min=1000,servo_max=2000):
    (base_angle, shoulder_angle, elbow_angle) = arm_angles 
    servo_dev = (servo_max - servo_min)/2
    base_us = int(base_angle / 180 * servo_dev) + servo_min
    shoulder_us = int(shoulder_angle / 180 * servo_dev) + servo_min
    elbow_us = int(elbow_angle / 180 * servo_dev) + servo_min
    us_tuple = (base_us, shoulder_us, elbow_us)
    return us_tuple