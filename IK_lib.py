from math import sin, cos, acos, atan, asin, sqrt
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