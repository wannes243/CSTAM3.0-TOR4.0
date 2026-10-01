import math

def quaternion_from_euler(roll, pitch, yaw):
    cr=math.cos(roll*0.5); sr=math.sin(roll*0.5); cp=math.cos(pitch*0.5); sp=math.sin(pitch*0.5); cy=math.cos(yaw*0.5); sy=math.sin(yaw*0.5)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)

def euler_from_quaternion(q):
    x,y,z,w=map(float,q)
    sinr=2*(w*x+y*z); cosr=1-2*(x*x+y*y); roll=math.atan2(sinr,cosr)
    sinp=2*(w*y-z*x); pitch=math.copysign(math.pi/2,sinp) if abs(sinp)>=1 else math.asin(sinp)
    siny=2*(w*z+x*y); cosy=1-2*(y*y+z*z); yaw=math.atan2(siny,cosy)
    return roll,pitch,yaw
