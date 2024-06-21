import math
from geometry_msgs.msg import Quaternion

def euler_to_quaternion(roll, pitch, yaw):
  '''
  @return geometry_msgs.msg Quaternion
  '''
  qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
  qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
  qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
  qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

def quaternion_to_euler(q:Quaternion):
  '''
  @return (roll, pitch, yaw)
  '''
  # Roll (x-axis rotation)
  sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
  cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
  roll = math.atan2(sinr_cosp, cosr_cosp)

  # Pitch (y-axis rotation)
  sinp = 2 * (q.w * q.y - q.z * q.x)
  if abs(sinp) >= 1:
      pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
  else:
      pitch = math.asin(sinp)

  # Yaw (z-axis rotation)
  siny_cosp = 2 * (q.w * q.z + q.x * q.y)
  cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
  yaw = math.atan2(siny_cosp, cosy_cosp)

  return (roll, pitch, yaw)