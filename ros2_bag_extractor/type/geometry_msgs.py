from math import sqrt, pow
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from ros2_bag_extractor.util.mathematics import quaternion_to_euler as q2e

'''
@brief PoseStamped, PoseWithCovariance
'''
def convertPoseStamped(p, time=0):
  roll, pitch, yaw = q2e(p.pose.orientation)
  data = pose(p.pose.position.x, p.pose.position.y, yaw, time)
  return data

def convertPoseWithCovariance(p:PoseWithCovarianceStamped, time=0):
  return convertPoseStamped(p.pose, time)

def convertTwist(tw:Twist, time=0):
  data = speed(tw.linear.x, tw.linear.y, tw.angular.z, time)
  return data

class pose:
  def __init__(self, pose_x, pose_y, pose_theta, time_nanoseconds=0):
    """
    @pose_x coordinate x [m]
    @pose_y coordinate y [m]
    @pose_theta coordinate theta [rad]
    @time_nanoseconds [ns]
    """
    self.x = pose_x
    self.y = pose_y
    self.head = pose_theta
    self.sec = time_nanoseconds*1e-9

class speed:
  def __init__(self, v_x, v_y, rot_w, time_nanoseconds=0):
    """
    @v_x linear speed x [m]
    @v_y linear speed y [m]
    @rot_w z-axis rotation speed [rad/s]
    @time_nanoseconds [ns]
    """
    self.vx = v_x
    self.vy = v_y
    self.speed = sqrt(pow(v_x, 2)+pow(v_y, 2))
    self.w = rot_w
    self.sec = time_nanoseconds*1e-9