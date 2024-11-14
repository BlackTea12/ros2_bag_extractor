from nav_msgs.msg import Path
from ros2_bag_extractor.type.geometry_msgs import convertPoseStamped

def convertPath(_path:Path, time=0):
  poses = []
  for p in _path.poses:
    poses.append(convertPoseStamped(p))
  data = path(poses, time)
  return data

class path:
  def __init__(self, poses, time_nanoseconds):
    """
    @poses list of ros2_bag_extractor.type.pose
    @time_nanoseconds [ns]
    """
    self.poses = poses
    self.sec = time_nanoseconds*1e-9