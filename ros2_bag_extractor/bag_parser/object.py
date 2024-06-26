from nav_msgs.msg import Path
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance, Twist, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from math import sqrt, pow
import ros2_bag_extractor.util.mathematics as math_helper
'''
@brief Selecting data with user needs
'''
class ObjectType():
  def __init__(self, data:list):
    """
    @data [(timestamp0, message0), (timestamp1, message1), ...]
    """
    self.org = data # list[tuple[any, any], ...]
    if self.org:
      self.timestamps = [t[0] for t in self.org]
      self.data = [t[1] for t in self.org]
    else:
      print("data none initialized")
  
  def get_data(self, data_type:str):
    '''
    @data_type: what type is bounded with timestamp
    '''
    if data_type == "Path":
      return self._get_PathDiffList(self.org)
    elif data_type == "MarkerArray":
      return self._get_VisMarkerArrayPoseList(self.org)
    elif data_type == "PoseStamped":
      return self._get_PoseStampedList(self.org)
    elif data_type == "PoseWithCovarianceStamped":
      return self._get_PoseWithCovarianceStamped(self.org)
    else:
      print(f"no matching type for {data_type}")
    return
  
  def _get_PathDiffList(self, data : list):
    '''
    @data: list of (timestamp, nav_msgs.msg Path)
    @brief: compares Path data and only save when it differs
    '''
    result = [(data[0][0], data[0][1])]
    for time, path in data:
      if not self._isPathSame(path, result[-1][1]):
        result.append((time, path))
    return result

  def _isPathSame(self, path1:Path, path2:Path):
    if len(path1.poses) == len(path2.poses):
      if path1.poses[0].pose.position.x == path2.poses[0].pose.position.x \
        and path1.poses[0].pose.position.y == path2.poses[0].pose.position.y:
        return True
    return False

  def _get_VisMarkerArrayPoseList(self, data : list):
    '''
    @data: list of (timestamp, visualization marker array)
    @brief: save for geometry_msgs Pose
    '''
    result = []
    for time, markers in data:
      result.append((time, self._get_one_marker_from_markers(markers)))
    return result
  
  def _get_one_marker_from_markers(self, markers:MarkerArray, num=0):
    '''
    @num : which index to observe
    '''
    return self._get_marker_pose(markers.markers[num])
  
  def _get_marker_pose(self, marker:Marker):
    return marker.pose
  
  def _get_PoseStampedList(self, data:list):
    result = []
    for time, posestamp in data:
      result.append((time, self._ps_get_xydeg(posestamp)))
    return result
  
  def _get_PoseWithCovarianceStamped(self, data:list):
    result = []
    for time, posestampcov in data:
      result.append((time, self._pcs_get_xydeg(posestampcov)))
    return result
  
  def _ps_get_xydeg(self, pose:PoseStamped):
    roll, pitch, yaw = math_helper.quaternion_to_euler(pose.pose.orientation)
    return (pose.pose.position.x, pose.pose.position.y, yaw)
  
  def _pcs_get_xydeg(self, pose:PoseWithCovarianceStamped):
    roll, pitch, yaw = math_helper.quaternion_to_euler(pose.pose.pose.orientation)
    return (pose.pose.pose.position.x, pose.pose.pose.position.y, yaw)
  
  def _get_speed_vector_size(self, vx, vy):
    '''
    @brief returning speed absolute value
    '''
    return sqrt(pow(vx,2)+pow(vy,2))