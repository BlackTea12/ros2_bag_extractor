from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from ros2_bag_extractor.type.geometry_msgs import convertPoseStamped, convertPoseWithCovariance
from ros2_bag_extractor.type.nav_msgs import convertPath

AVAILABLE_DATA_TYPE = ["Path", "MarkerArray", "PoseStamped", "PoseWithCovarianceStamped"]

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
      # print("splitting data and timestamp succeed!")
    else:
      print("\x1b[31;1mNo data to initialize!\x1b[0m")
  
  def get_data(self, data_type:str):
    '''
    @data_type: what type of the topic is
    '''
    if data_type == AVAILABLE_DATA_TYPE[0]:
      return self._get_PathDiffList(self.org)
    elif data_type == AVAILABLE_DATA_TYPE[1]:
      return self._get_VisMarkerArrayPoseList(self.org)
    elif data_type == AVAILABLE_DATA_TYPE[2]:
      return self._get_PoseStampedList(self.org)
    elif data_type == AVAILABLE_DATA_TYPE[3]:
      return self._get_PoseWithCovarianceStamped(self.org)
    else:
      print(f"\x1b[31;20mno matching type for {data_type}...\x1b[0m")
      print(f"available types are {AVAILABLE_DATA_TYPE}")

    return
  
  def _get_PathDiffList(self, data : list):
    '''
    @data: list of (timestamp, nav_msgs.msg Path)
    @brief: compares Path data and only save when it differs
    '''
    result = [convertPath(data[0][1], data[0][0])]
    if len(data) == 1:
      return result
    for time, path in data:
      if not self._isPathSame(path, result[-1]):
        result.append(convertPath(path, time))
    return result

  def _isPathSame(self, path1:Path, path2):
    if len(path1.poses) == len(path2.poses):
      if path1.poses[0].pose.position.x == path2.poses[0].x \
        and path1.poses[0].pose.position.y == path2.poses[0].y:
        return True
    return False
  
  def _get_PoseStampedList(self, data:list):
    result = []
    for time, posestamp in data:
      result.append(convertPoseStamped(posestamp, time))
    return result
  
  def _get_PoseWithCovarianceStamped(self, data:list):
    result = []
    for time, posestampcov in data:
      result.append(convertPoseWithCovariance(posestampcov,time))
    return result

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
  