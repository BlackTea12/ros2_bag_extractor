import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

'''
@brief Extracts data inside ros2 bag.
'''
class Bag2FileParser():
  def __init__(self, bag_file : str):
    '''
    @bag_file: absolute directory of ros2 bag
    '''

    print(f"parsing data in : {bag_file}")

    try:
      self.conn = sqlite3.connect(bag_file)
      self.cursor = self.conn.cursor()
      ## create a message type map
      topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()

      # {'/plan': 'nav_msgs/msg/Path', '/robot/odom': 'nav_msgs/msg/Odometry'}
      self.topic_type = {name_of:type_of for id_of, name_of,type_of in topics_data}

      # {'/plan': 1, '/robot/odom': 2}
      self.topic_id = {name_of:id_of for id_of, name_of, type_of in topics_data}

      # {'/plan': <class 'nav_msgs.msg._path.Path'>, '/robot/odom': <class 'nav_msgs.msg._odometry.Odometry'>}
      self.topic_msg_message = {name_of:get_message(type_of) for id_of, name_of, type_of in topics_data}
    
    except sqlite3.Error as e:
      print(e)
    
  def __del__(self):
    try:
      self.conn.close()
    except sqlite3.Error as e:
      print(e)
      return

  def get_messages(self, topic_name : str):
    """
    @topic_name: actual subscribed topic name
    @return [(timestamp0, message0), (timestamp1, message1), ...]
    """
    try:
      topic_id = self.topic_id[topic_name]
    except Exception as e:
      print(e)
      return []
    rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
    return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]