# ❤️‍🔥rosbag2 data extractor package
<div align="right">

  <a href="">![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-green)</a>
  <a href="">![ROS2](https://img.shields.io/badge/ROS2-humble-blue)</a>

</div>

## How to add ros2 messages to convert in python data type
1. Add your python data type in the below file

    /type

2. In **/bag_parser/object.py**, add your function to connect with **/type**


## How to use

    parsed_data = Bag2FileParser({your_rosbag2_directory})
    data_list = ObjectType(parsed_data.get_messages('/plan'))
    db_Path = data_list.get_data('Path')

