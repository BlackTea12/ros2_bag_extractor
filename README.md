# ❤️‍🔥rosbag2 data extractor package
<div align="right">

  <a href="">![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-green)</a>
  <a href="">![ROS2](https://img.shields.io/badge/ROS2-humble-blue)</a>

</div>

In this package, you can extract data into python script. <br />
Following the steps below, you will be able to add published data type into the script.

## How to add ros2 messages to convert in python data type
1. Add your python data type in the folder **'/type'**

2. In **'/bag_parser/object.py'**, add your function to connect with **'/type'**

3. After adding all function and types, please link with **'function_array.py'**.
   
  ``` python
  # dictionary
  {"message type": "function name"}
  ```

## How to use
``` python
parsed_data = Bag2FileParser({your_rosbag2_directory})
data_list = ObjectType(parsed_data.get_messages('/plan'))
db_Path = data_list.get_data('Path')
```
## Test code
``` shell
ros2 run ros2_bag_extractor test
```
