
from ros2_bag_extractor.util import directory as dir
from ros2_bag_extractor.bag_parser.parser import Bag2FileParser
from ros2_bag_extractor.bag_parser.object import ObjectType

def main():
  # rosbag2 recorded directory
  db_file_name = dir.get_rosbag_file('ros2bags/ours_obs', 'rosbag2_2024_11_14-12_21_39')

  if db_file_name:
    parsed_data = Bag2FileParser(db_file_name)
    msgs = parsed_data.get_messages('/plan')
    data_list = ObjectType(msgs)
    result = data_list.get_data('Path')
    print(result[0].poses[0].x)
    print("\x1b[38;20msuccessfully finished!\x1b[0m")
  else:
    print('\x1b[31;20mwrong access!\x1b[0m')

if __name__ == '__main__':
  main()