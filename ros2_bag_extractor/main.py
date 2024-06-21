
from ros2_bag_extractor.util import directory as dir
from ros2_bag_extractor.bag_parser.parser import Bag2FileParser
from ros2_bag_extractor.bag_parser.object import ObjectType
def main():
  db_file_name = dir.get_rosbag_file('rosbag', 'rosbag2_2024_06_21-09_53_33')

  if db_file_name:
    parsed_data = Bag2FileParser(db_file_name)
    data_list = ObjectType(parsed_data.get_messages('/amcl_pose'))
    result = data_list.get_data('MarkerArray')
    print(result[0][0])
    print(result[0][1])
    print("successfully finished!")
  else:
    print('wrong access!')

if __name__ == '__main__':
  main()