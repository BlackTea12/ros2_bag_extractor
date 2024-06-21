import os
import yaml, re

class MetaDataModify():
  '''
  @brief If ros2 bag folder and metadata is not correct, it will modify meta file \
  in a right way automatically
  '''
  def __init__(self, ros2bag_folder:str):
    self.meta_file = os.path.join(ros2bag_folder, 'metadata.yaml')
    print("[Initialized] "+ self.meta_file)

  def modify_yaml(self):
    if not os.path.exists(self.meta_file):
      print(f"file does not exist: {self.meta_file}")
      return
    
    # open data
    with open(self.meta_file, 'r') as file:
      self.yaml_file = yaml.safe_load(file)
    
    # save modified file
    self._modify_dict(self.yaml_file)
    with open(self.meta_file, 'w') as file:
        yaml.dump(self.yaml_file, file)

  def _modify_dict(self, data:yaml):
    for i, ele in enumerate(data['rosbag2_bagfile_information']['topics_with_message_count']):
      org_type = data['rosbag2_bagfile_information']['topics_with_message_count'][i]['topic_metadata']['type']
      pkg_name, pkg_msg, msg_type = re.split(r'[./]', org_type)
      org_type = pkg_name+"/"+pkg_msg+"/"+msg_type
      data['rosbag2_bagfile_information']['topics_with_message_count'][i]['topic_metadata']['type'] = org_type

# def main():
#   test = MetaDataModify('/home/hd/rosbag/after_turtle/2024-03-25-11')
#   test.modify_yaml()

# if __name__ == '__main__':
#   main()