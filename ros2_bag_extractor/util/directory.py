import os
from datetime import datetime

def get_rosbag_file(root_folder='rosbag', choose_rosbag=''):
  '''
  @return db3 absolute path in string
  '''
  default_dir = os.path.join(os.path.expanduser('~'), root_folder)
  if choose_rosbag == '':
    print(f'no such file {default_dir}/{choose_rosbag}')
    return str()
  
  check_file = os.path.join(default_dir, choose_rosbag)
  check_file = os.path.join(check_file, choose_rosbag+"_0.db3")
  if os.path.isfile(check_file):
    return check_file
  else:
    print(f'wrong file name of {check_file}')
    return str()