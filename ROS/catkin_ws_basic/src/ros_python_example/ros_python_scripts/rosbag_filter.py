import os
import sys

input_name = sys.argv[1]
output_name = input_name[:-4] + "_out" + input_name[-4:]

rosbag_filter_cmd = "rosbag filter " + input_name + \
    " " + output_name + " \"t.to_sec() >= 1616643024.10\""

print(rosbag_filter_cmd)
os.system(rosbag_filter_cmd)


import os

for file_name in os.listdir("./", ):
    if os.path.splitext(file_name)[1] == '.bag':
        print(file_name)
        rosbag_info_cmd = "rosbag info " + file_name
        os.system(rosbag_info_cmd)