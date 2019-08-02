import sys, re

file = open(r'/home/roboway/catkin_roboway/src/bringup/param/amcl.xml', 'a+')

contains = re.sub(r'"initial_pose_x" 			value="([0-9]{1,}[.][0-9]*)', r'"initial_pose_x" 			value="' + str(sys.argv[1]), file.read())
contains = re.sub(r'"initial_pose_y" 			value="([0-9]{1,}[.][0-9]*)', r'"initial_pose_y" 			value="' + str(sys.argv[2]), contains)
contains = re.sub(r'"initial_pose_a" 			value="([0-9]{1,}[.][0-9]*)', r'"initial_pose_a" 			value="' + str(sys.argv[3]), contains)

file.seek(0)
file.truncate()
file.write(contains)
file.close()