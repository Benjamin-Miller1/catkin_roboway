import sys, re
file = open(r'../launch/amcl.launch', 'a+')
contains = re.sub("map/(.*).yaml", "map/" + str(sys.argv[1]) + ".yaml", file.read())

file.seek(0)
file.truncate()
file.write(contains)
file.close()

file = open(r'../launch/move_base.launch', 'a+')
contains = re.sub("map/(.*)_modify.yaml", "map/" + str(sys.argv[1]) + "_modify.yaml", file.read())
contains = re.sub("map/(.*)_path.json", "map/" + str(sys.argv[1]) + "_path.json", contains)
file.seek(0)
file.truncate()
file.write(contains)
file.close()

