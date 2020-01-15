import json
import rosbag


path_bag = ""
path_json = ".json"
force_dict = {}

bag = rosbag.Bag("%s/%s" % (path,doc))

for doc in os.listdir(path_bag):

    for topic, msg, t in bag.read_messages(''):

	    if topic == "/endeffector_force":
	        force["larm"].append(msg.larm)
	        force["rarm"].append(msg.rarm)
	        ts.append(t)

	    if topic == "/semantic_annotation":
	        [action,flag_type] = msg.data.split("_")
	        if flag_type == "start":
	        	force["larm"] = []
	        	force["ram"] = []
	        if flag_type == "end":
	        	force_dict{action} = {"larm":force["larm"],"rarm":force["rarm"]}


json = json.dumps(force_dict)
f = open("path_json","w")
f.write(json)
f.close()


#for validation
f = open("path_json","r")
test = f.read()
f.close()
exec("dict2 = %s" % test)
print dict2