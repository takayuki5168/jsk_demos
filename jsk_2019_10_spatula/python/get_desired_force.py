import json
import rosbag
import os
import numpy as np

path_bag = "/home/leus/force_different_spatula_pos/test"
path_json = "%s/force.json" % path_bag



def main():
    read_bag()
    save_json()
    validate_json()

data = {}
force_dict = {}
indices = {}
force = {}
n_t_dict = {}
labels = []
force["larm"] = []
force["rarm"] = []


def read_bag():
    label = "long"

    for doc in os.listdir(path_bag):
        if doc.split(".")[-1] != "bag":
            continue
        bag = rosbag.Bag("%s/%s" % (path_bag,doc))

        labels.append(label)

        for topic, msg, t in bag.read_messages(''):

            if topic == "/endeffector_force":
                force["larm"].append(msg.larm)
                force["rarm"].append(msg.rarm)
                #print "debug"
                #print np.shape(force["larm"])
                #print np.shape(force["rarm"])

            if topic == "/semantic_annotation":
                [action,flag_type] = msg.data.split("_")
                if flag_type == "start":
                    force["larm"] = []
                    force["rarm"] = []
                if flag_type == "end":
                    if action in force_dict.keys():
                        update_force_dict(action,force)
                    else:
                        print np.shape(force["larm"])
                        print np.shape(force["rarm"])
                        #force_array = np.zeros([np.shape(force["larm"])[0],np.shape(force["larm"])[1],20])
                        n_t = np.shape(force["larm"])[0]
                        force_array = np.zeros([1.5*n_t ,np.shape(force["larm"])[1],20])
                        print np.shape(force_array)
                        force_dict[action] = {}
                        force_dict[action]["larm"] = force_array
                        force_dict[action]["rarm"] = force_array
                        print np.shape(force_dict[action]["larm"])
                        indices[action] = 0
                        n_t_dict[action] = []
                        update_force_dict(action,force)
                        

def update_force_dict(action,force):
    for arm in ["larm","rarm"]:
        force_array = np.array(force_dict[action][arm])
        print np.shape(force_array)
        print np.shape(force[arm])
        print "here"
        n_t = np.shape(force[arm])[0]
        force_array[0:n_t,0:np.shape(force[arm])[1],indices[action]] = force[arm]
        force_dict[action][arm] = force_array.tolist()

    n_t_dict[action].append(n_t)
    indices[action] = indices[action] + 1

def save_json():
    data["force"] = force_dict
    data["n_exp"] = indices
    data["label"] = labels
    data["n_t"] = n_t_dict
    json_dict = json.dumps(data)
    f = open(path_json,"w")
    f.write(json_dict)
    f.close()

def validate_json():
    #for validation
    f = open(path_json,"r")
    test = f.read()
    f.close()
    exec("dict2 = %s" % test)
    print dict2.keys()


if __name__ == "__main__":
    main()