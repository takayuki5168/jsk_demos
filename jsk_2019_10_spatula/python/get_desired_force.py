import json
import rosbag
import os
import numpy as np

path_bag = "/home/leus/force_different_spatula_pos/test"
path_json = "%s/force.json" % path_bag

data = {}
force_dict = {}
indices = {}
n_t_dict = {}
labels = []


def main():
    read_bag()
    save_json()
    

def read_bag():
    force = {}
    force["larm"] = []
    force["rarm"] = []
    for label in ["long","longer","shorter"]:
        path_bag = "/home/leus/force_different_spatula_pos/transfer/%s" % label

        for doc in os.listdir(path_bag):
            if doc.split(".")[-1] != "bag":
                continue
            bag = rosbag.Bag("%s/%s" % (path_bag,doc))
            print "reading document %s" % doc
            labels.append(label)

            ts = []
            start_ts = {}
            stop_ts = {}
            force["larm"] = []
            force["rarm"] = []
            for topic, msg, t in bag.read_messages(''):

                if topic == "/endeffector_force":
                    force["larm"].append(msg.larm)
                    force["rarm"].append(msg.rarm)
                    ts.append(t)

                if topic == "/semantic_annotation":
                    [action,flag_type] = msg.data.split("_")
                    if flag_type == "start":
                        start_ts[action] = t
                    if flag_type == "end":
                        stop_ts[action] = t

            split_sequence(ts,force,start_ts,stop_ts)

def split_sequence(ts,force,start_ts,stop_ts):
    force_action = {}
    force_action["larm"] = []
    force_action["rarm"] = []
    for action in start_ts.keys():
        if action not in stop_ts.keys():
            continue
        ts = np.array(ts)
        start_index = np.argmin(np.abs(ts-start_ts[action]))
        stop_index = np.argmin(np.abs(ts-stop_ts[action]))
        force_action["larm"] = force["larm"][start_index:stop_index]
        force_action["rarm"] = force["rarm"][start_index:stop_index]

        if action in force_dict.keys():
            update_force_dict(action,force_action)
        else:
            n_t = np.shape(force_action["larm"])[0]
            force_array = np.zeros([int(round(1.5*n_t)) ,np.shape(force_action["larm"])[1],20])
            force_dict[action] = {}
            force_dict[action]["larm"] = force_array
            force_dict[action]["rarm"] = force_array
            indices[action] = 0
            n_t_dict[action] = []
            update_force_dict(action,force_action)                      

def update_force_dict(action,force):
    for arm in ["larm","rarm"]:
        force_array = np.array(force_dict[action][arm])
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