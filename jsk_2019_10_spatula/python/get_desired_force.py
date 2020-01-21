import json
import rosbag
import os
import numpy as np


#path_bag = "/home/leus/force_different_spatula_pos/test"
path = "/home/leus/force_feedack_exp_19_01"
path_json = "%s/force.json" % path

data = {}
force_dict = {}
ts_dict = {}
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
    n_exp = 0
    for label in ["short","long","long_adapted","short_2","short_3","long_2","short_4"]:
        path_bag = "%s/%s" % (path,label)
        n_exp = n_exp + len(os.listdir(path_bag))

    for label in ["short","long","long_adapted","short_2","short_3","long_2","short_4"]:
        #path_bag = "/home/leus/force_different_spatula_pos/transfer/%s" % label
        path_bag = "%s/%s" % (path,label)

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
                    ts.append(t.to_sec())

                if topic == "/semantic_annotation":
                    [action,flag_type] = msg.data.split("_")
                    if flag_type == "start":
                        start_ts[action] = t.to_sec()
                        print action
                    if flag_type == "end":
                        stop_ts[action] = t.to_sec()

            split_sequence(ts,force,start_ts,stop_ts,n_exp)

def split_sequence(ts,force,start_ts,stop_ts,n_exp):
    force_action = {}
    force_action["larm"] = []
    force_action["rarm"] = []
    for action in start_ts.keys():
        if action not in stop_ts.keys():
            continue
        ts = np.array(ts)
        print "ts shape array"
        print np.shape(ts)
        start_index = np.argmin(np.abs(ts-start_ts[action]))
        stop_index = np.argmin(np.abs(ts-stop_ts[action]))
        ts_list_action = ts[start_index:stop_index] - ts[start_index]
        force_action["larm"] = force["larm"][start_index:stop_index]
        force_action["rarm"] = force["rarm"][start_index:stop_index]

        if action in force_dict.keys():
            update_force_dict(action,force_action,ts_list_action)
        else:
            n_t = np.shape(force_action["larm"])[0]
            force_array = np.zeros([int(round(1.5*n_t)) ,np.shape(force_action["larm"])[1],n_exp])
            ts_array = np.zeros([int(round(1.5*n_t)),n_exp])
            print "-------------"
            print np.shape(ts_array)
            force_dict[action] = {}
            force_dict[action]["larm"] = force_array
            force_dict[action]["rarm"] = force_array
            indices[action] = 0
            n_t_dict[action] = []
            ts_dict[action] = ts_array
            update_force_dict(action,force_action,ts_list_action)                      

def update_force_dict(action,force,ts):
    for arm in ["larm","rarm"]:
        force_array = np.array(force_dict[action][arm])
        n_t = np.shape(force[arm])[0]
        force_array[0:n_t,0:np.shape(force[arm])[1],indices[action]] = force[arm]
        force_dict[action][arm] = force_array.tolist()
    print "ts shaope list"
    print np.shape(ts)
    ts_array = np.array(ts_dict[action])
    print ",,,,,,,,,,,,,,,,,,,"
    print np.shape(ts_array)
    ts_array[0:n_t,indices[action]] = ts
    ts_dict[action] = ts_array.tolist()
    print "ts shape dict"
    print np.shape(ts_dict[action])
    n_t_dict[action].append(n_t)
    indices[action] = indices[action] + 1

def save_json():
    data["force"] = force_dict
    data["n_exp"] = indices
    data["label"] = labels
    data["n_t"] = n_t_dict
    data["ts"] = ts_dict
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