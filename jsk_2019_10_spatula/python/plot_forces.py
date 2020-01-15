#!/usr/bin/env python
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import os

task = "transfer"
window = 1

def main():
    #path = "/home/leus/force_clean_dirty_bag/clean8_dirty_speed1000"
    #path = "/home/leus/force_clean_dirty_bag/test"
    fig, axs = plt.subplots(6, 1)

    for label in ["long","longer","shorter"]:#["long","short"]:#["clean","dirty"]:#["clean","dirty","partlydirty"]:
        #path = "/home/leus/force_clean_dirty_bag/clean8_%s_speed4000" % label
        #path = "/home/leus/force_clean_dirty_bag/whole_bowl_%s_sleep05" % label
        path = "/home/leus/force_different_spatula_pos/transfer/%s" % label
        
        first = True
        for doc in os.listdir(path):
            print doc
            force = {"larm":[],"rarm":[]}

            effort = {"larm": {"desired":[],"error":[],"actual":[]},"rarm": {"desired":[],"error":[],"actual":[]}}
            position = {"larm": {"desired":[],"error":[],"actual":[]},"rarm": {"desired":[],"error":[],"actual":[]}}
            velocity = {"larm": {"desired":[],"error":[],"actual":[]},"rarm": {"desired":[],"error":[],"actual":[]}}

            bag = rosbag.Bag("%s/%s" % (path,doc))
            start_ts = None
            stop_ts = None
            ts = []

            for topic, msg, t in bag.read_messages(''):
                if topic == "/endeffector_force":
                    force["larm"].append(msg.larm)
                    force["rarm"].append(msg.rarm)
                    ts.append(t)

                if topic == "/semantic_annotation":
                    if task == "scrape":
                        print msg
                        [av,scrape_type,bowl_position,flag_type] = msg.data.split("_")
                        #if av == "av1wall" and flag_type == "start":
                        if av == "av2wall" and flag_type == "end":
                        #if msg.data == "av1wall_0_1_start":
                            start_ts = t
                        #if av == "av4wall" and flag_type == "end":
                        if av == "av3wall" and flag_type == "end":
                        #if msg.data == "av4wall_3_4_end":
                            stop_ts = t

                    elif task == "transfer":
                        [av,flag_type] = msg.data.split("_")
                        if av == "av5transfer" and flag_type == "start":
                            start_ts = t
                        if av == "av7transfer" and flag_type == "end":
                            stop_ts = t


            ts = np.array(ts)
            start_index = np.argmin(np.abs(ts-start_ts))
            stop_index = np.argmin(np.abs(ts-stop_ts))

            if first:
                plot_force(force,label,start_index,stop_index,fig,axs,True)
                first = False
            else:
                plot_force(force,label,start_index,stop_index,fig,axs,False)
    plt.show()

def mean_filter(signal,window):
    if window == 1:
        return signal
    filtered_signal = 1/float(window) * np.convolve(signal, np.ones([window]))
    return filtered_signal[window:len(filtered_signal)-window]

def plot_force(force,label,start_index,stop_index,fig=None,axs=None,set_label = True):
    print label
    if label == "dirty" or label == "short" or label == "shorter":
        color = "maroon"
    elif label == "partlydirty" or label == "long":
        color = "lightcoral"
    else:
        color = "lightseagreen"

    if fig is None or axs is None:
        fig, axs = plt.subplots(6, 1)

    line0, = axs[0].plot(mean_filter(np.transpose(force["larm"])[0][start_index:stop_index],window),color)
    line1, = axs[1].plot(mean_filter(np.transpose(force["larm"])[1][start_index:stop_index],window),color)
    line2, = axs[2].plot(mean_filter(np.transpose(force["larm"])[2][start_index:stop_index],window),color)

    line3, = axs[3].plot(mean_filter(np.transpose(force["rarm"])[0][start_index:stop_index],window),color)
    line4, = axs[4].plot(mean_filter(np.transpose(force["rarm"])[1][start_index:stop_index],window),color)
    line5, = axs[5].plot(mean_filter(np.transpose(force["rarm"])[2][start_index:stop_index],window),color)

    if set_label:
        line0.set_label(label)
        line1.set_label(label)
        line2.set_label(label)
        line3.set_label(label)
        line4.set_label(label)
        line5.set_label(label)

    axs[0].set_ylabel("Fx larm")
    axs[1].set_ylabel("Fy larm")
    axs[2].set_ylabel("Fz larm")
    axs[3].set_ylabel("Fx rarm")
    axs[4].set_ylabel("Fy rarm")
    axs[5].set_ylabel("Fz rarm")

    axs[0].legend()
    axs[1].legend()
    axs[2].legend()
    axs[3].legend()
    axs[4].legend()
    axs[5].legend()
    

if __name__ == "__main__":
    main()
