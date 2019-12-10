#!/usr/bin/env python
import rosbag
import matplotlib.pyplot as plt
import numpy as np



def main():
    Analyzer = AnalyzeEffort()
    path = "/home/leus/rosbag_clean_certain_piece/pushing/2019-12-10-19-31-09.bag"
    Analyzer.read_bag(path)
    Analyzer.plot_data(["effort","actual"])
    Analyzer.plot_data(["position","error"])


class AnalyzeEffort():
    def __init__(self):
        self.effort = {"desired":[],"error":[],"actual":[]}
        self.position = {"desired":[],"error":[],"actual":[]}
        self.velocity = {"desired":[],"error":[],"actual":[]}
        self.joint_names = []
        self.jacobian = np.zeros([6,8])

    def read_bag(self,path):
        self.bag = rosbag.Bag(path)
        first = True
        for topic, msg, t in self.bag.read_messages(''):
            if topic == "/scrape_left_jacobian":
                self.jacobian[0,:] = msg.x
                self.jacobian[1,:] = msg.y
                self.jacobian[2,:] = msg.z
                self.jacobian[3,:] = msg.roll
                self.jacobian[4,:] = msg.pitch
                self.jacobian[5,:] = msg.yaw
                print self.jacobian

            if topic != "/l_arm_controller/state":
                continue
            self.effort["desired"].append(msg.desired.effort)
            self.effort["error"].append(msg.error.effort)
            self.effort["actual"].append(msg.actual.effort)
            if first:
                self.joint_names = msg.joint_names
                first = False
            self.position["desired"].append(msg.desired.positions)
            self.position["error"].append(msg.error.positions)
            self.position["actual"].append(msg.actual.positions)
            self.velocity["desired"].append(msg.desired.velocities)
            self.velocity["error"].append(msg.error.velocities)
            self.velocity["actual"].append(msg.actual.velocities)


    def plot_data(self,datatype):
        fig, axs = plt.subplots(7, 1)
        for i, name in enumerate(self.joint_names):
            exec("axs[i].plot(np.transpose(self.%s[datatype[1]])[i])" % datatype[0])
            axs[i].set_xlabel('time',fontsize="small")
            axs[i].set_ylabel(name,fontsize="small")
            axs[i].grid(True)
        fig.suptitle("%s %s" % (datatype[0],datatype[1]))
        plt.show()


if __name__ == "__main__":
    main()