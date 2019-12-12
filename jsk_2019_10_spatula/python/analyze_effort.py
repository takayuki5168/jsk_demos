#!/usr/bin/env python
import rosbag
import matplotlib.pyplot as plt
import numpy as np



def main():
    Analyzer = AnalyzeEffort()
    #path = "/home/leus/rosbag_clean_certain_piece/pushing/2019-12-10-19-31-09.bag"
    path = "/home/leus/jacobian_bag/2019-12-11-11-33-46.bag"
    #path = "/home/leus/workspace/docs/2019-12-12-18-36-03.bag"
    Analyzer.read_bag(path)
    Analyzer.get_small_jacobi(1,3,4)
    Analyzer.calculate_force(1,3,4)
    #Analyzer.plot_data(["velocity","desired"])
    #Analyzer.plot_data(["velocity","actual"])
    Analyzer.plot_data(["effort","actual"])
    Analyzer.plot_data(["position","error"])


class AnalyzeEffort():
    def __init__(self):
        self.effort = {"larm": {"desired":[],"error":[],"actual":[]},"torso": {"desired":[],"error":[],"actual":[]}}
        self.position = {"larm": {"desired":[],"error":[],"actual":[]},"torso": {"desired":[],"error":[],"actual":[]}}
        self.velocity = {"larm": {"desired":[],"error":[],"actual":[]},"torso": {"desired":[],"error":[],"actual":[]}}
        self.joint_names = []
        self.jacobian = np.zeros([6,8])
        self.small_jacobiT = np.zeros([3,3])

    def read_bag(self,path):
        self.bag = rosbag.Bag(path)
        first = True
        first_jacobi = True
        for topic, msg, t in self.bag.read_messages(''):
            if topic == "/scrape_left_jacobian" and first_jacobi:
                self.jacobian[0,:] = msg.x
                self.jacobian[1,:] = msg.y
                self.jacobian[2,:] = msg.z
                self.jacobian[3,:] = msg.roll
                self.jacobian[4,:] = msg.pitch
                self.jacobian[5,:] = msg.yaw
                print self.jacobian
                self.jacobian[2,:] = self.jacobian[2,:] - (0.07 * self.jacobian[3,:])
                first_jacobi = False

            elif topic == "/torso_controller/state":
                self.effort["torso"]["desired"].append(msg.desired.effort)
                self.effort["torso"]["error"].append(msg.error.effort)
                self.effort["torso"]["actual"].append(msg.actual.effort)
                self.position["torso"]["desired"].append(msg.desired.positions)
                self.position["torso"]["error"].append(msg.error.positions)
                self.position["torso"]["actual"].append(msg.actual.positions)
                self.velocity["torso"]["desired"].append(msg.desired.velocities)
                self.velocity["torso"]["error"].append(msg.error.velocities)
                self.velocity["torso"]["actual"].append(msg.actual.velocities)

            elif topic == "/l_arm_controller/state":
                self.effort["larm"]["desired"].append(msg.desired.effort)
                self.effort["larm"]["error"].append(msg.error.effort)
                self.effort["larm"]["actual"].append(msg.actual.effort)
                if first:
                    self.joint_names = msg.joint_names
                    first = False
                self.position["larm"]["desired"].append(msg.desired.positions)
                self.position["larm"]["error"].append(msg.error.positions)
                self.position["larm"]["actual"].append(msg.actual.positions)
                self.velocity["larm"]["desired"].append(msg.desired.velocities)
                self.velocity["larm"]["error"].append(msg.error.velocities)
                self.velocity["larm"]["actual"].append(msg.actual.velocities)

    def get_small_jacobi(self,i,j,k):
        self.small_jacobiT[0,:] = np.transpose(self.jacobian)[i,0:3]
        self.small_jacobiT[1,:] = np.transpose(self.jacobian)[j,0:3]
        self.small_jacobiT[2,:] = np.transpose(self.jacobian)[k,0:3]
        print self.small_jacobiT
        print np.linalg.inv(self.small_jacobiT)

    def calculate_force(self,i,j,k):
        effortT = np.transpose(self.effort["larm"]["actual"])
        effortT_small = np.array([effortT[i-1],effortT[j-1],effortT[k-1]])
        self.force = np.matmul(np.linalg.inv(self.small_jacobiT),effortT_small)
        fig, axs = plt.subplots(3, 1)
        axs[0].plot(np.transpose(self.force[0]))
        axs[1].plot(np.transpose(self.force[1]))
        axs[2].plot(np.transpose(self.force[2]))
        plt.show()

    def plot_data(self,datatype):
        fig, axs = plt.subplots(8, 1)
        
        exec("axs[0].plot(self.%s['torso'][datatype[1]])" % datatype[0])
        axs[0].set_xlabel('time',fontsize="small")
        axs[0].set_ylabel("torso",fontsize="small")
        axs[0].grid(True)
        #plot all arm joints
        for i, name in enumerate(self.joint_names):
            exec("axs[i+1].plot(np.transpose(self.%s['larm'][datatype[1]])[i])" % datatype[0])
            axs[i+1].set_xlabel('time',fontsize="small")
            axs[i+1].set_ylabel(name,fontsize="small")
            axs[i+1].grid(True)
        fig.suptitle("%s %s" % (datatype[0],datatype[1]))
        plt.show()


if __name__ == "__main__":
    main()