#!/usr/bin/env python
import rosbag
import matplotlib.pyplot as plt
import numpy as np



def main():
    Analyzer = AnalyzeEffort()
    #path = "/home/leus/rosbag_clean_certain_piece/pushing/2019-12-10-19-31-09.bag"
    #path = "/home/leus/jacobian_bag/2019-12-11-11-33-46.bag"
    #path = "/home/leus/workspace/docs/2019-12-13-17-58-56.bag"
    path = "/home/leus/bag_trafo_jacobi/2019-12-13-17-46-49.bag"


    Analyzer.read_bag(path)
    #Analyzer.transform_jacobi([0,-0.2,0])
    Analyzer.get_small_jacobi(1,3,4)
    #Analyzer.transform_jacobi([1,2,3])
    Analyzer.calculate_force(1,3,4)
    #Analyzer.debug_force(1,3,4)

    
    fig, axs = plt.subplots(8, 1)
    for i in np.arange(0.0,8.0):
        Analyzer.transform_force_radial(i/4*np.pi) #3*np.pi/4
        axs[i].plot(np.transpose(Analyzer.force_radial))
    plt.show()
    


    #Analyzer.plot_data(["velocity","desired"])
    #Analyzer.plot_data(["velocity","actual"])
    Analyzer.plot_data(["effort","actual"])
    #Analyzer.plot_data(["position","error"])


class AnalyzeEffort():
    def __init__(self):
        self.effort = {"larm": {"desired":[],"error":[],"actual":[]},"torso": {"desired":[],"error":[],"actual":[]}}
        self.position = {"larm": {"desired":[],"error":[],"actual":[]},"torso": {"desired":[],"error":[],"actual":[]}}
        self.velocity = {"larm": {"desired":[],"error":[],"actual":[]},"torso": {"desired":[],"error":[],"actual":[]}}
        self.joint_names = []
        self.jacobian = np.zeros([6,8])
        self.small_jacobiT = np.zeros([3,3])
        np.set_printoptions(formatter={'float': '{: 0.3f}'.format})

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
                #self.jacobian[2,:] = self.jacobian[2,:] - (0.07 * self.jacobian[3,:])
                #self.transform_jacobi([0,0,-0.05])
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

    def transform_jacobi(self,r):
        T = np.vstack([np.hstack([np.eye(3), -self.skew(r)]) , np.hstack([np.zeros([3,3]),np.eye(3)])])
        self.jacobian = np.matmul(T,self.jacobian)
        print T
        print self.jacobian

    def skew(self,v):
        v_hat = np.array([[0,-v[2],v[1]], [v[2],0,-v[0]], [-v[1],v[0],0]])
        return v_hat

    def calculate_force(self,i,j,k,least_square=False):
        effortT = np.transpose(self.effort["larm"]["actual"])
        if least_square:
            self.force = np.linalg.lstsq(np.transpose(self.jacobian[:,1:8]),effortT)[0][0:3]
            #self.force = np.linalg.lstsq(np.transpose(self.jacobian),np.vstack([effortT[:,0:10445],np.transpose(self.effort["torso"]["actual"])]))[0]
        else:
            print "else"
            effortT_small = np.array([effortT[i-1],effortT[j-1],effortT[k-1]])
            self.force = np.matmul(np.linalg.inv(self.small_jacobiT),effortT_small)
        fig, axs = plt.subplots(3, 1)
        axs[0].plot(np.transpose(self.force[0]))
        axs[1].plot(np.transpose(self.force[1]))
        axs[2].plot(np.transpose(self.force[2]))
        plt.show()

    def debug_force(self,i,j,k):
        effortT = np.transpose(self.effort["larm"]["actual"])

        #calculate force with least squares
        force = np.linalg.lstsq(np.transpose(self.jacobian[:,1:8]),effortT)[0]
        fig, axs = plt.subplots(3, 1)
        axs[0].plot(np.transpose(force[0]),label='least squares')
        axs[1].plot(np.transpose(force[1]),label='least squares')
        axs[2].plot(np.transpose(force[2]),label='least squares')

        #calculate force with small matrix
        effortT_small = np.array([effortT[i-1],effortT[j-1],effortT[k-1]])
        force = np.matmul(np.linalg.inv(self.small_jacobiT),effortT_small)
        axs[0].plot(np.transpose(force[0]),label='choose small matrix')
        axs[1].plot(np.transpose(force[1]),label='choose small matrix')
        axs[2].plot(np.transpose(force[2]),label='choose small matrix')

        axs[0].legend()
        axs[1].legend()
        axs[2].legend()


        plt.show()


    def transform_force_radial(self,phi):
        r = np.array([np.cos(phi+np.pi/2),np.sin(phi+np.pi/2),0])
        print r
        print np.shape(self.force)
        self.force_radial = np.matmul(r,self.force)



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