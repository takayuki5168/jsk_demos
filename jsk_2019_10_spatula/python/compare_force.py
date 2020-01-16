#!/usr/bin/env python
import rospy
from jsk_2019_10_spatula.msg import Force
import numpy as np
from std_msgs.msg import String, Float64
import json


window = 10
path_bag = "/home/leus/force_different_spatula_pos/test"
path_json = "%s/force.json" % path_bag



def main():
	Compare = CompareForce()
	rospy.init_node('compare_force', anonymous=True)
	rospy.Subscriber("endeffector_force", Force, Compare.callback_force)
	rospy.Subscriber("semantic_annotation", String, Compare.callback_annotation)
	#rospy.spin()

	Compare.read_force("av5transfer")

	Compare.force["larm"] = [[1,1,1,2,3,4],[1,1,1,2,3,4],[1,1,1,2,3,4]]
	Compare.force["rarm"] = [[1,1,1,2,3,4],[1,1,1,2,3,4],[1,1,1,2,3,4]]
	Compare.xyz()


class CompareForce:

	def __init__(self):
		self.pub = rospy.Publisher("force_gain", Force, queue_size=2)
		self.force = {}
		self.force_des = {}
		self.force_dict = {}
		self.label = []
		self.n_exp = {}
		self.n_t = {}
		self.read_json(path_json)
		self.time = 0 #is used to count up the time during one task to synchronize
		self.window = 10 #the time window over which th mean is calculated

	def callback_force(self,msg):
		self.time = self.time + 1
		self.force["larm"].append(msg.larm)
		self.force["rarm"].append(msg.rarm)
		if len(self.force) == self.window and self.force_des:
			self.xyz()

	def callback_annotation(self,msg):
		[action,flag] = msg.data.split("_")
		if flag == "start":
			read_force(action)
			self.time = 0

	def read_json(self,path):
		f = open(path,"r")
		force_dict_str = f.read()
		f.close()
		exec("self.force_dict = %s" % force_dict_str)
		print self.force_dict.keys()
		self.label = self.force_dict["label"] #indexing with i, the index of exp
		self.n_exp = self.force_dict["n_exp"] #indexing with action
		self.n_t = self.force_dict["n_t"] #indexing with action

	def read_force(self,action):
		self.force_des["larm"] = np.array(self.force_dict["force"][action]["larm"])
		self.force_des["rarm"] = np.array(self.force_dict["force"][action]["rarm"])

	def xyz(self):
		arm = "larm"
		direction = 2 #in this case use the z direction
		actual = self.mean_filter(self.force[arm][direction]) 
		print np.shape(self.force_des[arm])
		print type(self.force_des[arm])
		up = self.force_des[arm][self.time : self.time + self.window, direction, :] #last dimension should be handeled in read_force -> no : necessary anymore
		down = self.force_des[arm][self.time : self.time + self.window, direction, :]
		#gain = self.compute_gain(up,down,actual)


	def mean_filter(self,signal):
		mean = sum(signal)/len(signal)
		return mean

	def compute_gain(self,up,down,actual):
		d1 = up - actual
		d2 = up - down
		gain = d1/d2
		return gain


if __name__ == "__main__":
    main()


