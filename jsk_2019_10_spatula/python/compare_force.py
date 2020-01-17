#!/usr/bin/env python
import rospy
from jsk_2019_10_spatula.msg import Force
import numpy as np
from std_msgs.msg import String, Float64
import json
import matplotlib.pyplot as plt


path_bag = "/home/leus/force_different_spatula_pos/test"
path_json = "%s/force.json" % path_bag

offline = True


def main():
	Compare = CompareForce()
	rospy.init_node('compare_force', anonymous=True)
	rospy.Subscriber("endeffector_force", Force, Compare.callback_force)
	rospy.Subscriber("semantic_annotation", String, Compare.callback_annotation)
	#rospy.spin()
	#for offline tests
	if offline:
		Compare.action = "av5transfer"
		Compare.read_force()

		Compare.force["larm"] = [[1,1,1,2,3,4],[1,1,1,2,3,4],[1,1,1,2,3,4]]
		Compare.force["rarm"] = [[1,1,1,2,3,4],[1,1,1,2,3,4],[1,1,1,2,3,4]]
		Compare.compare()


class CompareForce:

	def __init__(self):
		self.pub = rospy.Publisher("force_gain", Float64, queue_size=2)
		self.force = {}
		self.force_des = {}
		self.force_des["larm"] = dict()
		self.force_des["rarm"] = dict()
		self.force_dict = {}
		self.label = []
		self.n_exp = {}
		self.n_t = {}
		self.read_json(path_json)
		self.time = 0 #is used to count up the time during one task to synchronize
		self.window = 6 #the time window over which th mean is calculated
		self.action_status = 0
		#maybe save this dicitonary in a json file in case it becomes larger
		self.action_force = {"av5transfer":{"arm":"larm","direction":2}} #dictionary mapping the relevant force to an action
		self.action = None

	def callback_force(self,msg):
		if self.action_status == 0:
			return True
		self.time = self.time + 1
		self.force["larm"].append(msg.larm)
		self.force["rarm"].append(msg.rarm)
		if len(self.force) == self.window:
			self.compare()

	def callback_annotation(self,msg):
		[self.action,flag] = msg.data.split("_")
		if flag == "start":
			self.action_status = 1
			read_force()
			self.time = 0
		if flag == "end":
			self.action_status = 0

	def read_json(self,path):
		f = open(path,"r")
		force_dict_str = f.read()
		f.close()
		exec("self.force_dict = %s" % force_dict_str)
		print self.force_dict.keys()
		self.label = np.array(self.force_dict["label"]) #indexing with i, the index of exp
		self.n_exp = self.force_dict["n_exp"] #indexing with action
		self.n_t = self.force_dict["n_t"] #indexing with action

	def read_force(self):
		force_des_larm = np.array(self.force_dict["force"][self.action]["larm"])[:,:,0:len(self.label)]
		force_des_rarm = np.array(self.force_dict["force"][self.action]["rarm"])[:,:,0:len(self.label)]

		force_des_larm_up = force_des_larm[:,:,self.label == "longer"]
		force_des_larm_down = force_des_larm[:,:,self.label == "shorter"]

		force_des_rarm_up = force_des_rarm[:,:,self.label == "longer"]
		force_des_rarm_down = force_des_rarm[:,:,self.label == "shorter"]

		self.force_des["larm"]["up"] =  force_des_larm_up
		self.force_des["larm"]["down"] = force_des_larm_down
		self.force_des["rarm"]["up"] =  force_des_rarm_up
		self.force_des["rarm"]["down"] = force_des_rarm_down

	def compare(self):
		if self.action not in self.action_force.keys():
			return True
		arm = self.action_force[self.action]["arm"]
		direction = self.action_force[self.action]["direction"]
		actual = self.mean_filter(self.force[arm][direction]) 
		up = self.force_des[arm]["up"][self.time : self.time + self.window, direction, :] #last dimension should be handeled in read_force -> no : necessary anymore
		down = self.force_des[arm]["down"][self.time : self.time + self.window, direction, :]
		#averaging over time first 
		up_mean = []
		down_mean = []
		for i in range(np.shape(up)[1]):
			up_mean.append(self.mean_filter(up[:,i]))
			down_mean.append(self.mean_filter(down[:,i]))
		up_mean = np.array(up_mean)
		down_mean= np.array(down_mean)
		#averaging/maximum/minimum over experiments next
		up_dict = {}
		down_dict = {}
		up_dict["max"] = np.max(up_mean)
		up_dict["min"] = np.min(up_mean)
		up_dict["mean"] = np.mean(up_mean)
		down_dict["max"] = np.max(down_mean)
		down_dict["min"] = np.min(down_mean)
		down_dict["mean"] = np.mean(down_mean)
		gain = self.compute_gain(up_dict,down_dict,actual)
		print gain #gain should be published
		self.pub.publish(gain)


	def mean_filter(self,signal):
		if len(signal) != self.window:
			print "CAUTION  applying mean filter with wrong size!"
		mean = sum(signal)/len(signal)
		return mean

	def compute_gain(self,up,down,actual):
		if abs(actual) <= abs(down["max"]):
			return 0.0 #no gain needed
		d1 = up["mean"] - actual
		d2 = up["mean"] - down["mean"]
		gain = abs(d1/d2)
		return gain

	def plot_signal(self,signal,color = "royalblue"):
		"""
		signal: np array of size t x n
		plots each of the n signals in a seperate subplot
		"""
		n = np.shape(signal)[1]
		fig, axs = plt.subplots(n, 1)
		for i in range(n):
			axs[i].plot(signal[:,i],color)
		plt.show()

if __name__ == "__main__":
    main()


