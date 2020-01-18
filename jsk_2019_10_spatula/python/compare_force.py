#!/usr/bin/env python
import rospy
from jsk_2019_10_spatula.msg import Force
import numpy as np
from std_msgs.msg import String, Float64
import json
import matplotlib.pyplot as plt


path_json = "/home/leus/force_different_spatula_pos/test/force.json"
offline = False
debug = True

def main():
	Compare = CompareForce()

	#for offline tests
	if offline:
		Compare.action = "av5transfer"
		Compare.read_force()

		Compare.force["larm"] = np.transpose([[1,1,1,2,3,4,5],[1,1,1,2,3,4,5],[1,1,1,2,3,4,5]])
		Compare.force["rarm"] = np.transpose([[1,1,1,2,3,4,5],[1,1,1,2,3,4,5],[1,1,1,2,3,4,5]])
		Compare.compare()
		Compare.plot_save_up_down()
	else:
		rospy.init_node('compare_force', anonymous=True)
		rospy.Subscriber("endeffector_force", Force, Compare.callback_force)
		rospy.Subscriber("semantic_annotation", String, Compare.callback_annotation)
		rospy.spin()

class CompareForce:

	def __init__(self):
		print "init compare force"
		#######################
		###general parameter###
		#######################
		self.window = 7 #the time window over which th mean is calculated
		#maybe save this dicitonary in a json file in case it becomes larger
		self.action_force = {"av5transfer":{"arm":"larm","direction":2}} #dictionary mapping the relevant force to an action
		self.label_up = "longer"
		self.label_down = "shorter"

		self.pub = rospy.Publisher("force_gain", Float64, queue_size=2)
		self.force = {}
		self.force["larm"] = []
		self.force["rarm"] = []
		self.force_des = {}
		self.force_des["larm"] = dict()
		self.force_des["rarm"] = dict()
		self.force_dict = {}
		self.label = []
		self.n_exp = {}
		self.n_t = {}
		self.time = 0 #is used to count up the time during one task to synchronize
		self.action_status = 0
		self.action = None

		#to see if filter work correct
		if debug:
			self.save_up = {}
			self.save_down = {}
			self.save_up["max"] = []
			self.save_up["min"] = []
			self.save_up["mean"] = []
			self.save_up["all"] = []
			self.save_down["max"] = []
			self.save_down["min"] = []
			self.save_down["mean"] = []
			self.save_down["all"] = []
			self.save_actual = []
		self.read_json(path_json)


	def callback_force(self,msg):
		if self.action_status == 0:
			return True
		self.time = self.time + 1
		self.force["larm"].append(msg.larm)
		self.force["rarm"].append(msg.rarm)
		if self.action not in self.action_force.keys():
			return True
		if np.shape(self.force[self.action_force[self.action]["arm"]])[0] == self.window:# and np.all(self.time < np.array(self.n_t[self.action])):
			self.compare()
		#if not np.all(self.time < np.array(self.n_t[self.action])):
		#	print "a sequence is too short"

	def callback_annotation(self,msg):
		[self.action,flag] = msg.data.split("_")
		print "action: %s" % self.action
		if flag == "start":
			self.action_status = 1
			self.read_force()
			self.time = 0
			self.force["larm"] = []
			self.force["rarm"] = []
		if flag == "end":
			self.action_status = 0
			#self.action = None

		if debug and self.action == "av5transfer" and flag == "end":
			self.plot_save_up_down()
		print "action status: %d" % self.action_status

	def read_json(self,path):
		f = open(path,"r")
		force_dict_str = f.read()
		f.close()
		exec("self.force_dict = %s" % force_dict_str)
		self.label = np.array(self.force_dict["label"]) #indexing with i, the index of exp
		self.n_exp = self.force_dict["n_exp"] #indexing with action
		self.n_t = self.force_dict["n_t"] #indexing with action

	def read_force(self):
		force_des_larm = np.array(self.force_dict["force"][self.action]["larm"])[:,:,0:len(self.label)]
		force_des_rarm = np.array(self.force_dict["force"][self.action]["rarm"])[:,:,0:len(self.label)]

		force_des_larm_up = force_des_larm[:,:,self.label == self.label_up]
		force_des_larm_down = force_des_larm[:,:,self.label == self.label_down]

		force_des_rarm_up = force_des_rarm[:,:,self.label == self.label_up]
		force_des_rarm_down = force_des_rarm[:,:,self.label == self.label_down]

		self.force_des["larm"]["up"] =  force_des_larm_up
		self.force_des["larm"]["down"] = force_des_larm_down
		self.force_des["rarm"]["up"] =  force_des_rarm_up
		self.force_des["rarm"]["down"] = force_des_rarm_down

	def compare(self):
		arm = self.action_force[self.action]["arm"]
		direction = self.action_force[self.action]["direction"]
		actual = self.mean_filter(np.array(self.force[arm])[:,direction]) 
		self.save_actual.append(actual)
		self.force["larm"] = []
		self.force["rarm"] = []

		up = self.force_des[arm]["up"][self.time - self.window : self.time , direction, :]
		down = self.force_des[arm]["down"][self.time - self.window : self.time , direction, :]
		#averaging over time first 
		up_mean = []
		down_mean = []
		for i in range(np.shape(up)[1]):
			if np.any(up[:,i]==0):
				up_mean.append(0)
			else:
				up_mean.append(self.mean_filter(up[:,i]))
			if np.any(down[:,i]==0):
				down_mean.append(0)
			else:
				down_mean.append(self.mean_filter(down[:,i]))
		up_mean = np.array(up_mean)
		down_mean= np.array(down_mean)
		self.save_up["all"].append(up_mean)
		self.save_down["all"].append(down_mean)
		#averaging/maximum/minimum over experiments next
		up_dict = {}
		down_dict = {}
		#don't compute gain if the data available is less than 50%
		if float(len(up_mean[up_mean != 0])) / len(up_mean) < 0.5 or float(len(down_mean[down_mean != 0])) / len(down_mean) < 0.5:
			print "too little signals"
			return True
		up_dict["max"] = np.max(up_mean[up_mean != 0])
		up_dict["min"] = np.min(up_mean[up_mean != 0])
		up_dict["mean"] = np.mean(up_mean[up_mean != 0])
		down_dict["max"] = np.max(down_mean[down_mean != 0])
		down_dict["min"] = np.min(down_mean[down_mean != 0])
		down_dict["mean"] = np.mean(down_mean[down_mean != 0])
		if debug:
			self.save_up["max"].append(np.max(up_mean[up_mean != 0]))
			self.save_up["min"].append(np.min(up_mean[up_mean != 0]))
			self.save_up["mean"].append(np.mean(up_mean[up_mean != 0]))
			self.save_down["max"].append(np.max(down_mean[down_mean != 0]))
			self.save_down["min"].append(np.min(down_mean[down_mean != 0]))
			self.save_down["mean"].append(np.mean(down_mean[down_mean != 0]))
		gain = self.compute_gain(up_dict,down_dict,actual)
		print gain #gain should be published
		if not offline:
			self.pub.publish(gain)


	def mean_filter(self,signal):
		if len(signal) != self.window:
			print "CAUTION window size:%d  singal length:%d " % (self.window,len(signal))
		mean = sum(signal)/len(signal)
		return mean

	def compute_gain(self,up,down,actual):
		if abs(actual) <= abs(down["max"]):
			return 0.0 #no gain needed
		if abs(actual) >= abs(up["mean"]):
			return 1.0
		d1 = actual - down["mean"]
		d2 = up["mean"] - down["mean"]
		if d2 == 0:
			return -1.0
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

	def plot_save_up_down(self):
		color = "royalblue"
		fig, axs = plt.subplots(2, 1)
		for i in range(np.shape(self.save_up["all"])[1]):
			signal_up = np.array(self.save_up["all"])[:,i]
			axs[1].plot(signal_up[signal_up!=0], "lightseagreen")
			signal_down = np.array(self.save_down["all"])[:,i]
			axs[1].plot(signal_down[signal_down!=0], "maroon")
		
		line2, = axs[1].plot(self.save_up["mean"], "blue")
		line2, = axs[1].plot(self.save_up["max"], "blue")
		line2, = axs[1].plot(self.save_up["min"], "blue")

		line2, = axs[1].plot(self.save_down["mean"], "red")
		line2, = axs[1].plot(self.save_down["max"], "red")
		line2, = axs[1].plot(self.save_down["min"], "red")

		axs[1].plot(self.save_actual[1::],"orange")
		#axs[1].plot(self.save_down["all"], color)

		plt.show()



if __name__ == "__main__":
    main()


