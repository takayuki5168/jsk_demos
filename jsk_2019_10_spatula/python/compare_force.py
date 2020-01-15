#!/usr/bin/env python
import rospy
from jsk_2019_10_spatula.msg import Force
import numpy as np
from std_msgs.msg import String
import json


window = 10
force_dict_path = ".json"



def main():
	Compare = CompareForce()
	rospy.init_node('calculate_force', anonymous=True)
	rospy.Subscriber("endeffector_force", Force, Compare.callback_force)
	rospy.Subscriber("semantic_annotation", String, Compare.callback_annotation)
	rospy.spin()
	


class CompareForce:

	def __init__(self):
		self.force = []
		self.force_des = []
		self.force_dict = {}
		self.read_json(force_dict_path)

	def callback_force(self,msg):
		self.force["larm"].append(msg.larm)
		self.force["rarm"].append(msg.rarm)
		if len(self.force) == window and self.force_des:
			self.comparison()

	def callback_annotation(self,msg):
		msg.data
		self.force_des = self.force_dict[action][force][time]

	def read_json(self,path):
		f = open("path","r")
		force_dict_str = f.read()
		f.close()
		exec("self.force_dict = %s" % force_dict_str)


	def comparison(self):
		pass





