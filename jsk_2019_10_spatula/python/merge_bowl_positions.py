#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField, JointState
from std_msgs.msg import Header, String
import sys
import math
import tf
import time


debug = True

def main():
    rospy.init_node('merge_pointcloud_sequence', anonymous=True)
    Merger = MergePointClouds()
    rospy.Subscriber("/pcl_nodelet/ExtractIndices/output", PointCloud2, Merger.callback_ptcloud)
    rospy.Subscriber("/semantic_annotation", String, Merger.callback_semantics)
    rospy.spin()


class MergePointClouds:
    def __init__(self):
        self.pub = rospy.Publisher("point_cloud_sequence_merged", PointCloud2, queue_size=2)
        if debug:
            self.pub_debug = rospy.Publisher("point_cloud_one_look", PointCloud2, queue_size=2)
        self.listener = tf.TransformListener()
        self.add_ptcloud = False #if set to True, the next published ptcloud is added
        self.points = []


    def callback_semantics(self,msg):
        split_action = msg.data.split("_")
        if split_action[0] == "vision":
            if split_action[1] == "end":
                print "publishing pointcloud"
                ptcloud_merged = self.generate_pointcloud()
                self.pub.publish(ptcloud_merged)
                self.points = []
            else:
                self.add_ptcloud = True


    def callback_ptcloud(self,ptcloud):
        #always have the newest version of the ptcoud
        self.ptcloud = ptcloud
        if self.add_ptcloud:
            #look up trafo right away as otherwise there is a delay due to read_ptcloud
            self.add_ptcloud = False
            self.trafo = self.get_transform()
            points = self.read_pointcloud(ptcloud)
            points_transformed = self.transform_points(points)
            self.points = self.points + points_transformed
            #publish the ptcloud of one look for debugging
            if debug:
                ptcloud_merged = self.generate_pointcloud(points_transformed)
                self.pub_debug.publish(ptcloud_merged)
            


    def transform_points(self,points):
        xyz = []
        rgb = []
        for point in points:
            xyz.append([point.x,point.y,point.z])
            rgb.append(point.rgb)
        xyz = np.array(xyz)
        rgb = np.array(rgb)
        rgb = np.reshape(rgb,[len(rgb),1])

        #trafo = self.get_transform()
        trafo = self.trafo
        xyz1 = np.hstack([xyz,np.ones([np.shape(xyz)[0],1])])
        xyz1_transformed = np.transpose(np.matmul(trafo,np.transpose(xyz1)))

        xyz_transformed = xyz1_transformed[:,0:3]
        point_array = np.hstack([xyz_transformed,rgb])
        point_list = list(point_array)
        return point_list

    def quat2mat(self,quat):
        #http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html
        #implementation from https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]
        mat = np.array([[1-(2*(y**2))-(2*(z**2)), 2*x*y-(2*z*w), 2*x*z+(2*y*w)],
                        [2*x*y + 2*z*w, 1-(2*(x**2))-(2*(z**2)), 2*y*z-(2*x*w)],
                        [2*x*z-(2*y*w), 2*y*z + 2*x*w, 1-(2*(x**2))-(2*(y**2))]])
        return mat

    def get_transform(self):
        (trans,rot) = self.listener.lookupTransform('/l_gripper_tool_frame', '/head_mount_kinect_rgb_optical_frame', rospy.Time(0))
        rotmat = self.quat2mat(rot)
        trans_v = np.array(trans)
        trans_v = np.reshape(trans,[3,1])
        trafo = np.vstack([np.hstack([rotmat,trans_v]),np.array([0,0,0,1])])
        return trafo

    def read_pointcloud(self,ptcloud):
        fields = ptcloud.fields
        fields[3].datatype = 6
        points = pc2.read_points_list(ptcloud, skip_nans=True)
        return points

    def generate_pointcloud(self,points=None):
        header =  self.ptcloud.header
        header.frame_id = "/l_gripper_tool_frame"
        fields = self.ptcloud.fields
        #apparently 'ptcloud_merged.fields[3].datatype = 7' later changes also self.ptcloud.fields[3].datatype = 7
        #call by reference! in case of debugging it messes up the colors, thats why this line is needed
        fields[3].datatype = 6
        if not points:
            points = self.points
        ptcloud_merged = pc2.create_cloud(header, fields, points)
        ptcloud_merged.fields[3].datatype = 7
        return ptcloud_merged
        


if __name__ == "__main__":
    main()
