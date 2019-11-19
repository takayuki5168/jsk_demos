#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField, JointState
from std_msgs.msg import Header
import sys
import math
import tf



def main():
    print "test"
    rospy.init_node('merge_pointcloud_sequence', anonymous=True)
    Merger = MergePointClouds()
    #rospy.Subscriber("/pcl_nodelet/hsi_filter_white/output", PointCloud2, Merger.callback_ptcloud)
    rospy.Subscriber("/pcl_nodelet/ExtractIndices/output", PointCloud2, Merger.callback_ptcloud)
    rospy.Subscriber("/joint_states", JointState, Merger.callback_position)
    rospy.spin()

class MergePointClouds:
    def __init__(self):
        self.pub = rospy.Publisher("point_cloud_sequence_merged", PointCloud2, queue_size=2)
        self.listener = tf.TransformListener()
        self.n_brown = 0
        self.n_white = 0
        self.angle = 0
        self.points_back = []
        self.points_front = []
        self.pos1 = 0
        self.pos2 = 0
        self.cmd_joints = ["torso_lift_joint", "l_shoulder_pan_joint","l_shoulder_lift_joint","l_upper_arm_roll_joint","l_elbow_flex_joint","l_forearm_roll_joint","l_wrist_flex_joint","l_wrist_roll_joint","r_shoulder_pan_joint","r_shoulder_lift_joint","r_upper_arm_roll_joint","r_elbow_flex_joint","r_forearm_roll_joint","r_wrist_flex_joint","r_wrist_roll_joint","head_pan_joint","head_tilt_joint"]

    def callback_position(self,msg):
        #used to specify in which direction the bowl was rotated
        if abs(msg.position[msg.name.index("l_forearm_roll_joint")] * 360 / (2*np.pi) - 4.18) < 0.01:
            self.angle = -np.pi/8
            self.pos1 = msg.position[msg.name.index("l_forearm_roll_joint")]
        elif abs(msg.position[msg.name.index("l_forearm_roll_joint")] * 360 / (2*np.pi) + 13.09) < 0.01:
            self.angle = np.pi/12
            self.pos2 = msg.position[msg.name.index("l_forearm_roll_joint")] 
        else:
            self.angle = 0


    def callback_ptcloud(self,ptcloud):
        if self.angle != 0:
            if abs(self.angle - (-np.pi/8)) < abs(self.angle - (np.pi/12)):
                #print "found points back"
                points_back = self.read_pointcloud(ptcloud)
                self.points_back = self.transform_points(points_back)
            else:
                #print "found points front"
                points_front = self.read_pointcloud(ptcloud)
                self.points_front = self.transform_points(points_front)
        #if both types of pointcloud are available, merge them to one
        if self.points_back and self.points_front:
            print "publish points"
            ptcloud_merged = self.generate_pointcloud(ptcloud)
            self.pub.publish(ptcloud_merged)
            self.points_front = []
            self.points_back = []

    def transform_points(self,points):
        xyz = []
        rgb = []
        #print "########################"
        #print type(points[0])
        #testp = points[0]
        #testp = points[0]._replace(x=1.0,y=2.0,z=5.0)
        #print testp
        #print type(testp)
        for point in points:
            xyz.append([point.x,point.y,point.z])
            rgb.append(point.rgb)
        xyz = np.array(xyz)
        rgb = np.array(rgb)
        rgb = np.reshape(rgb,[len(rgb),1])

        trafo = self.get_transform()
        xyz1 = np.hstack([xyz,np.ones([np.shape(xyz)[0],1])])
        xyz1_transformed = np.transpose(np.matmul(trafo,np.transpose(xyz1)))

        xyz_transformed = xyz1_transformed[:,0:3]
        point_array = np.hstack([xyz_transformed,rgb])
        point_list = list(point_array)
        #return point_list
        #point_list = []
        #for i in range(np.shape(point_array)[0]):
        #    point_list.append(points[0]._replace(x=point_array[i,0],y=point_array[i,1],z=point_array[i,2],rgb=point_array[i,3]))
            #print "type poibnt list element"
            #print type(point_list[0])
            #points[i].x,points[i].y,points[i].z,points[i].rgb = point_array[i,:] 
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
        (trans,rot) = self.listener.lookupTransform('/l_gripper_l_finger_tip_frame', '/head_mount_kinect_rgb_optical_frame', rospy.Time(0))
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

    def generate_pointcloud(self,ptcloud):
        header =  ptcloud.header
        header.frame_id = "/l_gripper_l_finger_tip_frame"
        fields = ptcloud.fields
        #fields[3].datatype = 7
        points = self.points_back + self.points_front
        ptcloud_merged = pc2.create_cloud(header, fields, points)
        #points_test = pc2.read_points_list(ptcloud, skip_nans=True)
        #print ";;;;;;;;;;;;;;;;;"
        #print type(points_test[0])
        ptcloud_merged.fields[3].datatype = 7
        return ptcloud_merged
        


if __name__ == "__main__":
    main()
