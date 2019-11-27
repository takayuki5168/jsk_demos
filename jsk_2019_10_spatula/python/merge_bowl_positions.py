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
import time


def main():
    rospy.init_node('merge_pointcloud_sequence', anonymous=True)
    Merger = MergePointClouds()
    rospy.Subscriber("/pcl_nodelet/ExtractIndices/output", PointCloud2, Merger.callback_ptcloud)
    rospy.Subscriber("/joint_states", JointState, Merger.callback_position)
    rospy.spin()

class MergePointClouds:
    def __init__(self):
        self.pub = rospy.Publisher("point_cloud_sequence_merged", PointCloud2, queue_size=2)
        self.listener = tf.TransformListener()
        self.angle = 0
        self.points = []
        self.look_front = [49.9331,48.1046,24.0788,125.944,-102.011,-13.0963,-30.5362,126.148,-12.2823,-12.0782,-84.1743,-68.6902,-58.5151,-108.045,336.0,3.0727,72.0]
        self.look_right = [49.9331,45.4864,16.3124,126.845,-108.835,-5.52084,-15.5745,129.482,-14.2446,-9.58389,-84.0429,-77.551,-59.1313,-103.819,341.559,3.0727,72.0]
        self.look_back = [49.9331,51.737,30.1327,120.422,-101.764,4.18285,-28.8173,149.013,-15.7055,-13.3653,-80.7665,-83.8848,-54.1232,-103.834,348.24,3.0727,72.0]
        self.look_left = [49.9331,51.1648,45.4815,116.822,-85.7364,-9.41469,-61.5682,142.069,-18.4818,-17.5874,-75.8746,-87.1968,-48.6971,-106.389,352.394,3.0727,72.0]
        self.look_middle = [49.9331,49.5519,26.3404,123.787,-101.985,-6.62817,-29.4461,135.656,-15.2124,-9.77758,-83.654,-81.938,-58.164,-102.254,344.738,3.0727,72.0]
        self.look = ""
        self.change_pos = False
        self.cmd_joints = ["torso_lift_joint", "l_shoulder_pan_joint","l_shoulder_lift_joint","l_upper_arm_roll_joint","l_elbow_flex_joint","l_forearm_roll_joint","l_wrist_flex_joint","l_wrist_roll_joint","r_shoulder_pan_joint","r_shoulder_lift_joint","r_upper_arm_roll_joint","r_elbow_flex_joint","r_forearm_roll_joint","r_wrist_flex_joint","r_wrist_roll_joint","head_pan_joint","head_tilt_joint"]


    def callback_position(self,msg):
        #if there was no pointcloud publichsed to a position it should not change yet
        if self.change_pos:
            return True
        if self.look == "" and abs(msg.position[msg.name.index("l_forearm_roll_joint")] * 360 / (2*np.pi) + 13.096) < 0.1:
            self.look = "front"
            self.change_pos = True
            print "front"
        if self.look is "front" and abs(msg.position[msg.name.index("l_forearm_roll_joint")] * 360 / (2*np.pi) + 5.521) < 0.1:
            self.look = "right"
            self.change_pos = True
            print "right"
        if self.look is "right" and abs(msg.position[msg.name.index("l_forearm_roll_joint")] * 360 / (2*np.pi) - 4.183) < 0.1:
            self.look = "back"
            self.change_pos = True
            print "back"
        if self.look is "back" and abs(msg.position[msg.name.index("l_forearm_roll_joint")] * 360 / (2*np.pi) + 6.886) < 0.1:
            self.look = "left"
            self.change_pos = True
            print "left"
        if self.look is "left" and abs(msg.position[msg.name.index("l_forearm_roll_joint")] * 360 / (2*np.pi) + 9.415) < 0.1:
            self.look = "middle"
            self.change_pos = True
            print "middle"

    def callback_position_new(self,msg):
        #for improvement/generalization
        #https://github.com/AnneKoepken/jsk_demos/commit/cdbbb9c56b540aa9b995bdf4605ff81c1ddfb163#diff-504fa1e0c92264f4bd58021c15ed8343
        #np.sum(np.abs((pos_array-self.av_1[r_arm_controller_indices])/self.av_1[r_arm_controller_indices])) < diff_limit:
        i = 0
        front = True
        right = True
        back = True
        left = True
        middle = True
        for joint in self.cmd_joints:
            if self.look != "" or (abs(msg.position[msg.name.index(joint)] * 360 / (2*np.pi) - self.look_front[i])/abs(self.look_front[i])) > 0.2:
                front = False
            if self.look != "front" or abs(msg.position[msg.name.index(joint)] * 360 / (2*np.pi) - self.look_right[i])/abs(self.look_right[i]) > 0.2:
                right = False
            if self.look != "right" or abs(msg.position[msg.name.index(joint)] * 360 / (2*np.pi) - self.look_back[i])/abs(self.look_back[i]) > 0.2:
                back = False
            if self.look != "back" or abs(msg.position[msg.name.index(joint)] * 360 / (2*np.pi) - self.look_left[i])/abs(self.look_left[i]) > 0.2:
                left = False
            if self.look != "left" or abs(msg.position[msg.name.index(joint)] * 360 / (2*np.pi) - self.look_middle[i])/abs(self.look_middle[i]) > 0.2:
                middle = False
            i = i +1
        if front:
            self.look = "front"
            self.change_pos = True
            print "front"
        elif right:
            self.look = "right"
            self.change_pos = True
            print "right"
        elif back:
            self.look = "back"
            self.change_pos = True
            print "back"
        elif left:
            self.look = "left"
            self.change_pos = True
            print "left"
        elif middle:
            self.look = "middle"
            self.change_pos = True
            print "middle"

    def callback_ptcloud(self,ptcloud):
        if self.change_pos is False:
            return True
        else:
            self.change_pos = False
        points = self.read_pointcloud(ptcloud)
        self.points = self.points + self.transform_points(points)
        if self.look is "middle":
            print "publishing pointcloud"
            ptcloud_merged = self.generate_pointcloud(ptcloud)
            self.pub.publish(ptcloud_merged)
            self.look = ""
            self.points = []


    def transform_points(self,points):
        xyz = []
        rgb = []
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
        t2 = time.time()
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

    def generate_pointcloud(self,ptcloud):
        t1 = time.time()
        header =  ptcloud.header
        header.frame_id = "/l_gripper_tool_frame"
        fields = ptcloud.fields
        ptcloud_merged = pc2.create_cloud(header, fields, self.points)
        ptcloud_merged.fields[3].datatype = 7
        return ptcloud_merged
        


if __name__ == "__main__":
    main()
