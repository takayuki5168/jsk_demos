#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import sensor_msgs.point_cloud2 as pc2
import numpy as np




def main():
    rospy.init_node('detect_dirt', anonymous=True)
    Detector = DirtDetector()
    rospy.Subscriber("/pcl_nodelet/hsi_filter_white/output", PointCloud2, Detector.callback_ptcloud_white)
    rospy.Subscriber("/pcl_nodelet/hsi_filter_brown/output", PointCloud2, Detector.callback_ptcloud_brown)
    rospy.spin()

class DirtDetector():
    def __init__(self):
        #0:clean, 1:dirty
        self.pub = rospy.Publisher("dirt_label", Bool, queue_size=2)
        self.pub_white = False
        self.pub_brown = False
        #determine parameters for cutting off the ptcloud
        self.m = [0.14,0,0]
        self.r_middle = 0.05
        self.r_bowl = 0.1
        self.z_max = 0.1
        self.z_min = -0.1

        self.n_pieces = 12

    def callback_ptcloud_brown(self, ptcloud):
        points = self.read_pointcloud(ptcloud)
        self.xyz_brown = self.read_points(points)
        self.pub_brown = True
        if self.pub_white:
            self.get_labels()


    def callback_ptcloud_white(self, ptcloud):
        points = self.read_pointcloud(ptcloud)
        self.xyz_white = self.read_points(points)
        self.pub_white = True
        if self.pub_brown:
            self.get_labels()

    def read_pointcloud(self,ptcloud):
        fields = ptcloud.fields
        fields[3].datatype = 6
        points = pc2.read_points_list(ptcloud, skip_nans=True)
        return points

    def read_points(self,points):
        xyz = []
        for point in points:
            xyz.append([point.x,point.y,point.z])
        xyz = np.array(xyz)
        return xyz

    def get_labels(self):        
        percentage = self.calculate_percentage()
        self.pub_brown = False
        self.pub_white = False
        labels = percentage > 0.1
        print percentage
        print labels
        #self.pub.publish(labels)

    def calculate_percentage(self):
        percentage = np.zeros(self.n_pieces)
        for i in range(self.n_pieces):
            phi_min = (i)*2*np.pi/self.n_pieces - np.pi
            phi_max = (i+1)*2*np.pi/self.n_pieces - np.pi
            #print "slice size"
            #print phi_max - phi_min
            points_cut_brown = self.cut_points(self.xyz_brown,phi_min,phi_max)
            points_cut_white = self.cut_points(self.xyz_white,phi_min,phi_max)
            #print "shape brown and white cut points"
            shape_brown = np.shape(points_cut_brown)
            shape_white = np.shape(points_cut_white)
            n_brown = shape_brown[0]
            print shape_brown
            n_white = shape_white[0]
            print shape_white
            percentage[i] = float(n_brown) / float(n_brown + n_white)
            print percentage[i]
        return percentage

    def cut_points(self,xyz,phi_min,phi_max):    
        r_phi = np.transpose(np.array([np.sqrt(np.power((xyz[:,0]-self.m[0]),2) + np.power((xyz[:,1]-self.m[1]),2)),np.arctan2(xyz[:,1]-self.m[1],xyz[:,0]-self.m[0])]));

        #print np.shape(r_phi)
        #print type(r_phi)
        #print np.shape(r_phi[:,0] > self.r_middle)
        #print np.shape(xyz[:,2] < self.z_max)
        #print np.shape(r_phi[:,1] > phi_min)
        #print "logicals"
        #print np.logical_and(r_phi[:,0] > self.r_middle, r_phi[:,0] < self.r_bowl)
        #print np.sum(np.logical_and(r_phi[:,0] > self.r_middle, r_phi[:,0] < self.r_bowl))
        #print np.logical_and(r_phi[:,1] > phi_min,r_phi[:,1] < phi_max)
        #print np.sum(np.logical_and(r_phi[:,1] > phi_min,r_phi[:,1] < phi_max))
        #print np.logical_and(xyz[:,2] > self.z_min,xyz[:,2] < self.z_max)
        #print np.sum(np.logical_and(xyz[:,2] > self.z_min,xyz[:,2] < self.z_max))
        #print np.sum(np.logical_and( np.logical_and(r_phi[:,0] > self.r_middle, r_phi[:,0] < self.r_bowl), np.logical_and(r_phi[:,1] > phi_min,r_phi[:,1] < phi_max)))
        
        #(r_middle < r < r_bowl) & (phi_min < phi < phi_max) & (z_min < z < z_max)
        xyz_cut = xyz[np.logical_and(np.logical_and( np.logical_and(r_phi[:,0] > self.r_middle, r_phi[:,0] < self.r_bowl), np.logical_and(r_phi[:,1] > phi_min,r_phi[:,1] < phi_max)), np.logical_and(xyz[:,2] > self.z_min,xyz[:,2] < self.z_max)),:]
        return xyz_cut



if __name__ == "__main__":
    main()
