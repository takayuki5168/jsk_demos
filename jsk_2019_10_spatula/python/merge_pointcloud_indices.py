#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def main():
    #pub = rospy.Publisher('feedback_touching', Float64, queue_size=10)
    Merger = MergePointClouds()
    rospy.init_node('merge_pointcloud', anonymous=True)
    rospy.Subscriber("/pcl_nodelet/hsi_filter_brown/output", PointCloud2, Merger.callback_brown)
    rospy.Subscriber("/pcl_nodelet/hsi_filter/output", PointCloud2, Merger.callback_white)
    rospy.spin()

class MergePointClouds:
    def __init__(self):
        self.pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)
        self.n_brown = 0
        self.n_white = 0

    def callback_white(self,ptcloud):
        self.ptcloud_white = ptcloud
        self.n_white = self.n_white + 1
        print "new white pointcloud available n_brown:%d  n_white:%d" % (self.n_brown,self.n_white)
        self.points_white = self.read_pointcloud(ptcloud)
        if self.n_brown == self.n_white:
            print "publish in white"
            ptcloud_merged = self.generate_pointcloud()
            self.pub.publish(ptcloud_merged)

    def callback_brown(self,ptcloud):
        self.ptcloud_brown = ptcloud
        self.n_brown = self.n_brown + 1
        self.points_brown = self.read_pointcloud(ptcloud)
        print "new brown pointcloud available n_brown:%d  n_white:%d" % (self.n_brown,self.n_white)
        if self.n_brown == self.n_white:
            print "publish in brown"
            ptcloud_merged = self.generate_pointcloud()
            self.pub.publish(ptcloud_merged)

    def read_pointcloud(self,ptcloud):
        gen = pc2.read_points(ptcloud, skip_nans=True)
        points = list(gen)
        if False:
            for x in points:
                test = x[3] 
                # cast float32 to int so that bitwise operations are possible
                s = struct.pack('>f' ,test)
                i = struct.unpack('>l',s)[0]
                # you can get back the float value by the inverse operations
                pack = ctypes.c_uint32(i).value
                r = (pack & 0x00FF0000)>> 16
                g = (pack & 0x0000FF00)>> 8
                b = (pack & 0x000000FF)
                print r,g,b # prints r,g,b values in the 0-255 range
                # x,y,z can be retrieved from the x[0],x[1],x[2]
        return points

    def generate_pointcloud(self):
        header =  self.ptcloud_brown.header
        fields = self.ptcloud_brown.fields
        if False:
            for p in points:
                print np.shape(p)
                x,y,z,r,g,b = p
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r))[0]
                print hex(rgb)
                pt = [x, y, z, rgb]
                points_data.append(pt)

        #points = self.points_brown #+ self.points_white
        gen = pc2.read_points(self.ptcloud_brown, skip_nans=True)
        points = list(gen)
        
        ptcloud_merged = pc2.create_cloud(header, fields, points)

        #ptcloud_merged.height = self.ptcloud_brown.height
        #ptcloud_merged.height = 1
        #ptcloud_merged.width = self.ptcloud_brown.width
        #ptcloud_merged.width = len(points)
        #ptcloud_merged.point_step = 20
        #ptcloud_merged.row_step = 20*len(points)
        #ptcloud_merged.is_bigendian = self.ptcloud_brown.is_bigendian
        #ptcloud_merged.point_step = self.ptcloud_brown.point_step
        #ptcloud_merged.row_step = self.ptcloud_brown.row_step

        return ptcloud_merged


if __name__ == "__main__":
    main()