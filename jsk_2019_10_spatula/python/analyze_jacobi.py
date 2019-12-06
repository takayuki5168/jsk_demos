#!/usr/bin/env python
from jsk_2019_10_spatula.msg import Jacobian
import rospy
import rosbag

offline = True
path = '/home/leus/2019-12-06-21-07-24.bag'

def main():

    Analyzer = AnalyzeJacobian()
    if offline:
        Analyzer.read_bag()
        #open bagfile and extract data from there
    else:
        rospy.init_node('analyze_jacobian', anonymous=True)
        rospy.Subscriber("/scrape_left_jacobian", Jacobian, Analyzer.callback)
        rospy.spin()



class AnalyzeJacobian():

    def __init__(self):
        self.left_link_list = ["torso_lift_joint", "l_shoulder_pan_joint","l_shoulder_lift_joint","l_upper_arm_roll_joint","l_elbow_flex_joint","l_forearm_roll_joint","l_wrist_flex_joint","l_wrist_roll_joint"]
    
    def callback(self,msg):
        print type(msg.x)
        print "x component"
        print msg.x
        print "y component"
        print msg.y
        print "z component"
        print msg.z
        print "roll component"
        print msg.roll
        print "pitch component"
        print msg.pitch
        print "yaw component"
        print msg.yaw

    def read_bag(self):
        self.bag = rosbag.Bag(path)
        for topic, msg, t in self.bag.read_messages(''):
            print type(msg.x)
            print "x component"
            print msg.x
            print "y component"
            print msg.y
            print "z component"
            print msg.z
            print "roll component"
            print msg.roll
            print "pitch component"
            print msg.pitch
            print "yaw component"
            print msg.yaw


if __name__ == "__main__":
    main()