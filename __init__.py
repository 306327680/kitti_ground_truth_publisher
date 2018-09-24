from __future__ import print_function
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped as Pose
import pykitti
import numpy
from nav_msgs.msg import Odometry as odom


# from os.path import join
import os.path
from click import progressbar
from pyquaternion import Quaternion


basedir = '/media/echo/Data/dataset'


# Specify the dataset to load
sequence = '00'
dataset = pykitti.odometry(basedir, sequence, frames=range(0, 4541, 1))

def kitti_publisher(vo_fn="", im_dir="", rate=1.0):
    i = 0
    rospy.init_node('kitti_publisher', anonymous=True)
    rate = rospy.Rate(10.0)  # 20 * rate hz
    if vo_fn != "":
        point_pub = rospy.Publisher('/kitti/position', Pose, queue_size=100)
        odom_pub = rospy.Publisher('/kitti/odom', odom, queue_size=100)
    with open(vo_fn, 'r') as vo_f:
        with progressbar(vo_f.readlines(), label="Publishing KITTI Data") as bar:
            for line in bar:
                while not rospy.is_shutdown():
                    #while not rospy.is_shutdown():
                    if len(line) != 0 and line[0] != "#":  # skip comments and empty line at the end
                        line_split = line.split()
                        frame_id = "map"
                        i += 1
                        second_pose = dataset.poses[i]
                        print('\nSecond ground truth pose:\n' + str(second_pose))
                        print(second_pose[1][1])
                        point = Pose()
                        odom = odom()
                        rotation = numpy.eye(3)
                        rotation[0][0] = second_pose[0][0]
                        rotation[0][1] = second_pose[0][1]
                        rotation[0][2] = second_pose[0][2]
                        rotation[1][0] = second_pose[1][0]
                        rotation[1][1] = second_pose[1][1]
                        rotation[1][2] = second_pose[1][2]
                        rotation[2][0] = second_pose[2][0]
                        rotation[2][1] = second_pose[2][1]
                        rotation[2][2] = second_pose[2][2]

                        # quat = tf.transformations.(rotation)
                        # q8d = Quaternion(matrix=rotation)
                        #print(quat)
                        point.header.frame_id = frame_id
                        odom.h
                        point.pose.pose.position.x = second_pose[0][3]
                        point.pose.pose.position.y = second_pose[1][3]
                        point.pose.pose.position.z = second_pose[2][3]
                        point_pub.publish(point)
                        rate.sleep()
                        # if im_dir != "":
                        #     fl_nm = str(i).zfill(6) + ".png"
                        #     imL = imread(join(im_dir, "image_0/" + fl_nm))
                        #     imR = imread(join(im_dir, "image_1/" + fl_nm))
                        #     if imL is None or imR is None:
                        #         break
                        #     msgL = cvb.cv2_to_imgmsg(imL, "bgr8")
                        #     msgL.header.frame_id = frame_id
                        #     msgR = cvb.cv2_to_imgmsg(imR, "bgr8")
                        #     msgR.header.frame_id = frame_id
                        #     imL_pub.publish(msgL)
                        #     imR_pub.publish(msgR)



if __name__ == '__main__':
    vo_fn = "/media/echo/Data/dataset/sequences/00/00.txt"
    im_dir = ""

    try:
        kitti_publisher(vo_fn)
    except rospy.ROSInterruptException:
        pass
