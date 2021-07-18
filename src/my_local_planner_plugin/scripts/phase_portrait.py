#!/usr/bin/env python
# coding=utf-8
# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and plots the current velocity.
# publish_feedback must be turned on such that the planner publishes this information.
# Author: christoph.roesmann@tu-dortmund.de

import rospy
import math
from mpl_toolkits.mplot3d import Axes3D
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32, PoseWithCovarianceStamped
import numpy as np
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
import tf2_py
import tf2_ros


def feedback_callback(data):
    global err_
    global xVel_
    global beta_
    global yaw_
    # 质心运动速度
    xVel = data.twist.twist.linear.x
    yVel = data.twist.twist.linear.y
    yaw = data.twist.twist.angular.z
    mag = math.hypot(xVel, yVel)
    quaternion = data.pose.pose.orientation
    explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll,pitch,heading) = euler_from_quaternion(explicit_quat)
    beta = math.atan2(yVel, xVel) - (heading)
    transVel = math.cos(beta) * mag
    if (abs(transVel) < err_) | (abs(beta) > 0.3):
        return
    xVel_.append(transVel)
    beta_.append(beta)
    yaw_.append(yaw)
    # print(str(math.atan2(yVel, xVel)) + "|" + str(heading) + "|" + str(beta))
    # print(str(mag) + "|" + str(transVel))
    return


def plot_vel(ax, xVel_, beta_, yaw_):
    ax.cla()
    ax.set_xlabel("beta")
    ax.set_ylabel("yaw")
    ax.set_zlabel("vx")
    ax.scatter(beta_, yaw_, xVel_)
    return
    # plt.show()


def velocity_plotter():
    rospy.init_node("visualize_velocity_profile", anonymous=True)
    global xVel_
    global beta_
    global yaw_
    global ax
    global fig
    topic_name = "/vesc/odom"
    pose_name = "/amcl_pose"
    rospy.Subscriber(topic_name, Odometry, feedback_callback, queue_size=1)  # define feedback topic here!

    rospy.loginfo(
        "Visualizing velocity profile published on '%s'.", topic_name)
    rospy.loginfo(
        "Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")
    plt.ion()
    plt.show()
    # two subplots sharing the same t axis
    r = rospy.Rate(0.5)  # define rate here
    while not rospy.is_shutdown():
        # rospy.loginfo("none")
        plot_vel(ax, xVel_, beta_, yaw_)
        fig.canvas.draw()
        r.sleep()
    return



if __name__ == '__main__':
    try:
        # plt.hold(True)
        xVel_ = []
        beta_ = []
        yaw_ = []
        initialized_ = False
        fig = plt.figure()
        ax = Axes3D(fig)
        err_ = 0.01
        odomMsg = Odometry()
        odomMsg.pose.pose.orientation
        velocity_plotter()
    except rospy.ROSInterruptException:
        pass
