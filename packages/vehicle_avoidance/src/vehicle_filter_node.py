#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import VehicleCorners, VehiclePose, Pose2DStamped
from geometry_msgs.msg import Point32
from image_geometry import PinholeCameraModel
from mutex import mutex
from sensor_msgs.msg import CameraInfo
from math import sqrt, sin, cos
from std_msgs.msg import Float32
import cv2
import numpy as np
import os
import rospkg
import rospy
import threading
import yaml


class VehicleFilterNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()

        self.distance_between_centers = self.setupParam('distance_between_centers', 0.0125)
        self.max_reproj_pixelerror_pose_estimation = self.setupParam('max_reproj_pixelerror_pose_estimation', 5)

        self.sub_corners = rospy.Subscriber("~corners", VehicleCorners,
                                            self.processCorners, queue_size=1)

        self.pub_pose = rospy.Publisher("~pose", VehiclePose, queue_size=1)
        self.sub_info = rospy.Subscriber("~camera_info", CameraInfo,
                                         self.processCameraInfo, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~pose_calculation_time",
                                                Float32, queue_size=1)
        self.pcm = PinholeCameraModel()
        rospy.loginfo("[%s] Initialization completed" % (self.node_name))
        self.lock = mutex()

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def processCameraInfo(self, camera_info_msg):
        if self.lock.testandset():
            self.pcm.fromCameraInfo(camera_info_msg)
            self.lock.unlock()

    def processCorners(self, vehicle_corners_msg):
        # do nothing - just relay the detection
        rospy.loginfo("HMMMM")
        if self.lock.testandset():
            start = rospy.Time.now()
            # print(start)
            self.calcCirclePattern(vehicle_corners_msg.H,
                                   vehicle_corners_msg.W)
            points = []
            for Point32 in vehicle_corners_msg.corners:
                point = [Point32.x, Point32.y]
                points.append(point)
            points = np.array(points)
            # points = np.reshape(points, (2,-1))
            # print(points)
            # print(self.pcm.intrinsicMatrix())
            size = [480, 640]
            focal_length = size[1]
            center = (size[1]/2, size[0]/2)
            camera_matrix = np.array(
                                    [[focal_length, 0, center[0]],
                                    [0, focal_length, center[1]],
                                    [0, 0, 1]], dtype = "double"
                                    )
            print(self.pcm.distortionCoeffs())
            # print(self.circlepattern)
            (success, rotation_vector, translation_vector) = cv2.solvePnP(
                self.circlepattern, points, camera_matrix, self.pcm.distortionCoeffs())
            rospy.loginfo("WATWAT")
            print("Rot: {}".format(rotation_vector))
            print("Trans: {}".format(translation_vector))
            if success:
                points_reproj, _ = cv2.projectPoints(
                    self.circlepattern, rotation_vector, translation_vector, camera_matrix, self.pcm.distortionCoeffs())
                error = 0
                for i in range(0, len(points_reproj)):
                    error += cv2.norm(points[i],
                                      points_reproj[i, 0], cv2.NORM_L2)

                mean_reproj_error = error / len(points_reproj)
                print(mean_reproj_error)
                print(self.max_reproj_pixelerror_pose_estimation)

                if mean_reproj_error < self.max_reproj_pixelerror_pose_estimation:
                    # print(translation_vector)
                    (R, jac) = cv2.Rodrigues(rotation_vector)
                    R_inv = np.transpose(R)
                    translation_vector = -np.dot(R_inv, translation_vector)
                    pose_msg_out = VehiclePose()
                    pose_msg_out.header.stamp = rospy.Time.now()
                    pose_msg_out.rho.data = sqrt(
                        translation_vector[2] ** 2 + translation_vector[0] ** 2)
                    pose_msg_out.psi.data = np.arctan2(
                        -R_inv[2, 0], sqrt(R_inv[2, 1] ** 2 + R_inv[2, 2] ** 2))
                    pose_msg_out.detection.data = vehicle_corners_msg.detection.data
                    R2 = np.array([[cos(pose_msg_out.psi.data), -sin(pose_msg_out.psi.data)], [
                                    sin(pose_msg_out.psi.data), cos(pose_msg_out.psi.data)]])
                    translation_vector = - \
                        np.array([translation_vector[2],
                                    translation_vector[0]])
                    translation_vector = np.dot(
                        np.transpose(R2), translation_vector)
                    pose_msg_out.theta.data = np.arctan2(
                        translation_vector[1], translation_vector[0])
                    self.pub_pose.publish(pose_msg_out)
                    rospy.loginfo("HEYO")
                else:
                    rospy.loginfo(
                        "[%s] Pose estimation failed, too high reprojection error." % (self.node_name))
            else:
                rospy.loginfo("[%s] Pose estimation failed." %
                              (self.node_name))

            elapsed_time = (rospy.Time.now() - start).to_sec()
            self.pub_time_elapsed.publish(elapsed_time)
        self.lock.unlock()
        return

    def calcCirclePattern(self, height, width):
        self.circlepattern_dist = self.distance_between_centers
        self.circlepattern = np.zeros([height * width, 3])
        for i in range(0, width):
            for j in range(0, height):
                self.circlepattern[width - 1 - i + j * width, 0] = self.circlepattern_dist * \
                    i - self.circlepattern_dist * (width - 1) / 2
                self.circlepattern[width - 1 - i + j * width, 1] = self.circlepattern_dist * \
                    j - self.circlepattern_dist * (height - 1) / 2
        print(self.circlepattern)


if __name__ == '__main__':
    rospy.init_node('vehicle_filter_node', anonymous=False)
    vehicle_filter_node = VehicleFilterNode()
    rospy.spin()
