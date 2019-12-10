#!/usr/bin/env python
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped, VehicleCorners
from geometry_msgs.msg import Point32
from mutex import mutex
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
import cv2
import numpy as np
import os
import rospkg
import rospy
import threading
import time
import yaml


class VehicleDetectionNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        self.active = True

        self.publish_freq = self.setupParam("~publish_freq", 2.0)
        self.circlepattern_dims = tuple(self.setupParam('~circlepattern_dims/data', [7, 3]))
        self.blobdetector_min_area = self.setupParam('~blobdetector_min_area', 10)
        self.blobdetector_min_dist_between_blobs = self.setupParam('~blobdetector_min_dist_between_blobs', 2)
        self.publish_circles = self.setupParam('~publish_circles', True)

        self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.last_stamp = rospy.Time.now()


        self.lock = mutex()
        self.sub_image = rospy.Subscriber("~image", CompressedImage,
                                          self.cbImage, buff_size=921600, 
                                          queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped,
                                           self.cbSwitch, queue_size=1)
        self.pub_detection = rospy.Publisher("~detection",
                                             BoolStamped, queue_size=1)
        self.pub_corners = rospy.Publisher("~corners",
                                           VehicleCorners, queue_size=1)
        self.pub_circlepattern_image = rospy.Publisher("~circlepattern_image",
                                                       Image, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~detection_time",
                                                Float32, queue_size=1)

        rospy.loginfo("[%s] Initialization completed" % (self.node_name))

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def cbImage(self, image_msg):
        if not self.active:
             return
        # Start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()
        # Returns rightaway

    def processImage(self, image_msg): 
        if not self.active:
            return
        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now
        
        
        vehicle_detected_msg_out = BoolStamped()
        vehicle_corners_msg_out = VehicleCorners()
        try:
            image_cv = self.bridge.compressed_imgmsg_to_cv2(
                image_msg, "bgr8")
        except CvBridgeError as e:
            print e

        start = rospy.Time.now()
        params = cv2.SimpleBlobDetector_Params()
        # params.filterByArea = True
        params.minArea = self.blobdetector_min_area
        # params.minThreshold = 100
        # params.maxThreshold = 255
        # params.filterByCircularity = True
        # params.minCircularity = 0.4
        # params.filterByInertia = True
        # params.minInertiaRatio = 0.6
        # params.filterByConvexity = True
        # params.minConvexity = 0.8
        params.minDistBetweenBlobs = self.blobdetector_min_dist_between_blobs
        simple_blob_detector = cv2.SimpleBlobDetector_create(params)
        
        stay_detected = False
        (detection, corners) = cv2.findCirclesGrid(image_cv, self.circlepattern_dims, flags=cv2.CALIB_CB_SYMMETRIC_GRID,
                                                    blobDetector=simple_blob_detector)

        # print(detection)

        # if not stay_detected:
        #     keypoints = simple_blob_detector.detect(image_cv)
        #     im_with_keypoints = cv2.drawKeypoints(image_cv, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #     self.pub_circlepattern_image.publish(self.bridge.cv2_to_imgmsg(im_with_keypoints, "bgr8"))

        vehicle_detected_msg_out.data = detection
        self.pub_detection.publish(vehicle_detected_msg_out)
        if detection:
            # print(corners)
            points_list = []
            for point in corners:
                corner = Point32()
                # print(point[0])
                corner.x = point[0, 0]
                # print(point[0,1])
                corner.y = point[0, 1]
                corner.z = 0
                points_list.append(corner)
            vehicle_corners_msg_out.header.stamp = rospy.Time.now()
            vehicle_corners_msg_out.corners = points_list
            vehicle_corners_msg_out.detection.data = detection
            vehicle_corners_msg_out.H = self.circlepattern_dims[1]
            vehicle_corners_msg_out.W = self.circlepattern_dims[0]
            self.pub_corners.publish(vehicle_corners_msg_out)
        
        if detection and self.publish_circles:
            stay_detected = True
            cv2.drawChessboardCorners(image_cv,
                                        self.circlepattern_dims, corners, detection)
            image_msg_out = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
            self.pub_circlepattern_image.publish(image_msg_out)
        
        elapsed_time = (rospy.Time.now() - start).to_sec()
        self.pub_time_elapsed.publish(elapsed_time)
    

if __name__ == '__main__':
    rospy.init_node('vehicle_detection', anonymous=False)
    vehicle_detection_node = VehicleDetectionNode()
    rospy.spin()
