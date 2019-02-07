#!/usr/bin/python

import sys

import cv2
import cv_bridge
from image_geometry import PinholeCameraModel

import tf
import rospy
import sensor_msgs
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

import numpy as np
import struct
import math
import yaml


class LidarImageOverlay():
    def __init__(self, cameraInfoFile):
        """ Overlay lidar points onto images by setting up the camera model, camera intrinsics, and lidar-camera extrinsics.
        ===============
        cameraInfoFile : an opened yaml file that stores camera intrinsics and other params
            used to initialize the cameraInfo which is used to help cameraModel reproject lidar points to image space. 
        """
        self.cameraParams = yaml.load(cameraInfoFile)
        self.cameraInfo = CameraInfo()
        self._fillCameraInfo()
        self.cameraModel = PinholeCameraModel()
        self.cameraModel.fromCameraInfo( self.cameraInfo )

        print ('Distortion model:', self.cameraInfo.distortion_model)

        self.bridge = cv_bridge.CvBridge()
        # Get transformation/extrinsics between lidar (velodyne frame) and camera (world frame)
        self.tf = tf.TransformListener()

        # Get the topics' names to subscribe or publish 
        self.inImageName = rospy.resolve_name('image')
        self.outImageName = rospy.resolve_name('lidar_image')
        self.lidarName = rospy.remap_name('lidar')

        # Create subscribers and publishers
        self.subImage = rospy.Subscriber(self.inImageName, Image, callback=self.imageCallback, queue_size=1)
        self.lidar = rospy.Subscriber(self.lidarName, PointCloud2, callback=self.lidarCallback, queue_size=1)
        self.pubImage = rospy.Publisher(self.outImageName, Image, queue_size=1)

        self.lidarPoints = None

    def _fillCameraInfo(self,):
        """ Fill the camera params from yaml file to the sensor_msgs/CameraInfo
        """

        self.cameraInfo.width = self.cameraParams['image_width']
        self.cameraInfo.height = self.cameraParams['image_height']
        self.cameraInfo.distortion_model = self.cameraParams['distortion_model']
        self.cameraInfo.D = self.cameraParams['distortion_coefficients']["data"]
        self.cameraInfo.R = self.cameraParams['rectification_matrix']["data"]
        self.cameraInfo.P = self.cameraParams['projection_matrix']["data"]
        self.cameraInfo.K = self.cameraParams['camera_matrix']["data"]

    def imageCallback(self, data):
        """ Once received an image, first get the lidar-camera transformation, then reproject the received lidar points to the camera frame (in pixel), finally draw the projected points on the received image and publish the modified image, i.e., the overlay lidar image.
        ============
        data : a ros image message
            - header (timestamp, frame_id, ...)
            - data (Image)
            ... 
        """
        print( 'Received an image ...' )

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr( 'Failed to convert received ros image msg to CvImage: ', e)
            return 

        t, r = self.tf.lookupTransform('world', 'velodyne', rospy.Time(0))
        t += (1, )
        Rq = tf.transformations.quaternion_matrix(r)
        Rq[:, 3] = t

        if self.lidarPoints:
            for point in self.lidarPoints:
                try:
                    if np.sum(np.array(point[:3])**2) > 16:
                        continue
                    intensity = point[3]
                except IndexError:
                    print (point)
                    break
                if intensity < 1e-3:
                    continue
                
                rotatedPoint = Rq.dot(point)
                # if point is behind the camera, don't need to reproject
                if rotatedPoint[2] < 0:
                    continue

                # if the reprojected point lands in the image space, let's draw it on the current image
                uv = self.cameraModel.project3dToPixel(rotatedPoint)
                if uv[0] >= 0 and uv[0] < data.width and uv[1] >=0 and uv[1] < data.height:
                    # intensityI = (255-Int(intensity)) * 255 *255
                    cv2.circle(cv_image, (int(uv[0]), int(uv[1])), 2, (0, 0, 180), 2 )
        else:
            print ('No lidar data received!')

        try:
            self.pubImage.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8') )
        except cv_bridge.CvBridgeError as e:
            rospy.logerr( 'Failed to convert CvImage to ros image message:', e)
            return 

    def lidarCallback(self, data):
        """ Unpack the raw lidar data to get the [x, y, z, intensity] formated points
        ============
        data : a ros image message
            - header (timestamp, frame_id, ...)
            - data (PointCloud2)
            ... 
        """
        print('Received lidar data ...')

        formatString = 'ffff'
        if data.is_bigendian:
            formatString = '>' + formatString
        else:
            formatString = '<' + formatString

        self.lidarPoints = []
        for i in range(0, len(data.data), 16):
            self.lidarPoints.append(struct.unpack(formatString, data.data[i:i+16]))

        print (len(self.lidarPoints))

if __name__ == '__main__':
    if len( sys.argv ) < 2:
        print('Usage:')
        print( sys.argv[0] + ' camera_info.yaml' )
        sys.exit(1)

    try:
        cameraInfoFile = open(sys.argv[1], 'r')
        rospy.init_node('overlay_lidar_image')
        lio = LidarImageOverlay(cameraInfoFile)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass