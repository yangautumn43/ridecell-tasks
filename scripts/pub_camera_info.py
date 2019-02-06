#!/usr/bin/python

import rospy
import sys
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

class SubPub():
    def __init__(self, yaml_name, pub_image_topic, pub_camera_topic, sub_topic):
        '''
        Later 
        '''
        rospy.init_node('SubPub', anonymous=True)
        self.pub_image_ = rospy.Publisher(pub_image_topic, Image, queue_size=10)
        self.pub_camera_ = rospy.Publisher(pub_camera_topic, CameraInfo, queue_size=10)
        self.sub_ = rospy.Subscriber(sub_topic, Image, self.callBack)
        self.calib_camera_info = self.yaml2CameraInfo(yaml_name)

    def callBack(self, data):
        image_to_pub = data
        now = rospy.Time.now()
        image_to_pub.header.stamp = now

        self.calib_camera_info.header.stamp = now

        self.pub_image_.publish(image_to_pub)
        self.pub_camera_.publish(self.calib_camera_info)
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        # print ("pub ... ")        

    def yaml2CameraInfo(self, yaml_name):
        """
        Parse a yaml file containing camera calibration data (as produced by 
        rosrun camera_calibration cameracalibrator.py) into a 
        sensor_msgs/CameraInfo msg.
        
        Parameters
        ----------
        yaml_name : str
            Path to yaml file containing camera calibration data
        Returns
        -------
        camera_info_msg : sensor_msgs.msg.CameraInfo
            A sensor_msgs.msg.CameraInfo message containing the camera calibration
            data
        """
        # Load data from file
        with open(yaml_name, "r") as file_handle:
            calib_data = yaml.load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        camera_info_msg.roi.do_rectify = True
        return camera_info_msg


if __name__ == "__main__":
    # Get fname from command line (cmd line input required)
    filename = sys.argv[1]

    pub_image_topic = '/ridecell/image_raw'
    pub_camera_topic = '/ridecell/camera_info'
    sub_topic = '/sensors/camera/image_color'

    sp = SubPub(filename, pub_image_topic, pub_camera_topic, sub_topic)

    rospy.spin()
