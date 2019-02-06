#!/usr/bin/python

import sys 

import cv2
from image_geometry import PinholeCameraModel

import tf
from sensor_msgs.msg import CameraInfo

import numpy as np
from scipy.optimize import minimize
import math
import random
import json
import yaml


class LidarCameraCalibration():
    def __init__(self, settingsFile, cameraInfoFile, ):
        """ Init the cameraModel as PinholeCameraModel with camera intrinsics and params
        =============
        settingsFile : json file storing 3D-2D point pairs and params for optimization
        cameraInfoFile : yaml file storing camera intrinsics and params
        """

        self.settings = json.load(settingsFile)
        self.cameraParams = yaml.load(cameraInfoFile)
        self.cameraModel = PinholeCameraModel()
        self.cameraInfo = CameraInfo()

        self.__fillCameraInfo()
        self.cameraModel.fromCameraInfo( self.cameraInfo )

        print ('Camera Model is initialized, ready to calibrate ...\n')

    def calibrateTranformation(self, ):
        """ Calculate the transformation/extrinsics between lidar and camera by minimizing the reprojection errors, providing camera model and params and 2d-3d correspondence points.
        """

        print ('Calibrating the transformation ... ')
        result = minimize(self.costFuction, self.settings['initialTransform'], args=(1.0), bounds=self.settings['bounds'], method='SLSQP', options={'disp': True, 'maxiter': 500})

        while not result.success or result.fun > 25:
            for i in range( 0, len(self.settings['initialTransform']) ):
                self.settings['initialTransform'][i] = random.uniform( self.settings['bounds'][i][0], self.settings['bounds'][i][1] )

            print ('\nRestart with new initialTransform ...\n')
            result = minimize(self.costFuction, self.settings['initialTransform'], args=(1.0), bounds=self.settings['bounds'], method='SLSQP', options={'disp': True, 'maxiter': 500})

        print ('Calibration done ... ')
    	print ('Final transformation:')
        print (result.x)
        print ('Error: ' + str( result.fun))

    def costFuction(self, x, sign=1):
        """ Define the cost function with reprojection errors. Reprojection error is defined as the difference between ground-truth image points and reprojected image points from the corresponding lidar 3d points.
        =============
        x : the initial guess of the transformation (x, y, z, yaw, pitch, roll)
        sign : ditermines minimization (+1) or maximization (-1) since the 'from scipy.optimize import minimize' is used. 
        """

        params = x
        translation = [params[0], params[1], params[2], 1.0]
        rotationMatrix = tf.transformations.euler_matrix( params[5], params[4], params[3] )

        rotationMatrix[:, 3] = translation

        error = 0
        for point, uv in zip (self.settings['points'], self.settings['uvs']):
            rotatedPoint = rotationMatrix.dot(point)

            uv_new = self.cameraModel.project3dToPixel( rotatedPoint )

            diff = np.array(uv_new) - np.array(uv)
            error += math.sqrt(np.sum(diff**2))

        return error * sign

    def __fillCameraInfo(self,):
        """ Fill the camera params from yaml file to the sensor_msgs/CameraInfo
        """

        self.cameraInfo.width = self.cameraParams['image_width']
        self.cameraInfo.height = self.cameraParams['image_height']
        self.cameraInfo.distortion_model = self.cameraParams['distortion_model']
        self.cameraInfo.D = self.cameraParams['distortion_coefficients']["data"]
        self.cameraInfo.R = self.cameraParams['rectification_matrix']["data"]
        self.cameraInfo.P = self.cameraParams['projection_matrix']["data"]
        self.cameraInfo.K = self.cameraParams['camera_matrix']["data"]
    


if __name__ == "__main__":
    if len( sys.argv ) < 3:
        print('Usage:')
        print( sys.argv[0] + ' settings_file.json camera_info.yaml' )
        sys.exit(1)

    settingsFile = open(sys.argv[1], 'r')
    cameraInfoFile = open(sys.argv[2], 'r')

    # init lcc with the setting files
    lcc = LidarCameraCalibration(settingsFile, cameraInfoFile)
    lcc.calibrateTranformation()
    

