# ridecell-tasks
Platform: Ubuntu 16.4

ROS version: Kinetic


## Task One: camera calibration
### 1. Get the camera intrinsic calibration parameters
The ros package for camera intrinsic calibration is `camera_calibration`.

The command is `rosrun camera_calibration cameracalibrator.py --size 5x7 --square 0.05 image:=/sensors/camera/image_color camera:=/sensors/camera/camera_info --no-service-check/`, then I run `rosbag play [path-to-bag-file]`

Here is what I got after running `rostopic list`:
```
/clock
/rosout
/rosout_agg
/sensors/camera/camera_info
/sensors/camera/image_color
/sensors/velodyne_points
```
After enough images are collected (for different x, y, skew, and size), I clicked the `Calibrate` button to start calibration. The caliration results are saved to `/tmp/calibrationdata.tar.gz/` after I clicked the `Save` button.

The yaml file (now in the `/results/calibrationdata/` folder) stores all the calibrated parameters we will need to rectify images taken from that camera.

### 2. Use the above parameters to update the camera_info topic of the original rosbag file 
The best way to prepare the input for rectifying the images is to modify the original rosbag file by replacing the 'camera_info' topic. 'bag_tools' would be the best option, however it can't be installed on my Ubuntu 16.4 LTE ROS kinetic.

Instead I wrote a script (`pub_camera_info.py`) to subscribe `image` topic and publish new `image` and `camera_info` topics, within `ridecell` namespace, with the same timestamp and calibrated camera parameters. This is needed by the next step.


### 3. Launch to play the rosbag and image_proc node to get the rectified images

I wrote the launch file (`camera_calibration.launch`) to run the above three altogether.

```xml
<launch>
	<!-- Play the rosbag file to publish original sensor data -->
	<node name="rosbag" pkg="rosbag" type="play" 
	args="$(find ridecell)/bags/2016-11-22-14-32-13_test.bag -l"
	output="screen"
	/>

	<node name="subpub" pkg="ridecell" type="pub_camera_info.py"
	args="$(find ridecell)/config/ost.yaml" 
	/>

	<!-- Run to subscribe topics image_raw and camera_info in namespace, eg., defined in ns="/ridecell" -->
	<node name="image_proc" pkg="image_proc" type="image_proc" ns="/ridecell" output="screen"
	/>
</launch>
```

Then fire up `rqt`, we can view the rectified images by choosing the `/ridecell/image_rect_color`, as shown below:

![Task One: Image Rectification](results/rqt_rectified_image.png)


## Task Two: lidar-camera extrinsic calibration

The extrinsic parameters of a lidar-camera calibration includes the rotation and translation, which is 6DoF. 2D-3D point correspondence method is used to calculate the transformation (rotation+translation) between Lidar and camera. 

### 1. The first step is to collect the correspondence points (lidar-3D, camera-2D)
-  The first problem is to find out how to collect those points. (Set a specific time so that the checkerboard is clear to select corresponding lidar points?) 

- Together with the above problem, now maybe I need to get a rectified rosbag file using a Indigo Ros version (for bag_tools).

    [NO. With the launch file in Task One, we can use `space` to control pause and select the corresponding lidar and camera frames]

- How to sync a camera frame and a lidar frame?

    [See the answer above.]

- How to select the lidar points within a frame and corresponding camera points in a image?

    [Use rviz. Top menu --> Panels --> Selection. Then add a tool called 'Select' besides 'Move Camera' by clicking the `plus` button. After that, choose `Select` and click a lidar point in the rviz gui, then in the `Selection` panel you will see the 3D point (x, y, z).]

    Reference: how to point and click on rviz map and output the position (https://answers.ros.org/question/69019/how-to-point-and-click-on-rviz-map-and-output-the-position/)

- However, to select the corresponding points in the image, we need to save the rectified image of original size. 

    [We can save the image of original size using `image_view` by setting `autosize=true`.]

    I wrote a script(`collect_image_points.py`) to manually select the points of interest in the image given the path as the command line argument.

    Here is the image points I selected.
    ![Image points](results/image_points.png)


To view the lidar data together with rectified images, I run the `static_transform_publisher` in `tf` to set `world` frame and `velodyne` frame transformation to be zeros (no rotation, no translation).
```
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 world velodyne 1000
```
This command is now added to `camera_calibration.launch` and `view_lidar.launch`.

```xml
<launch>
	<!-- Play the rosbag file to publish original sensor data -->
	<node name="rosbag" pkg="rosbag" type="play" 
	args="--clock $(find ridecell)/bags/2016-11-22-14-32-13_test.bag -l"
	output="screen"
	/>

    <!-- rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 world velodyne 1000 -->
    <node name="static_transform" pkg="tf" type="static_transform_publisher"
    args="0.0 0.0 0.0 0.0 0.0 0.0 1.0 world velodyne 1000"
    />
</launch>
```

Here is a screenshot of the rviz window visualizing lidar data and rectified image.

![Task Two: Lidar and camera visualization](results/rviz_lidar_camera.png)

Then the collected six lidar-image point pairs are stored in a json file for calculating the transformation using optimization method.


### 2. Then run the optimization algorithm to calibrate the lidar-camera transformation (extrinsic)

The basic logic behind this is to minimize the reprojection errors. The correspondence 3D-2D points and camera params are passed to the calibration script. A pinhole camera model is defined and initialized with the camera intrinsic params, which is used reproject the 3D lidar points to image space. Actually the number of correspondence points can be variable. For a better estimation, spatial uniformly distributed points are recommended. However, precision of the selected points is also important.

Here I start the optimization by firing up the command:
```
rosrun ridecell calibrate_lidar_camera.py config/lidar_image_points.json config/ost.yaml
```

Here is the terminal output:
```
Optimization terminated successfully.    (Exit mode 0)
            Current function value: 22.8676357989
            Iterations: 59
            Function evaluations: 561
            Gradient evaluations: 59
Calibration done ...
Final transformation:
[-0.53831089 -0.076088   -0.41682017  2.67673445  4.53119823  5.19606972]
```

### 3. Use the calibrated transformation to output overlay images

Now that we got a rough estimation of the lidar-camera transformation, we can move on to create lidar-image overlay. To be more specific, this transformation is from camera to lidar, which means we see the camera frame as the world frame.

**Basic logic:**
- Overlay lidar points onto images by setting up the camera model, camera intrinsic, and lidar-camera extrinsic. 

- The yaml file that stores camera intrinsic and other params. It used to initialize the cameraInfo which is used to help cameraModel reproject lidar points to image space. 

- Once received an image, first get the lidar-camera transformation, then reproject the received lidar points to the camera frame (in pixel), finally draw the projected points on the received image and publish the modified image, i.e., the overlay lidar image.

- Once received a lidar message, unpack the raw lidar data to get the [x, y, z, intensity] formated points

The number of lidar points included in one PointCloud2 message is around 23~24K. It makes a quite long time to reproject the lidar points to image space and draw them on the current image. I also filtered out some faraway lidar points, besides those can't be projected on the image.

```
Received lidar data ...
23436
Received an image ... 
Received lidar data ...
23378
Received lidar data ...
23434
```

Launch file (`overlay_lidar_camera.launch`) is used to overlay lidar camera, where I slow down the rosbag play by 0.1:
```xml
<launch>
    <param name="use_sim_time" value="true" />

	<!-- Play the rosbag file to publish original sensor data -->
	<node name="rosbag" pkg="rosbag" type="play" 
	args="--clock -r 0.1 -s 22 -u 20 $(find ridecell)/bags/2016-11-22-14-32-13_test.bag -l"
	output="screen"
	/>

    <!-- Sync the raw images from rosbag and camera_info by publish modified image and camera_info with the same timestamp -->
	<node name="subpub" pkg="ridecell" type="pub_camera_info.py"
	args="$(find ridecell)/config/ost.yaml" 
	/>

	<!-- Run to subscribe topics image_raw and camera_info in namespace, eg., defined in ns="/ridecell" -->
	<node name="image_proc" pkg="image_proc" type="image_proc" ns="/ridecell" output="screen"
	/>

    <!-- Overlay lidar points on images by setting up the camera model and lidar-camera extrinsics -->
	<node name="overlay_lidar_image" pkg="ridecell" type="overlay_lidar_camera.py" args="$(find ridecell)/config/ost.yaml" output="screen">
		<remap from="image" to="/ridecell/image_rect_color"/>
		<remap from="lidar_image" to="/ridecell/lidar_image"/>
		<remap from="lidar" to="/sensors/velodyne_points"/>
	</node>

	<!-- rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 world velodyne 1000 -->
    <node name="static_transform" pkg="tf" type="static_transform_publisher"
    args="-0.53831089 -0.076088   -0.41682017  2.67673445  4.53119823  5.19606972 world velodyne 10"
    />
</launch>
```

**The screen casted overlay results can be found in `results/overlay_lidar_camera.mp4` or on [YouTube](https://youtu.be/08oPyJI3T1A).**

The screenshot of the overlay results is shown below:

![Task Two: Lidar and camera overlay](results/rviz_overlay.png)

The ros node graph is displayed for a final overview:
![Task Two: Lidar and camera overlay rqt graph](results/rosgraph.png)
