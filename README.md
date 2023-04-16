# Aruco tags in Gazebo simulation

## Launching the bot

Run the following command
```
roslaunch mybot mybot.launch
```

If you don't have a bot already, you may use this: 

https://github.com/harshmittal2210/Robotics_ws#atom-robot-sdf

and run the following command 

```roslaunch atom world.launch```

Note: Here is the guide for the aformentioned bot: http://www.harshmittal.co.in/tutorials/Custom-Robot-ROS/


## Generating aruco tags in gazebo simulation 

Use this link to generate the tags: https://chev.me/arucogen/

1. Navigate to the aruco_in_gazebo/gazebo_models/ar_tags directory 
2. In the images folder add the .png images for the ar tags required. Note that they have to be resized to dimensions 170*170 px
3. Run the following command(the python file is in the aruco_in_gazebo/gazebo_models/ar_tags/scripts/ directory)
```
./generate_markers_model.py -i <path to the image directory> -s <size of the model in mm> -w <white contour aroud the images; put 0 if not required>
```
(example : `./generate_markers_model.py -i ~/robocon/gazebo_models/ar_tags/images -s 100 -w 0`)

5. The models should be present in ./gazebo/models (check hidden directories to get the gazebo directory (press ctrl h)). This should be in your home folder.
6. In gazebo the models should be available in the model insert tab. Insert the model and place it in front of your robot's camera (optional).

Credit to @mikaelaguerdas for the generator script

## Launching the aruco detection script

### Dependancies for this part

* Python 3.x
* Numpy
* OpenCV 3.3+
* OpenCV 4.6 Contrib modules (`pip install opencv-contrib-python==4.6.0.66`)

In the aruco_in_gazebo/scripts directory, launch the script with `python open_cv_demo_ros.py`
Edit the rostopic to whatever topic your camera is publishing to. Use `rostopic list` to get a list of topics.

Ensure you are setting the correct dictionary size needed in the script (see open_cv_demo_ros.py)

If you used the bot linked above, you may replace it by `sub_image = rospy.Subscriber("/atom/camera/rgb/image_raw", Image, image_callback)`

Use the ros2 script if you are using ros2. Update the aruco dict and the mtx and dist values from the calibration data.

## Camera Calliberation

### Installing the packages

#### For ros

Run the following command in your terminal.
`$ rosdep install camera_calibration`
Here is the guide: http://library.isr.ist.utl.pt/docs/roswiki/camera_calibration(2f)Tutorials(2f)MonocularCalibration.html

Documentation: http://wiki.ros.org/camera_calibration

Run the calibration node using the following script
`rosrun camera_calibration cameracalibrator.py --size <shape of checkerboard eg. 8x6)> --square <Size of individual square in meters> image:=<topic to which the camera is publishing to (will most likely end with image_raw)> camera:=<camera name(will most likely end with/camera)>`

The caliberation window will then open up. If the everything was done right you should see the scan value on the terminal as well colored calibration markings on the board. 
<img width="844" alt="Screenshot 2022-08-01 at 9 47 52 PM" src="https://user-images.githubusercontent.com/96363931/224107699-8c36f357-41b3-42f0-9dae-ab18088af356.png">

#### For ros2

Run the following commands
```
sudo apt install ros-<ros2-distro>-camera-calibration-parsers

sudo apt install ros-<ros2-distro>-camera-info-manager

sudo apt install ros-<ros2-distro>-launch-testing-ament-cmake

```
Image Pipeline need to be built from source in your workspace with:
`git clone – b <ros2-distro> git@github.com:ros-perception/image_pipeline.git`

To start calibration run 
`ros2 run camera_calibration cameracalibrator --size <checkerboard pattern> --square <square side length in meters> --ros-args -r image:=<raw image topic>-p camera:=<camera name eg. /my_camera>`

### Getting the data

To get all the calibration data required, use the following positions to place your boards in:
- checkerboard on the camera's left, right, top and bottom of field of view
  - X bar - left/right in field of view
  - Y bar - top/bottom in field of view
  - Size bar - toward/away and tilt from the camera
- checkerboard filling the whole field of view
- checkerboard tilted to the left, right, top and bottom (Skew)

As you move the checkerboard around you will see three bars on the calibration sidebar increase in length. When the CALIBRATE button lights, you have enough data for calibration and can click CALIBRATE to see the results.

After the calibration is complete you will see the calibration results in the terminal and the calibrated image in the calibration window and the save and commit buttons light up. 

Press the save button to see the result. Data is saved to “/tmp/calibrationdata.tar.gz”

To use the the calibration file unzip the calibration.tar.gz `tar -xvf calibration.tar.gz`

In the folder images used for calibration are available and also “ost.yaml” and “ost.txt”. You can use the yaml file which contains the calibration parameters as directed by the camera driver.
The same package and process can and is used to calibrate real life cameras as well, just edit the rostopics as required.
