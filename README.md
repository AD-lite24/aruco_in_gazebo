# Aruco tags in Gazebo simulation

## Launching the bot (optional) 

Run the following command
```
roslaunch mybot mybot.launch
```
Note: This is just for demonstration, launch any bot or camera configuration needed. Package may not always work for whatever reason so using a camera known to work on your system is recommended. Use the scripts folder and use it with the your package.
## Generating aruco tags in gazebo simulation 

1. Navigate to the Gazebo_models directory 
2. In the images folder add the .png images for the ar tags required. Note that they have to be resized to dimensions 170*170 px
3. Run the following command
```
./generate_markers_model.py -i <path to the image directory> -s <size of the model in mm> -w <white contour aroud the images; put 0 if not required>
```
5. The models should be present in ./gazebo/models (check hidden directories to get the gazebo directory)
6. In gazebo the models should be available in the model insert tab 

Credit to @mikaelaguerdas for the generator script

## Launching the aruco detection script

In the scripts directory, launch the script with `python open_cv_demo_ros.py`
Edit the rostopic to whatever topic your camera is publishing to. Use `rostopic list` to get a list of topics.
Use the ros2 script for ros2. Update the aruco dict and the mtx and dist values from the calibration data.

## Camera Calliberation

### Installing the packages

#### For ros

Run the following command in your terminal.
`$ rosdep install camera_calibration`

Run the calibration node using the following script
`rosrun camera_calibration cameracalibrator.py --size <shape of checkerboard eg. 8x6)> --square <Size of individual square in meters> image:=<topic to which the camera is publishing to (will most likely end with image_raw)> camera:=<camera name(will most likely end with/camera)>`

The caliberation window will then open up. If the everything was done right you should see the scan value on the terminal as well colored markings on the board. 

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
