# Aruco tags in Gazebo simulation

## Launching the bot 

Run the following command
```
roslaunch mybot mybot.launch
```
Note: This is just for demonstration, launch any bot or camera configuration needed
## Generating aruco tags in gazebo simulation 

1. Navigate to the Gazebo_models directory 
2. In the images folder add the .png images for the ar tags required. Note that they have to be resized to dimensions 170*170 px
3. Run the following command
```
./generate_markers_model.py -i <path to the image directory> -s <size of the model in mm> -w <white contour aroud the images; put 0 if not required>
```
5. The models should be present in ./gazebo/models (check hidden directories to get the gazebo directory)
6. In gazebo the models should be available in the model insert tab 

## Launching the aruco detection script

In the scripts directory, launch the script with `python open_cv_demo_ros.py`
Edit the rostopic to whatever topic your camera is publishing to. Use `rostopic list` to get a list of topics

## Camera Calliberation

Run the following command in your terminal.
`$ rosdep install camera_calibration`

Run the calibration node using the following script
`rosrun camera_calibration cameracalibrator.py --size <shape of checkerboard eg. 8x6)> --square <Size of individual square in meters> image:=<topic to which the camera is publishing to (will most likely end with image_raw)> camera:=<topic of your camera (will most likely end with/camera)>`

The caliberation window will then open up. If the everything was done right you should see the scan value on the terminal as well colored markings on the board. 

To get all the calibration data required, use the following positions to place your boards in:
- checkerboard on the camera's left, right, top and bottom of field of view
  - X bar - left/right in field of view
  - Y bar - top/bottom in field of view
  - Size bar - toward/away and tilt from the camera
- checkerboard filling the whole field of view
- checkerboard tilted to the left, right, top and bottom (Skew)

As you move the checkerboard around you will see three bars on the calibration sidebar increase in length. When the CALIBRATE button lights, you have enough data for calibration and can click CALIBRATE to see the results.

After the calibration is complete you will see the calibration results in the terminal and the calibrated image in the calibration window

