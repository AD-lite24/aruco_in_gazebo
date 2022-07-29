#!/usr/bin/env python2.7

from ossaudiodev import control_labels
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2.aruco as aruco

#test
print ("Hello")
rospy.init_node('opencv_test', anonymous=True)
rospy.loginfo("Hello")

bridge = CvBridge()

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)


def image_callback(img_msg):

      ####---------------------- CALIBRATION ---------------------------
# termination criteria for the iterative algorithm
      criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# checkerboard of size (7 x 6) is used
      objp = np.zeros((6*7,3), np.float32)
      objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# arrays to store object points and image points from all the images.
      objpoints = [] # 3d point in real world space
      imgpoints = [] # 2d points in image plane.
      rospy.loginfo(img_msg.header)

      #convert the ROS Image message to a CV2 Image
      try:
          cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
      except CvBridgeError as e:
          rospy.logerr("CvBridge Error: {0}".format(e))
          
      

      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
      
      # find the chess board (calibration pattern) corners
      for i in range(5):
        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
        print(ret, corners)

    # if calibration pattern is found, add object points,
    # image points (after refining them)
      
        if ret == True:
                objpoints.append(objp)

                # Refine the corners of the detected corners
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                frame = cv2.drawChessboardCorners(frame, (7,6), corners2,ret)
                print('calliberation done')
                break
        else:
                print('calliberation failed')
                continue

      ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
      print('starting camera feed')

      # set dictionary size depending on the aruco marker selected
      aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

      # detector parameters can be set here (List of detection parameters[3])
      parameters = aruco.DetectorParameters_create()
      parameters.adaptiveThreshConstant = 10

      # lists of ids and the corners belonging to each id
      corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

     # font for displaying text (below)
      font = cv2.FONT_HERSHEY_SIMPLEX

       # check if the ids list is not empty
      # if no check is added the code will crash
      if np.all(ids != None):

          # estimate pose of each marker and return the values
          # rvet and tvec-different from camera coefficients
          rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
          (rvec-tvec).any() # get rid of that nasty numpy value array error

          for i in range(0, ids.size):
              # draw axis for the aruco markers
              cv2.drawFrameAxes(cv_image, mtx, dist, rvec[i], tvec[i], 0.1)

          # draw a square around the markers
          aruco.drawDetectedMarkers(cv_image, corners)


          # code to show ids of the marker found
          strg = ''
          for i in range(0, ids.size):
              strg += str(ids[i][0])+', '

          cv2.putText(cv_image, "Id: " + strg, (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


      else:
          # code to show 'No Ids' when no markers are found
          cv2.putText(cv_image, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

      #cv_image = cv2.flip(cv_image, 0)
      show_image(cv_image)


sub_image = rospy.Subscriber("/mybot/camera/image_raw", Image, image_callback)

while not rospy.is_shutdown():
      rospy.spin()
