from tkinter import Image
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2.aruco as aruco

bridge = CvBridge()

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/mybot/camera/image_raw',  #Enter rostopic
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, img_msg):
        
        #dummy mtx and dist values, use calibration to get actual values
        mtx = np.array([[534.34144581, 0., 339.15527826],[0., 534.68425884, 233.84359502],[  0., 0., 1.]])
        dist = np.array([[-2.88320983e-01, 5.41079711e-02, 1.73501622e-03, -2.61333898e-04, 2.04110457e-01]])

        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError as e:
            print(e)
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
      

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

        show_image(cv_image)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)


    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()