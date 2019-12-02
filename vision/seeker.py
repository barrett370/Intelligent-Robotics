import rospy
from geometry_msgs.msg import Twist
from imutils.video import VideoStream
import face_recognition
import argparse
import imutils
import pickle
import time
import cv2

RIGHT = -90
LEFT = 90
BACKWARDS = 180
FORWARD = 0
HZ = 10

# # construct the argument parser and parse the arguments
# argParser = argparse.ArgumentParser()
# argparser.add_argument("-e", "--encodings", required=True,
#                        help="SEEKER :path to serialized db of facial encodings")
# argparser.add_argument("-o", "--output", type=str,
#                        help="SEEKER: path to output video")
# argparser.add_argument("-y", "--display", type=int, default=1,
#                        help="SEEKER: whether or not to display output frame to screen")
# argparser.add_argument("-d", "--detection-method", type=str, default="hog",
#                        help="SEEKER face detection model to use: either `hog` or `cnn`")
# args = vars(argparser.parse_args())
#
# # load the known faces and embeddings
# print("SEEKER: [INFO] loading encodings...")
# data = pickle.loads(open(args["encodings"], "rb").read())
#
# # initialize the video stream and pointer to output video file, then
# # allow the camera sensor to warm up
# print("[INFO] starting video stream...")
# vs = VideoStream(src=0).start()
# writer = None
# time.sleep(2.0)
#

def setup():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    print('seeker set publisher to cmd_vel')
    rospy.init_node('Mover', anonymous=True)
    print('initialised node to mover')
    rate = rospy.Rate(HZ)  # 10hz
    base_data = Twist()
    searching = True
    current_bearing = 0
    TURN_SCALAR = 500.0
    while searching:
        base_data.angular.z = 0.2
        pub.publish(base_data)
        rate.sleep()


setup()

