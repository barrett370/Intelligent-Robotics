import rospy
from geometry_msgs.msg import Twist
from imutils.video import VideoStream
import face_recognition
import argparse
import imutils
import pickle
import time
import cv2

# Movement code
CONFIDENT_GUESSES_THRESHOLD = 3
ACTIVATION_THRESHOLD = 20
HZ = 10
confidentGuesses = 0

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
print('seeker set publisher to cmd_vel')
rospy.init_node('Mover', anonymous=True)
print('initialised node to mover')
rate = rospy.Rate(HZ)  # 10hz
base_data = Twist()
found = False

# construct the argument parser and parse the arguments
argParser = argparse.ArgumentParser()
argParser.add_argument("-e", "--encodings", default="encodings.pickle",
                       help="SEEKER :path to serialized db of facial encodings")
argParser.add_argument("-o", "--output", type=str,  # default="output/webcam_face_recognition_output.avi",
                       help="SEEKER: path to output video")
argParser.add_argument("-y", "--display", type=int, default=1,
                       help="SEEKER: whether or not to display output frame to screen")
argParser.add_argument("-d", "--detection-method", type=str, default="hog",
                       help="SEEKER face detection model to use: either `hog` or `cnn`")
args = vars(argParser.parse_args())

# load the known faces and embeddings
print("[INFO] loading encodings...")
data = pickle.loads(open(args["encodings"], "rb").read())

# initialize the video stream and pointer to output video file, then
# allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
writer = None
time.sleep(2.0)

# loop over frames from the video file stream
while not found:
    # grab the frame from the threaded video stream
    base_data.angular.z = 0.2
    pub.publish(base_data)
    rate.sleep()
    frame = vs.read()

    # convert the input frame from BGR to RGB then resize it to have
    # a width of 750px (to speedup processing)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    rgb = imutils.resize(frame, height=240, width=426)

    # detect the (x, y)-coordinates of the bounding boxes
    # corresponding to each face in the input frame, then compute
    # the facial embeddings for each face
    boxes = face_recognition.face_locations(rgb,
                                            model=args["detection_method"])
    encodings = face_recognition.face_encodings(rgb, boxes)
    names = []

    # loop over the facial embeddings
    for encoding in encodings:
        # attempt to match each face in the input image to our known
        # encodings
        matches = face_recognition.compare_faces(data["encodings"],
                                                 encoding)
        name = "Unknown"

        # check to see if we have found a match
        if True in matches:
            # find the indexes of all matched faces then initialize a
            # dictionary to count the total number of times each face
            # was matched
            faceMatches = [i for (i, b) in enumerate(matches) if b]
            counts = {}
            # loop over the matched indexes and maintain a count for
            # each recognized face face
            for i in faceMatches:
                name = data["names"][i]
                counts[name] = counts.get(name, 0) + 1
            name = max(counts, key=counts.get)
            matchCount = counts[name]
            # sumCounts = sumCounts + matchCount

            if matchCount > ACTIVATION_THRESHOLD:
                if confidentGuesses > CONFIDENT_GUESSES_THRESHOLD:
                    found = True
                confidentGuesses = confidentGuesses + 1
            print(name + " found, certainty: " + "{:%}".format((matchCount / 35)))

        # update the list of names
        names.append(name)

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()




