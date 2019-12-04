import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from imutils.video import VideoStream
import face_recognition
import argparse
import imutils
import pickle
import time
import cv2


class Seeker:

    def __init__(self):
        self.CONFIDENT_GUESSES_THRESHOLD = 3
        self.ACTIVATION_THRESHOLD = 5
        self.HZ = 10
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry)
        self.odom = None
        print('seeker set publisher to cmd_vel')
        rospy.init_node('Mover', anonymous=True)
        print('initialised node to mover')
        self.rate = rospy.Rate(self.HZ)  # 10hz
        print("[INFO] loading encodings...")
        self.data = pickle.loads(open("encodings.pickle", "rb").read())
        self.found = False

    def callback(self, msg):
        self.odom = msg

    def seek(self):
        print('seeking...')
        confident_guesses = 0
        base_data = Twist()
        # initialize the video stream and pointer to output video file, then
        # allow the camera sensor to warm up
        print("[INFO] starting video stream...")
        vs = VideoStream(src=0).start()
        # time.sleep(2.0)
        base_data.angular.z = 1.0
        found = False
        # loop over frames from the video file stream
        counter = 0
        while not found and counter < 20:
            print("Looping...")
            # grab the frame from the threaded video stream
            # self.pub.publish(base_data)
            # self.rate.sleep()
            frame = vs.read()

            # convert the input frame from BGR to RGB then resize it to have
            # a width of 750px (to speedup processing)
            rgb = imutils.resize(frame, height=240, width=426)

            # detect the (x, y)-coordinates of the bounding boxes
            # corresponding to each face in the input frame, then compute
            # the facial embeddings for each face
            boxes = face_recognition.face_locations(rgb,
                                                    model="hog")
            encodings = face_recognition.face_encodings(rgb, boxes)
            names = []

            # loop over the facial embeddings
            for encoding in encodings:
                # attempt to match each face in the input image to our known
                # encodings
                matches = face_recognition.compare_faces(self.data["encodings"],
                                                         encoding)

                # check to see if we have found a match
                if True in matches:
                    # find the indexes of all matched faces then initialize a
                    # dictionary to count the total number of times each face
                    # was matched
                    face_matches = [i for (i, b) in enumerate(matches) if b]
                    counts = {}
                    # loop over the matched indexes and maintain a count for
                    # each recognized face face
                    for i in face_matches:
                        name = self.data["names"][i]
                        counts[name] = counts.get(name, 0) + 1
                    name = max(counts, key=counts.get)
                    match_count = counts[name]
                    # sumCounts = sumCounts + matchCount

                    if match_count > self.ACTIVATION_THRESHOLD:
                        if confident_guesses > self.CONFIDENT_GUESSES_THRESHOLD:
                            base_data.angular.z = 0
                            print("found!")
                            found = True
                        else:
                            confident_guesses = confident_guesses + 1
                    print(name + " found, certainty: " + "{:%}".format((match_count / 35)))
                    names.append(name)
            counter += 1
            self.pub.publish(base_data)
            self.rate.sleep()

        # exit
        cv2.destroyAllWindows()
        vs.stop()


if __name__ == "__main__":
    s = Seeker()
    s.seek()
