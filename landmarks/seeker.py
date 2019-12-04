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
import os


class Seeker:

    def __init__(self):
        self.CONFIDENT_GUESSES_THRESHOLD = 3
        self.ACTIVATION_THRESHOLD = 1
        self.HZ = 10
        self.confident_guesses = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry)
        self.odom = None
        print('seeker set publisher to cmd_vel')
        print('initialised node to mover')
        self.rate = rospy.Rate(self.HZ)  # 10hz
        print("[INFO] loading encodings...")
        self.data = pickle.loads(open(self.__pickle_path(), "rb").read())
        self.found = False
        self.vs = VideoStream(src=0).start()

    def callback(self, msg):
        self.odom = msg

    
    def __pickle_path(self):
        TEST_FILENAME = os.path.join(os.path.dirname(__file__), 'encodings.pickle')
        print(TEST_FILENAME)
        return TEST_FILENAME

    def seek(self, target):
        print('seeking...')
        base_data = Twist()
        found = False
        # initialize the video stream and pointer to output video file, then
        # allow the camera sensor to warm up
        print("[INFO] starting video stream...")
        
        # time.sleep(2.0)
        base_data.angular.z = 1.0
        
        # loop over frames from the video file stream
        counter = 0
        while not found and counter < 20:
            
            found = self.scan(target)
            counter += 1
            self.pub.publish(base_data)
            self.rate.sleep()
        base_data.angular.z = 0
        self.pub.publish(base_data)
        # exit
        # cv2.destroyAllWindows()
        return found

    def scan(self, target):
        frame = self.vs.read()
        found = False
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

                if match_count >= self.ACTIVATION_THRESHOLD and int(name) == int(target):
                    if self.confident_guesses > self.CONFIDENT_GUESSES_THRESHOLD:
                        print("found!")
                        found = True
                        self.confident_guesses = 0
                    else:
                        self.confident_guesses += 1
                print(f"{name} found:{found},confident: {self.confident_guesses}, certainty: {(match_count / 35)}")
                names.append(name)
        return found


if __name__ == "__main__":
    s = Seeker()
    s.seek()
