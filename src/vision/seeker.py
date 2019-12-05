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
import requests
import os
import threading

class Seeker:
    vs = VideoStream(src=0).start()
    def __init__(self):
        self.CONFIDENT_GUESSES_THRESHOLD = 3
        self.ACTIVATION_THRESHOLD = 1
        self.HZ = 10
        self.confident_guesses = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry)
        self.odom = None
        self.rate = rospy.Rate(self.HZ)  # 10hz
        print("[INFO] loading encodings...")
        self.found = False
        self.FRAMES = 15
        self.data = pickle.loads(open(self.pickle_path(), "rb").read())


    def callback(self, msg):
        self.odom = msg

    def get_names(self):
       return list(set(self.data["names"]))

    def pickle_path(self):
        TEST_FILENAME = os.path.join(os.path.dirname(__file__), 'encodings.pickle')
        os.system(f"touch {TEST_FILENAME}")
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
                        try:
                            requests.get("http://localhost:4200/found/"+str(target))
                        except:
                            print("Couldnt send found")
                        try:
                            word="Hey, I have found you. How can I help?"
                            req = requests.get("http://localhost:5001/say/"+word)
                            if(req.status_code==200):
                                print("said Found")
                        except:
                            print("Failed to Say: Found")
                        

                        found = True
                        self.confident_guesses = 0
                    else:
                        self.confident_guesses += 1
                print(f"{name} found:{found},confident: {self.confident_guesses}, certainty: {(match_count / 35)}")
                names.append(name)
        return found

    def learn_face(self, name: str):
        knownEncodings = []
        knownNames = []
        for i in range(self.FRAMES):

            print("[INFO] processing image {}/{}".format(i, self.FRAMES))

            # load the input image and convert it from RGB (OpenCV ordering)
            # to dlib ordering (RGB)
            image = self.vs.read()
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # detect the (x, y)-coordinates of the bounding boxes
            # corresponding to each face in the input image
            boxes = face_recognition.face_locations(rgb,
                                                    model='hog')

            # compute the facial embedding for the face
            encodings = face_recognition.face_encodings(rgb, boxes)

            # loop over the encodings
            for encoding in encodings:
                # add each encoding + name to our set of known names and
                # encodings
                knownEncodings.append(encoding)
                print(len(set(self.data["names"])))
                # print(set(self.data["names"]))
                print(self.data["names"])
                
                new_index = len(set(self.data["names"])) #len(list(filter(self.data,lambda x: x['names'])))
                knownNames.append((name,new_index))

        # dump the facial encodings + names to disk
        print("[INFO] serializing encodings...")
        model_data = {'encodings': [], 'names': []}
        os.system(f"touch {self.pickle_path()}")
        
        with open(self.pickle_path(), "rb") as model:
            print("here3")
            try:
                model_data = pickle.load(model)
            except Exception as e:
                print(e)
                pass
            model_data['encodings'] += knownEncodings
            model_data['names'] += knownNames
            print("updated model data")
        with open(self.pickle_path(), "wb") as model:
            pickle.dump(model_data, model)
            self.data = model_data
            print("updated self.data")
        # cv2.destroyAllWindows()
        # vs.stop()

if __name__ == "__main__":
    s = Seeker()
    s.seek()
