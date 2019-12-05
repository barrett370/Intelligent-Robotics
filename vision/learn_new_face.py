import cv2
import pickle
import os
import face_recognition
from imutils.video import VideoStream
import logging

# log = lambda x: logging.log(logging.DEBUG, x)
MODEL_PATH = "encodings_new.pickle"
FRAMES = 15
log = lambda x: print(x)
log("[INFO] quantifying faces...")
# initialize the list of known encodings and known names
knownEncodings = []

knownNames = []
log("[INFO] starting video stream...")
vs = VideoStream(src=0).start()


def learn_face(name: str):
    for i in range(FRAMES):

        log("[INFO] processing image {}/{}".format(i, FRAMES))

        # load the input image and convert it from RGB (OpenCV ordering)
        # to dlib ordering (RGB)
        image = vs.read()
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
            knownNames.append(name)

    # dump the facial encodings + names to disk
    log("[INFO] serializing encodings...")
    model_data = {'encodings': [], 'names': []}
    os.system(f"touch {MODEL_PATH}")
    with open(MODEL_PATH, "rb") as model:
        try:
            model_data = pickle.load(model)
        except EOFError:
            pass
        model_data['encodings'] += knownEncodings
        model_data['names'] += knownNames
    with open(MODEL_PATH, "wb") as model:
        pickle.dump(model_data, model)
    cv2.destroyAllWindows()
    vs.stop()