import cv2


class FaceDetector:

    def __init__(self, camera: int, classifier_path: str):
        self.face_cascade = cv2.CascadeClassifier(classifier_path)
        # To capture video from webcam.
        self.cap = cv2.VideoCapture(2)

    def detect_faces(self):
        while True:
            # Read the frame
            _, img = self.cap.read()
            # Convert to grayscale
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Detect the faces
            faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
            print(len(faces))
            # Draw the rectangle around each face
            for (x, y, w, h) in faces:
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            # Display
            cv2.imshow('img', img)
            # Stop if escape key is pressed
            k = cv2.waitKey(30) & 0xff
            if k == 27:
                self.cap.release()
                