# pyre-ignore-all-errors[21]
from vision.face_detection import FaceDetector

if __name__ == "__main__":
    detector = FaceDetector(0,
                            '../haarcascade_frontalface_default.xml')
    while True:
        print(detector.count_faces())
