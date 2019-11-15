from face_detection import FaceDetector


if __name__ == "__main__":
    detector = FaceDetector(2, '/home/sam/git-repos/Intelligent-Robotics/vision/src/haarcascade_frontalface_default.xml')

    detector.cont_detect_faces()
