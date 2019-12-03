from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import rospy
import threading
class CurrentPose:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.x_or = 0
        self.y_or = 0
        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback)
        threading.Thread(target=lambda: rospy.init_node('poser', anonymous=True, disable_signals=True)).start()

    def callback(self,msg):
        # print(msg)
        pose = msg.pose.pose.position
        qur = msg.pose.pose.orientation
        self.x = pose.x
        self.y = pose.y
        self.x_or = qur.x
        self.y_or = qur.y
    def get_pose(self):
        print({"x":self.x,"y":self.y})
        return {"x":self.x,"y":self.y}

