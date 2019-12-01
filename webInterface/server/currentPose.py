from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import rospy

class CurrentPose:
    def __init__(self, socket, topic):
        self.x = 0
        self.y = 0
        self.x_or = 0
        self.y_or = 0
        rospy.init_node('poser', anonymous=True)
        print("created poser node")
        self.sub = rospy.Subscriber('amcl_pose',PoseWithCovarianceStamped,self.callback)
        print("subscribed to amcl_pose")
        self.socket = socket
        self.topic = topic


    def callback(self,msg):
        # print(msg)
        pose = msg.pose.pose.position
        qur = msg.pose.pose.orientation
        self.x = pose.x
        self.y = pose.y
        self.x_or = qur.x
        self.y_or = qur.y
        self.socket.emit(self.topic, self.get_pose())

    def get_pose(self):
        print('get_pose ({})'.format({"x":self.x,"y":self.y}))
        return {"x":self.x,"y":self.y}

