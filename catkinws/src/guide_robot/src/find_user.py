import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.srv import GetMap
from std_msgs.msg import String
import actionlib

#  waits for map_server, which should be launched beforehand
rospy.wait_for_service('static_map')
map_getter = rospy.ServiceProxy('static_map', GetMap)
meta_data = map_getter().map.info
map_origin = meta_data.origin
map_resolution = meta_data.resolution
current_pose = None


class Howard:

    def __init__(self) -> None:
        self.current_pose = None
        rospy.init_node('howard_navigation')
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__callback_update_current_pose, queue_size=10)
        rospy.spin()

    def take_me_to(self, location: Pose) -> None:
        pass

    def __callback_update_current_pose(self, data) -> None:
        self.current_pose = data.pose.pose
        print(f"Current Pose: {self.current_pose}")
