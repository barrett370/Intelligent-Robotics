
import rospy
from geometry_msgs.msg import Twist


RIGHT = -90
LEFT = 90
BACKWARDS = 180
FORWARD = 0
HZ = 10

def setup():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    print('seeker set publisher to cmd_vel')
    rospy.init_node('Mover', anonymous=True)
    print('initialised node to mover')
    rate = rospy.Rate(HZ)  # 10hz
    base_data = Twist()
    current_bearing = 0
    TURN_SCALAR = 500.0