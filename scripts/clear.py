import rospy
import os

def _off():
    os.system('rosservice call /move_base/clear_costmaps "{}"')

def piss():
    rospy.init_node('map_boi',annonymous=False)
    rate = rospy.Rate(0.3)
    while not rospy.is_shutdown():
        rate.sleep()
        _off()

if __name__ == "__main__":
    try:
        piss()
    except:
        pass