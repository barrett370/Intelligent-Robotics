import rospy
import os

def off():
    print('off')
    os.system('rosservice call /move_base/clear_costmaps "{}"')

def piss():
    rospy.init_node('map_boi')
    rate = rospy.Rate(0.3)
    print('hi')
    while not rospy.is_shutdown():
        rate.sleep()
        off()
    print('bye')

if __name__ == "__main__":
    try:
        piss()
    except Exception as e:
        print(e)
        print('except')
        pass
