from flask import Flask, request, send_from_directory
from flask import render_template
from flask_socketio import SocketIO
from flask_socketio import send, emit
import requests 
import asyncio
import logging
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import threading
import schedule
import time
import sys
import logging
import math
# from .currentPose import CurrentPose
import rospy

# logging.getLogger('werkzeug').setLevel(logging.ERROR)
# loop = asyncio.get_event_loop()

#to run the code if the robot is not running
# try:

    # logging.error("Loaded pose node.")
# except:
#     pose = {"x":0,"y":0} 


#get the landmarks from landmark server
# try:
#     req = requests.get("http://localhost:5000/getAllLandmarks")
#     landmarks = req.json()
# except:
#     landmarks = {"water cooler":{"x":5, "y":5}}

people = ["Jonathan","Geroge","Charlie","Sam","Anant"]

robotX = 5 #Dummy x position of robot
robotY = 5 # Dummy Y position of robot
app = Flask(__name__, static_url_path='')
app.config['SECRET_KEY'] = 'secret!'
lastPath=[]
print("app configured")
socketio = SocketIO(app,cors_allowed_origins="*")
print("setup socket")
# pose = CurrentPose(socketio, 'robot-update')

def callbackPath(msg):
    global lastPath
    temp=[]
    if(msg.poses != []):
        for i in range(0,len(msg.poses)):
            if(i%20==0):
                temp.append( (((msg.poses[i].pose.position.x+11.47)/2.54)+3.24) )
                temp.append( -(((msg.poses[i].pose.position.y-8.75)/2.53)-1.36))
                # temp.append([msg.poses[i].pose.position.x,msg.poses[i].pose.position.y])
        # print(temp)
        lastPath = temp
        socketio.emit("path-update",temp)
    else:
        if(lastPath != []):
            socketio.emit("path-update",[])
            lastPath = []

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]



def callback(msg):
    print("Esitimated Position")
    pose = msg.pose.pose.position
    qur = msg.pose.pose.orientation
    x = pose.x
    y = pose.y
    euler=quaternion_to_euler(qur.x,qur.y,qur.z,qur.w)
    socketio.emit("robot-update", {'x':x, 'y':y,"yaw":euler[0],"pitch":euler[1]})
    # print(euler)
    # print(x,y,qur)

rospy.init_node('poser', anonymous=True)
subPath = rospy.Subscriber('move_base/NavfnROS/plan',Path, callbackPath)
threading.Thread(target=lambda: rospy.init_node('poser', anonymous=True, disable_signals=True)).start()
sub = rospy.Subscriber('amcl_pose',PoseWithCovarianceStamped, callback)
pub = rospy.Publisher('cmd_vel',Twist, queue_size=1)
@app.route('/')
def root(): 
    print("/")
    return app.send_static_file('index.html')

@app.route('/img/map.png')
def map():
    print("image")
    return app.send_static_file('img/map.png')

@app.route('/updateLocations')
def updateLocations():
    global landmarks
    global people
    try:
        req = requests.get("http://localhost:5000/getAllLandmarks")
        landmarks = req.json()
        socketio.emit("setup", {'locations': landmarks,"people":people})
        return "updated"
    except:
        print("down")

#{"water cooler":{"x":2,"y":3},"water cooler2":{"x":7,"y":5},"water cooler3":{"x":3,"y":5}}}
@socketio.on('connected')
def handle_my_custom_event(json):
    updateLocations()
    # socketio.emit("robot-update",  pose.get_pose())
    statusCheck()
    print('received json: ' + str(json))

@socketio.on('newLandmark')
def newLandmark(json):
    try:
        # print(json)
        req = requests.get("http://localhost:5000/setLandmark/"+json["name"]+"/"+str(json["x"])+"/"+str(json["y"]))
        if(req.status_code==200):
            updateLocations()
    except:
        print("down")

@socketio.on('goTo')
def goTo(json):
    try:
        req = requests.get("http://localhost:5000/go/"+json["data"])
        if(req.status_code==200):
            console.log("on way")
            # updateLocations()
    except:
        print("down")

@socketio.on('say')
def say(json):
    try:
        req = requests.get("http://localhost:5001/say/"+json["data"])
        if(req.status_code==200):
            console.log("said "+json["data"])
            # updateLocations()
    except:
        print("say down")

@socketio.on('removeLandmark')
def removeLandmark(json):
    try:
        req = requests.get("http://localhost:5000/removeLandmark/"+json["name"])
        if(req.status_code==200):
            updateLocations()
    except:
        print("down")



@socketio.on('statusCheck')
def statusCheck():
    status = {}
    try:
        landmarks = requests.get("http://localhost:5000/healthCheck")
        status["LANDMARK"] = landmarks.status_code
    except:
        status["LANDMARK"] = 500

    try:
        voice = requests.get("http://localhost:5001/healthCheck")
        status["VOICE"] = voice.status_code
    except:
        # print("fail")
        status["VOICE"] = 500
    status["MOTION"] = 500
    socketio.emit("statusUpdate",status)



@socketio.on('keyPress')
def keyPress(json):
    #TODO: Replace with code that talks to motors
    global robotX
    global robotY
    key = json["data"]
    twist = Twist()
    if(key=="w"):
        twist.linear.x=1
        robotY-=0.02
    elif(key=="a"):
        robotX -= 0.02
    elif(key=="s"):
        robotY+=0.02
    elif(key=="d"):
        robotX +=0.02
    print(twist)
    pub.publish(twist)
    socketio.emit("robot-update", pose.get_pose())

def getCurrentPosition():
    try:
        return pose.get_pose()
    except: #This is done so it can run even if not connected to robot
        return pose


if __name__ == "__main__":
    socketio.run(app, host='0.0.0.0', port=4200)
