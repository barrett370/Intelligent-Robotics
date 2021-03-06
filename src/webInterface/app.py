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
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import Path
import threading
import time
import sys
import logging
import numpy as np
import math
import pickle
import rospy
logging.getLogger('werkzeug').setLevel(logging.ERROR)
loop = asyncio.get_event_loop()
motionOn = False
app = Flask(__name__, static_url_path='')
app.config['SECRET_KEY'] = 'secret!'
lastPath=[]
socketio = SocketIO(app,cors_allowed_origins="*", async_handlers=True)
print("WEB SERVER STARTED")


def callbackPath(msg):
    global lastPath
    temp=[]
    if(msg.poses != []):
        for i in range(0,len(msg.poses)):
            if(i%20==0):
                temp.append( (((msg.poses[i].pose.position.x+11.47)/2.54)+3.24) )
                temp.append( -(((msg.poses[i].pose.position.y-8.75)/2.53)-1.36))
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


def get_people():
    try:
        req = requests.get("http://localhost:5000/faces")
        return req.json()
    except Exception as e:
        print(e)
        return []

def get_cameras():
    try:
        req = requests.get("http://localhost:5000/cameras")
        return req.json()
    except Exception as e:
        print(e)
        return []

def get_mics():
    try:
        req = requests.get("http://localhost:5000/mics")
        return req.json()
    except Exception as e:
        print(e)
        return []

def callback(msg):
    global motionOn
    pose = msg.pose.pose.position
    qur = msg.pose.pose.orientation
    x = pose.x
    y = pose.y
    euler=quaternion_to_euler(qur.x,qur.y,qur.z,qur.w)
    motionOn = True
    socketio.emit("robot-update", {'x':x, 'y':y,"yaw":euler[0],"pitch":euler[1]})

subPath = rospy.Subscriber('move_base/NavfnROS/plan',Path, callbackPath)
threading.Thread(target=lambda: rospy.init_node('poser', anonymous=False, disable_signals=True)).start()
sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, callback)
pub = rospy.Publisher('cmd_vel',Twist, queue_size=1)
init_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
twist = Twist()


@app.route('/')
def root(): 
    return app.send_static_file('index.html')

@app.route('/img/map.png')
def map():
    return app.send_static_file('img/map.png')

@app.route('/css/google.css')
def google():
    return app.send_static_file('css/google.css')

@app.route('/css/icon.css')
def icon():
    return app.send_static_file('css/icon.css')

@app.route('/js/socket.io.js')
def socket():
    return app.send_static_file('js/socket.io.js')


@app.route('/updateLocations')
def updateLocations():
    global landmarks
    try:
        req = requests.get("http://localhost:5000/getAllLandmarks")
        landmarks = req.json()
        people = get_people()
        cameras = get_cameras()
        mics = get_mics()
        socketio.emit("setup", {'locations': landmarks,"people":people, "camera":cameras,"mic":mics})
        return {'locations': landmarks,"people":people, "camera":cameras,"mic":mics}
    except Exception as e:
        return e
    

@app.route('/found/<id>')
def found(id):
    if(int(id)>=0):
        socketio.emit('found',{'id': id})
    cancel()
    return "success"



@socketio.on('setCam')
def set_cam(id):
    try:
        req = requests.get(f'http://localhost:5000/setCam/{id}')
        return f'{req.json()}'
    except Exception as e:
        print(f'[ERROR] {e}')
    return f'{e}'

@socketio.on('setMic')
def set_id(id):
    try:
        req = requests.get(f'http://localhost:5000/setMic/{id}')
        return f'{req.json()}'
    except Exception as e:
        print(f'[ERROR] {e}')
    return f'{e}'

@socketio.on('connected')
def handle_my_custom_event(json):
    updateLocations()
    statusCheck()
    print('received json: ' + str(json))

@socketio.on('resetPose')
def resetPose(json):
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id='map'
    pose.pose.pose.position.x=json['x']
    pose.pose.pose.position.y=json['y']
    pose.pose.pose.position.z=0
    q = euler_to_quaternion([(math.pi/180) *float(json["o"]),1,1])
    print((math.pi/180) *float(json["o"]))
    print(q)
    #(math.pi/180) *float(json["o"])
    pose.pose.pose.orientation= Quaternion(q[0], q[1], q[2], q[3])
    print(f'resetPose {pose}')
    init_pub.publish(pose)

def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

@socketio.on('newLandmark')
def newLandmark(json):
    try:
        req = requests.get("http://localhost:5000/setLandmark/"+json["name"]+"/"+str(json["x"])+"/"+str(json["y"]))
        if(req.status_code==200):
            updateLocations()
    except:
        print("Failed Set Landmark")

@socketio.on('goTo')
def goTo(json):
    try:
        req = requests.get("http://localhost:5000/go/"+json["data"])
        if(req.status_code==200):
            updateLocations()
            print("Go Success")
            # console.log("on way")
    except:
        print("Failed Set Goal")

@socketio.on('find')
def find(json):
    try:
        req = requests.get("http://localhost:5000/seek/"+json["data"])
        if(req.status_code==200):
            updateLocations()
            print("Find start Success")
            # console.log("on way")
    except:
        print("Failed to start Goal")

@socketio.on('say')
def say(json):
    try:
        req = requests.get("http://localhost:5001/say/"+json["data"])
        if(req.status_code==200):
            console.log("said "+json["data"])
            updateLocations()
    except:
        print("Failed to Say")

@socketio.on('removeLandmark')
def removeLandmark(json):
    try:
        req = requests.get("http://localhost:5000/removeLandmark/"+json["name"])
        if(req.status_code==200):
            updateLocations()
    except:
        print("Failed To remove landmark")


@socketio.on('cancel')
def cancel():
    print("Canceled Goal")
    try:
        req = requests.get("http://localhost:5000/cancel")
        if(req.status_code==200):
            updateLocations()
            socketio.emit("path-update",[])
    except:
        print("Failed To remove landmark")

@socketio.on('statusCheck')
def statusCheck():
    status = {}
    global motionOn
    try:
        landmarks = requests.get("http://localhost:5000/healthCheck")
        status["LANDMARK"] = landmarks.status_code
    except:
        status["LANDMARK"] = 500
    updateLocations()

    try:
        voice = requests.get("http://localhost:5001/healthCheck")
        status["VOICE"] = voice.status_code
    except:
        status["VOICE"] = 500
    if(motionOn):
        status["MOTION"] = 200
    else:
        status["MOTION"] = 500
    socketio.emit("statusUpdate",status)
    updateLocations()



@socketio.on('keyPress')
def keyPress(json):
    key = json["key"]
    if(json["event"]=="down"):
        if(key=="w"):
            twist.linear.x+=0.5
        elif(key=="a"):
            twist.angular.z=0.5
        elif(key=="s"):
            twist.linear.x+=-0.5
        elif(key=="d"):
            twist.angular.z=-0.5
    else:
        if(key=="w"):
            twist.linear.x=0
        elif(key=="a"):
            twist.angular.z=0
        elif(key=="s"):
            twist.linear.x =0
        elif(key=="d"):
            twist.angular.z=0
    pub.publish(twist)


if __name__ == "__main__":
    socketio.run(app, host='0.0.0.0', port=4200)
