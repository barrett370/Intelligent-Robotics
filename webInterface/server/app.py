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


logging.getLogger('werkzeug').setLevel(logging.ERROR)
loop = asyncio.get_event_loop()





#to run the code if the robot is not running
try:
    import rospy
    from .currentPose import CurrentPose
    pose = CurrentPose()
except:
    pose = {"x":0,"y":0} 


#get the landmarks from landmark server
try:
    req = requests.get("http://localhost:5000/getAllLandmarks")
    landmarks = req.json()
except:
    landmarks = {"water cooler":{"x":5,"y":5}}

robotX = 5 #Dummy x position of robot
robotY= 5 # Dummy Y position of robot
app = Flask(__name__, static_url_path='')
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app,cors_allowed_origins="*")


def callback(msg):
        # print(msg)
        pose = msg.pose.pose.position
        qur = msg.pose.pose.orientation
        x = pose.x
        y = pose.y
        x_or = qur.x
        y_or = qur.y
        socketio.emit("robot-update", {'x':x,'y':y})
        print(x,y)

try:
    sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,callback)
    rospy.init_node('poser', anonymous=False)
except: 
    print("ROS not found")

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
    try:
        req = requests.get("http://localhost:5000/getAllLandmarks")
        landmarks = req.json()
        socketio.emit("setup", {'locations': landmarks})
        return "updated"
    except:
        print("down")

#{"water cooler":{"x":2,"y":3},"water cooler2":{"x":7,"y":5},"water cooler3":{"x":3,"y":5}}}
@socketio.on('connected')
def handle_my_custom_event(json):
    socketio.emit("setup", {'locations': landmarks})
    socketio.emit("robot-update", {'x':robotX,'y':robotY})
    statusCheck()
    print('received json: ' + str(json))

@socketio.on('newLandmark')
def newLandmark(json):
    try:
        req = requests.get("http://localhost:5000/setLandmark/"+json["name"]+"/"+str(json["x"])+"/"+str(json["y"]))
        if(req.status_code==200):
            updateLocations()
    except:
        print("down")

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
    status["VOICE"] = 500
    status["MOTION"] = 500
    socketio.emit("statusUpdate",status)



@socketio.on('keyPress')
def keyPress(json):
    #TODO: Replace with code that talks to motors
    global robotX
    global robotY
    key = json["data"]
    if(key=="w"):
        robotY-=0.02
    elif(key=="a"):
        robotX -= 0.02
    elif(key=="s"):
        robotY+=0.02
    elif(key=="d"):
        robotX +=0.02
    socketio.emit("robot-update", {'x':robotX,'y':robotY})


# import sched, time
# s = sched.scheduler(time.time, time.sleep)
# def do_something(sc): 
#     print ("Doing stuff...")
#     # do your stuff
#     s.enter( 60, 1, do_something, (sc,))

# s.enter(60, 1, do_something, (s,))
# s.run()


def getCurrentPosition():
    try:
        return pose.get_pose()
    except: #This is done so it can run even if not connected to robot
        return pose


if __name__ == "__main__":
    socketio.run(app, host='localhost', port=4200)