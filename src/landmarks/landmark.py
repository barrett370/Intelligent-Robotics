from flask import Flask, abort
import difflib
from landmarks.currentPose import CurrentPose
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray

import threading
import requests
import random
from itertools import cycle
import json
from speech import get_inputs
from vision import get_cameras

import sys
from vision.seeker import Seeker
from speech.speech_to_text import gspeech_live

sys.path.append('../')

rospy.init_node('landmarks', anonymous=False, disable_signals=True)
pose = CurrentPose()

seek_lock = threading.Lock()
seeking = False
seeker = Seeker()
stt = gspeech_live.Listener()
thread = threading.Thread(target=stt.main, daemon=True)

thread.start()
seek_locations = ['a', 'b', 'c']
current_seek = cycle(seek_locations)
spinning = False
goalReached = ""
spin_lock = threading.Lock()
target = ''
targetName = ""
seek_steps_done = 0
sayCounter = 0
followStrings = ["Follow me", "This way", "Follow me to your destination", "I'm over here", "Almost there",
                 "Head towards me", "Head towards the sound of my voice", "Your destination is this way"]

pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
cancelPub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
locationsDict = {"a": {"x": -9.6, "y": 1.38}, "b": {"x": 0.12, "y": 1.38}, "c": {"x": 10.38, "y": 0.93}}
print("LANDMARKS SERVER RUNNING")

app = Flask(__name__)


@app.route("/healthCheck")
def hello():
    return "Landmarks Server is Running"


# Return the coords of a specific landmark
# Returns the coords if found in dict, if name is similar it returns check for confirmation, if the name is not similar is returns error
@app.route("/getLandmark/<locString>")
def getLandmark(locString):
    if (locString in locationsDict):
        return locationsDict[locString]
    else:
        sim, landmark = 0, ""
        for i in locationsDict:
            seq = difflib.SequenceMatcher(a=i, b=locString).ratio()
            if seq > sim:
                sim = seq
                landmark = i
        if sim > 0.8:
            return locationsDict[landmark]
        elif sim > 0.6:
            return {"check": landmark}
        else:
            return {"error": "nothing found"}


# Return all landmarks
@app.route("/getAllLandmarks")
def getAllLandmarks():
    return locationsDict


# create a new Landmark based on current posiition
@app.route("/setLandmark/<locString>")
def setLandmark(locString):
    locationsDict[locString] = getCurrentPosition()
    return "success"


# create a new Landmark based on position
@app.route("/setLandmark/<locString>/<x>/<y>")
def setLandmarkXY(locString, x, y):
    locationsDict[locString] = {"x": float(x), "y": float(y)}
    return "success"


# Remove a landmark
@app.route("/removeLandmark/<locString>")
def removeLandmark(locString):
    locationsDict.pop(locString)
    return "success"


# Get the relative location of the robot
@app.route("/getRelLoc")
def getRelLoc():
    distance, landmark = 10000, ""
    for i in locationsDict:
        d = abs(getCurrentPosition()["x"] - locationsDict[i]["x"]) + abs(
            getCurrentPosition()["y"] - locationsDict[i]["y"])
        if distance > d:
            distance = d
            landmark = i
    if distance < 15:
        return {"text": "You are near the " + landmark}
    else:
        return {"text": "You are not near anything"}


def go(landmark, seek):
    global seeking
    global seek_lock
    if seek:
        seek_lock.acquire()
        seeking = False
        seek_lock.release()

    print(f'go: {landmark}')
    loc = getLandmark(landmark)
    goal = PoseStamped()
    goal.pose.position.x = loc['x']
    goal.pose.position.y = loc['y']
    goal.pose.position.z = 0
    goal.pose.orientation = Quaternion(0, 0, 1, 0)
    goal.header.frame_id = "map"
    pub.publish(goal)
    print("[INFO] set goal position")
    return "success"


@app.route("/go/<landmark>")
def go_to(landmark):
    return go(landmark, False)


@app.route("/current")
def getCurrentPosition():
    try:
        return pose.get_pose()
    except:
        return pose


@app.route("/cancel")
def cancel():
    global seeking
    global seek_lock
    seek_lock.acquire()
    seeking = False
    seek_lock.release()
    print("Canceled Goal")
    cancelPub.publish()
    return 'success'


def loop_scan(target):
    global seeker
    global seeking
    while seeking:
        if seeker.scan(int(target)):
            seek_lock.acquire()
            seeking = False
            seek_lock.release()
            cancel()
            print("[INFO] found target while in motion!")


@app.route("/seek/<name>")
def seek(name: str):
    global target
    global seeking
    global seek_lock
    global current_seek
    global targetName
    global seek_steps_done
    seek_steps_done = 0
    target = None
    names = seeker.get_names()
    for n in names:
        if n[0] == name:
            target = n[1]
    if target is None:
        return "Person not found"
    targetName = name
    print("[INFO] current target:", target)
    # target = name
    print(f"[INFO] seek={seeking}")
    seek_lock.acquire()
    seeking = True
    # print("set seek to true")
    goal = next(current_seek)
    # print(f"current seek = {goal}")
    seek_lock.release()
    print(f"current seek = {goal}")
    go_to(goal)
    threading.Thread(target=loop_scan(target)).start()
    return f"[INFO] going to {goal}"


@app.route("/learn/<name>")
def learn_name(name: str):
    try:
        seeker.learn_face(name)
        print("learned face")
        return {"text": "success"}
    except Exception as e:
        print(f"[ERROR] {e}")
        return {"text": e}


@app.route("/faces")
def get_faces():
    names = seeker.get_names()
    return json.dumps(names)


@app.route("/mics")
def mics():
    return json.dumps(get_inputs.get())


@app.route("/cameras")
def cameras():
    return json.dumps(get_cameras.get())


@app.route('/setCam/<id>')
def set_cam(id):
    global seeker
    print(f'[INFO] setting cam to: {id}')
    seeker.change_device(int(id))


@app.route('/setMic/<id>')
def set_id(id):
    global stt
    print(f'[INFO] setting audio to: {id}')
    stt.change_mic_id(int(id))


def callbackStatus(msg):
    global seeking
    global seek_lock
    global seek_locations
    global current_seek
    global spin_lock
    global spinning
    global goalReached
    global target
    global seeker
    global seek_steps_done
    global sayCounter
    if (len(msg.status_list) > 0 and "This goal has been accepted by the simple action server" == msg.status_list[
        -1].text and not seeking):
        sayCounter += 1
        if (sayCounter % 25 == 0):
            word = random.choice(followStrings)
            try:
                req = requests.get("http://localhost:5001/say/" + word)
                if (req.status_code == 200):
            # print("said Follow me")
            except:
                print("[ERROR] Failed to Say:" + word)
            # print("Follow me")
    if len(msg.status_list) > 0 and "Goal reached." == msg.status_list[-1].text and goalReached != msg.status_list[
        -1].goal_id.id:
        print("[INFO] Goal reached.")
        goalReached = msg.status_list[-1].goal_id.id
        spin_lock.acquire()
        if seeking and not spinning:
            print("[INFO] Seeking")

            # spin_lock.acquire()
            spinning = True
            spin_lock.release()
            # print("TTTTTTTTT", target, targetName)

            try:
                req = requests.get("http://localhost:5001/say/Where are you " + targetName)
                if (req.status_code != 200):
                    # print("said Follow me")
                    print("[ERROR] Failed to Say:" + word)
            except:
                print("[ERROR] Failed to Say:" + word)
            found = seeker.seek(int(target))

            if found:
                print(f"[INFO] {found}")
                seek_lock.acquire()
                seeking = False
                seek_lock.release()
                # while next(seek_locations) not 'a':
                #     print("Recyling seek locationse")
            else:
                next_goal = next(current_seek)
                print(f"[INFO] Going to next goal: {next_goal}")
                go_to(next_goal)
                seek_steps_done += 1
                # goalReached=""

            spin_lock.acquire()
            spinning = False

        spin_lock.release()
        if not seeking:
            word = "[INFO] You have reached your destination"
            try:
                req = requests.get("http://localhost:5001/say/" + word)
                if req.status_code != 200:
                    print("[ERROR] Failed to Say:" + word)
                    # print("said Arrival")
            except:
                print("[ERROR] Failed to Say:" + word)
            try:
                req = requests.get("http://localhost:4200/found/-1")
                # if (req.status_code == 200):
                # print("Web communicated arrival")
            except:
                # print("Failed to communicate arrival with web")
                pass


subGoal = rospy.Subscriber('move_base/status', GoalStatusArray, callbackStatus)

if __name__ == "__main__":
    app.run()
