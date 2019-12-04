from flask import Flask,abort
import difflib
from currentPose import CurrentPose
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
from seeker import Seeker
import threading
from itertools import cycle


# threading.Thread(target=lambda: rospy.init_node('landmarks', anonymous=True, disable_signals=True)).start()
rospy.init_node('landmarks', disable_signals=True)
pose = CurrentPose()

pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)

locationsDict = {"a":{"x":-9.6,"y":1.38},"b":{"x":0.12,"y":1.38},"c":{"x":10.38,"y":0.93}}
print("LANDMARKS SERVER RUNNING")

app = Flask(__name__)

@app.route("/healthCheck")
def hello():
    return "Landmarks Server is Running"

#Return the coords of a specific landmark
#Returns the coords if found in dict, if name is similar it returns check for confirmation, if the name is not similar is returns error
@app.route("/getLandmark/<locString>")
def getLandmark(locString):
    if (locString in locationsDict):
        return locationsDict[locString]
    else:
        sim,landmark = 0 , ""
        for i in locationsDict:
            seq = difflib.SequenceMatcher(a= i, b=  locString).ratio()
            if seq > sim:
                sim = seq 
                landmark = i
        if sim > 0.8:
            return locationsDict[landmark]
        elif sim >0.6:
            return {"check":landmark}
        else:
            return {"error": "nothing found"}

#Return all landmarks
@app.route("/getAllLandmarks")
def getAllLandmarks():
    return locationsDict

#create a new Landmark based on current posiition
@app.route("/setLandmark/<locString>")
def setLandmark(locString):
    locationsDict[locString]= getCurrentPosition()
    return  "success"

#create a new Landmark based on position
@app.route("/setLandmark/<locString>/<x>/<y>")
def setLandmarkXY(locString,x,y):
    locationsDict[locString]= {"x":float(x),"y":float(y)}
    return  "success"

#Remove a landmark
@app.route("/removeLandmark/<locString>")
def removeLandmark(locString):
    locationsDict.pop(locString)
    return  "success"

#Get the relative location of the robot
@app.route("/getRelLoc")
def getRelLoc():
    distance,landmark = 10000 , ""
    for i in locationsDict:
        d = abs(getCurrentPosition()["x"] - locationsDict[i]["x"]) + abs(getCurrentPosition()["y"]- locationsDict[i]["y"])
        if distance > d:
            distance = d
            landmark = i
    if distance < 15:
        return {"text":"You are near the " + landmark}
    else: 
        return {"text":"You are not near anything"}

@app.route("/go/<landmark>")
def go_to(landmark):
    print(f'go: {landmark}')
    loc = getLandmark(landmark)
    goal = PoseStamped()
    goal.pose.position.x = loc['x']
    goal.pose.position.y = loc['y']
    goal.pose.position.z = 0
    goal.pose.orientation = Quaternion(0,0,1,0)
    goal.header.frame_id = "map"
    pub.publish(goal)
    print("set goal position")
    return "success"


@app.route("/current")
def getCurrentPosition():
    try:
        return pose.get_pose()
    except:
        return pose

seek_lock = threading.Lock()
seeking = False
seeker = Seeker()
seek_locations = ['a', 'b', 'c']
current_seek = cycle(seek_locations)
spinning = False
goalReached = ""
spin_lock = threading.Lock()

@app.route("/seek")
def seek():
    global seeking
    global seek_lock
    # global seek_locations
    global current_seek
    print(f"seek={seeking}")
    seek_lock.acquire()
    seeking = True
    print("set seek to true")
    goal = next(current_seek)
    print(f"current seek = {goal}")
    seek_lock.release()
    print(f"current seek = {goal}")
    go_to(goal)
    return f"going to {goal}"



def callbackStatus(msg):
    global seeking
    global seek_lock
    global seek_locations
    global current_seek
    global spin_lock
    global spinning
    global goalReached
    # ss = [i.text for i in msg.status_list]
    if len(msg.status_list) > 0 and "Goal reached." == msg.status_list[-1].text and goalReached != msg.status_list[-1].goal_id.id:
        print("Goal reached.")
        goalReached = msg.status_list[-1].goal_id.id
        spin_lock.acquire()
        if seeking and not spinning:
            print("Seeking")

            # spin_lock.acquire()
            spinning = True
            spin_lock.release()

            found = seeker.seek()

            

            if found:
                print(found)
                seek_lock.acquire()
                seeking = False
                seek_lock.release()
                # while next(seek_locations) not 'a':
                #     print("Recyling seek locationse")
            else:
                next_goal = next(current_seek)
                print(f"Going to next goal: {next_goal}")
                go_to(next_goal)
                goalReached=""

            spin_lock.acquire()
            spinning = False
        spin_lock.release()

subGoal = rospy.Subscriber('move_base/status', GoalStatusArray, callbackStatus)



if __name__ == "__main__":
    app.run()
