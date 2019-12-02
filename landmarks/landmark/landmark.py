from flask import Flask,abort
import difflib
from .currentPose import CurrentPose
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
#to run the code if the robot is not running
# try:
#     # import currentPose
#     pose = CurrentPose()
# except:
#     pose = {"x":0,"y":0} 

pose = CurrentPose()

pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)

# except:
#     pose = {"x":0,"y":0} 


# def callback(msg):
#     # print(msg)
#     pose = msg.pose.pose.position
#     qur = msg.pose.pose.orientation
#     x = pose.x
#     y = pose.y
#     x_or = qur.x
#     y_or = qur.y


# sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, callback)
locationsDict = {
    "top left": {"x": -11.47,"y" :8.75} ,
    "top right" : {"x" : 11.89, "y" :8.61},
    "bottom left": {"x":-12.4,"y":-7.6},
    "bottom right":{"x":13.24,"y":-7.83}
}



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
            print(seq)
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
        return "You are near the " + landmark
    else: 
        return "You are not near anything"

@app.route("/go/<landmark>")
def go_to(landmark):
    print(f'go: {landmark}')
    loc = getLandmark(landmark)
    print(1)
    # loc = locationsDict[landmark]
    goal = PoseStamped()
    print(2)
    goal.pose.position.x = loc['x']
    print(3)
    goal.pose.position.y = loc['y']
    goal.pose.position.z = 0
    print(4)
    goal.pose.orientation = Quaternion(0,0,1,0)
    print(5)
    goal.header.frame_id = "map"
    print(6)
    pub.publish(goal)
    print("set goal position")
    print(goal.pose.position)
    return "success"


@app.route("/current")
def getCurrentPosition():
    try:
        return pose.get_pose()
    except:
        return pose

if __name__ == "__main__":
    app.run()