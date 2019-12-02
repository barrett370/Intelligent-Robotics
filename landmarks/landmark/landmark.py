from flask import Flask,abort
import difflib
import rospy
from geometry_msgs.msg import PoseStamped

#to run the code if the robot is not running
try:
    import currentPose
    pose = currentPose.CurrentPose()

except:
    pose = {"x":0,"y":0} 



locationsDict = {
    "top left": {"x": 0.6,"y" :18.8} ,
    "top right" : {"x" : 26.3, "y" :18.8},
    "bottom left": {"x":2.1,"y":2.1},
    "bottom right":{"x":25.8,"y":2.11}
}
# rospy.init_node('goal_setter',anonymous=True)
pub = rospy.Publisher('/move_base/current_goal',PoseStamped)

app = Flask(__name__)

@app.route("/healthcheck")
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

@app.route("/go/<locString>")
def go(locString:str):
    landmark = getLandmark(locString)
    goal = PoseStamped()
    goal.pose.position.x = landmark['x']
    goal.pose.position.y = landmark['y']
    pub.publish(goal)
    return f"Going to {locString} at x:{landmark['x']}, y: {landmark['y']}"
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
    locationsDict[locString]= {"x":x,"y":y}
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

@app.route("/current")
def getCurrentPosition():
    try:
        return pose.get_pose()
    except:
        return pose

if __name__ == "__main__":
    app.run()

