from flask import Flask,abort
import difflib
from .currentPose import CurrentPose


locationsDict = {
    "water cooler": {"x": 10,"y" :5} ,
    "lift" : {"x" : 11, "y" :11}
}


pose = CurrentPose()
app = Flask(__name__)

@app.route("/")
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
    # currentLocation = {"x":100, "y":20}
    return pose.get_pose()

if __name__ == "__main__":
    app.run()