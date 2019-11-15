from flask import Flask, request, send_from_directory
from flask import render_template
from flask_socketio import SocketIO
from flask_socketio import send, emit
import requests 

import logging
logging.getLogger('flask_socketio').setLevel(logging.ERROR)

#get the landmarks from landmark server
req = requests.get("http://localhost:5000/getAllLandmarks")
landmarks = req.json()

robotX = 5 #Dummy x position of robot
robotY= 5 # Dummy Y position of robot
app = Flask(__name__, static_url_path='')
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, logger=False, engineio_logger=False)

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
    req = requests.get("http://localhost:5000/getAllLandmarks")
    landmarks = req.json()
    socketio.emit("setup", {'locations': landmarks})
    return "updated"

#{"water cooler":{"x":2,"y":3},"water cooler2":{"x":7,"y":5},"water cooler3":{"x":3,"y":5}}}
@socketio.on('connected')
def handle_my_custom_event(json):
    socketio.emit("setup", {'locations': landmarks})
    socketio.emit("robot-update", {'x':robotX,'y':robotY})
    print('received json: ' + str(json))

@socketio.on('newLandmark')
def newLandmark(json):
    req = requests.get("http://localhost:5000/setLandmark/"+json["name"]+"/"+str(json["x"])+"/"+str(json["y"]))
    if(req.status_code==200):
        updateLocations()

@socketio.on('removeLandmark')
def removeLandmark(json):
    req = requests.get("http://localhost:5000/removeLandmark/"+json["name"])
    if(req.status_code==200):
        updateLocations()


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


if __name__ == "__main__":
    socketio.run(app, host='localhost', port=4200)