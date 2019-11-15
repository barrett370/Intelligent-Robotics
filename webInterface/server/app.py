from flask import Flask, request, send_from_directory
from flask import render_template
from flask_socketio import SocketIO
from flask_socketio import send, emit
import requests 

req = requests.get("http://localhost:5000/getAllLandmarks")
print(req)
landmarks = req.json()
app = Flask(__name__, static_url_path='')
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

@app.route('/')
def root(): 
    print("/")
    return app.send_static_file('index.html')

@app.route('/img/map.png')
def map():
    print("image")
    return app.send_static_file('img/map.png')

@app.route('/broadcast')
def broadcast():
    socketio.emit("broadcast", {'landmarks': {"water cooler":{"x":2,"y":3}}})
    return "broadcasted"

#{"water cooler":{"x":2,"y":3},"water cooler2":{"x":7,"y":5},"water cooler3":{"x":3,"y":5}}}
@socketio.on('connected')
def handle_my_custom_event(json):
    socketio.emit("setup", {'locations': landmarks})
    socketio.emit("robot-update", {'x':5,'y':5})
    print('received json: ' + str(json))



if __name__ == "__main__":
    socketio.run(app, host='localhost', port=4200)