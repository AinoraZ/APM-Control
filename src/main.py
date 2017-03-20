from control import DroneControl
import socketio
import threading
import config
import eventlet.wsgi
from flask import Flask, render_template
from cmd import Cmd

if __name__ == '__main__':
    sio = socketio.Server()

    worker = DroneControl(config.DRONE_LOCAL, sio)

    @sio.on('connect')
    def connect(sid, environ):
        print('connect ', sid)

    @sio.on('_test_mission')
    def message(sid):
        t = threading.Thread(target = worker._test_mission)
        t.start()

    @sio.on('force_land')
    def message(sid):
        t = threading.Thread(target = worker.land)
        t.start()

    @sio.on('disconnect')
    def disconnect(sid):
        print('disconnect ', sid)

    app = Flask(__name__, template_folder='../templates')
    app.debug = True

    @app.route('/')
    def index():
        return render_template("index.html")

    app = socketio.Middleware(sio, app)
    eventlet.wsgi.server(eventlet.listen(('', 8001)), app)

    while True:
        cmd = raw_input("Command: ")
        if not cmd.__contains__('worker.'):
            cmd = 'worker.' + cmd
        if not cmd.__contains__('('):
            cmd += '()'
        try:
            exec cmd
        except:
            print "Something went wrong"
