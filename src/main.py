from control import DroneControl
import socketio
import threading
import config
import eventlet.wsgi
from flask import Flask, render_template
from data_parser import ParseAndWork

if __name__ == '__main__':
    #eventlet.monkey_patch()
    sio = socketio.Server()
    worker = DroneControl(config.DRONE_LOCAL, sio)

    @sio.on('connect')
    def connect(sid, environ):
        pass
        #print('connect ', sid)

    @sio.on('mission')
    def message(sid, data):
        t = threading.Thread(target = ParseAndWork, kwargs={'data' : data, 'vehicle' : worker})
        t.daemon = True
        t.start()

    @sio.on('_test_mission')
    def message(sid):
        pass
        #t = threading.Thread(target = worker.arm_and_takeoff, kwargs={'alt': 2})
        #t.start()

    @sio.on('force_land')
    def message(sid):
        t = threading.Thread(target = worker.force_land)
        t.daemon = True
        t.start()

    @sio.on('force_rtl')
    def message(sid):
        t = threading.Thread(target = worker.force_RTL)
        t.daemon = True
        t.start()

    @sio.on('clear_missions')
    def message(sid):
        t = threading.Thread(target = worker.clear_missions)
        t.daemon = True
        t.start()

    @sio.on('vehicle_auto')
    def message(sid):
        t = threading.Thread(target = worker.vehicle_auto)
        t.daemon = True
        t.start()

    @sio.on("vehicle_connect")
    def vehicle_connect(sid):
        if not worker.success:
            worker.connect(config.DRONE_LOCAL)
        else:
            print "Already connected"

    @sio.on("vehicle_disconnect")
    def vehicle_disconnect(sid):
        if worker.success:
            worker.listener._remove_listeners()
        if worker.vehicle != []:
            worker.vehicle.close()
        else:
            print "Not connected!"
            sio.emit("response", {'data': "Not connected!"})
        worker.success = False
        print worker

    @sio.on('disconnect')
    def disconnect(sid):
        pass
        #print('disconnect ', sid)

    #app = Flask(__name__, template_folder='../templates')
    #app.debug = True

    #@app.route('/')
    #def index():
        #return render_template("index.html")

    app = socketio.Middleware(sio)
    eventlet.wsgi.server(eventlet.listen(('', 8001)), app)
