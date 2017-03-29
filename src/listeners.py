import tools

class Listen(object):
    def __init__(self, worker):
        self.vehicle = worker.vehicle
        self._add_listeners()
        self.sio = worker.sio

    def attitude_listener(self, attribute, name, msg):
        att = self.vehicle.attitude
        obj = {'pitch': att.pitch, 'yaw': att.yaw, 'roll': att.roll}
        self.sio.emit('gyro', obj)
        #print obj

    def frame_listener(self, attribute, name, value):
        glob = self.vehicle.location.global_relative_frame
        obj = {'lat' : glob.lat, 'lng' : glob.lon, 'alt' : glob.alt}
        self.sio.emit('location', obj)
        #print obj

    def _add_listeners(self):
        self.vehicle.add_attribute_listener('attitude', self.attitude_listener)
        self.vehicle.add_attribute_listener('location.global_relative_frame', self.frame_listener)

    def _remove_listeners(self):
        self.vehicle.remove_attribute_listener('attitude', self.attitude_listener)
        self.vehicle.remove_attribute_listener('location.global_relative_frame', self.frame_listener)
