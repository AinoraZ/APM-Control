import tools

class Listen(object):
    def __init__(self, vehicle):
        self.vehicle = vehicle.vehicle
        self._add_listeners()
        self.sio = vehicle.sio

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
        obj = {'lat' : 0, 'lng' : 0, 'alt' : 0}
        self.sio.emit('location', obj)
        obj = {'pitch': 0, 'yaw': 0, 'roll': 0}
        self.sio.emit('gyro', obj)
        self.vehicle.remove_attribute_listener('attitude', self.attitude_listener)
        self.vehicle.remove_attribute_listener('location.global_relative_frame', self.frame_listener)

