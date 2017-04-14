import config


class Listen(object):
    def __init__(self, vehicle):
        self.vehicle = vehicle.vehicle
        self._add_listeners()
        self.sio = vehicle.sio

    def attitude_listener(self, attribute, name, msg):
        att = self.vehicle.attitude
        obj = {'pitch': att.pitch, 'yaw': att.yaw, 'roll': att.roll}
        self.sio.emit('gyro_info', obj)
        #print obj

    def frame_listener(self, attribute, name, value):
        obj = {'lat': value.lat, 'lng': value.lon, 'alt': value.alt}
        self.sio.emit('location_info', obj)

    def battery_listener(self, attribute, name, value):
        max_dif = 4.2 * config.BATTERY_CELL_COUNT - 3 * config.BATTERY_CELL_COUNT
        dif = value - (3 * config.BATTERY_CELL_COUNT)
        level = int(dif/max_dif * 100)

        obj = {'voltage': value, 'level': level}
        print obj
        self.sio.emit('battery_info', obj)

    def compass_listener(self, attribute, name, value):
            self.sio.emit('compass_info', value)

    def arm_listener(self, attribute, name, value):
        self.sio.emit('armed_info', value)

    def mode_listener(self, attribute, name, value):
        self.sio.emit('mode_info', value.name)

    def speed_listener(self, attribute, name, value):
        self.sio.emit('speed_info', {'groundspeed': self.vehicle.groundspeed, 'airspeed': value})

    def _add_listeners(self):
        self.vehicle.add_attribute_listener('attitude', self.attitude_listener)
        self.vehicle.add_attribute_listener('location.global_relative_frame', self.frame_listener)
        self.vehicle.add_attribute_listener('battery.voltage', self.battery_listener)
        self.vehicle.add_attribute_listener('heading', self.compass_listener)
        self.vehicle.add_attribute_listener('armed', self.arm_listener)
        self.vehicle.add_attribute_listener('mode', self.mode_listener)
        self.vehicle.add_attribute_listener('airspeed', self.speed_listener)

    def _remove_listeners(self):
        obj = {'pitch': 0, 'yaw': 0, 'roll': 0}
        self.sio.emit('gyro', obj)

        self.vehicle.remove_attribute_listener('attitude', self.attitude_listener)
        self.vehicle.remove_attribute_listener('location.global_relative_frame', self.frame_listener)
        self.vehicle.remove_attribute_listener('battery.voltage', self.battery_listener)
        self.vehicle.remove_attribute_listener('heading', self.compass_listener)
        self.vehicle.remove_attribute_listener('armed', self.arm_listener)
        self.vehicle.remove_attribute_listener('mode', self.mode_listener)
        self.vehicle.remove_attribute_listener('airspeed', self.speed_listener)
