import config


class ParseAndWork(object):
    def __init__(self, data, worker):
        self.data = data
        self.worker = worker
        self.worker.critical = False

        if not self.worker.taking_off:
            self.worker.clear_missions()
            for mission in self.data:
                if 'takeoff' in mission["name"]:
                    alt = mission["alt"]
                    self.takeoff(alt)
                elif 'fly_to' in mission["name"]:
                    lat = mission["lat"]
                    lon = mission["lng"]
                    alt = mission["alt"]
                    self.fly_to(lat, lon, alt)
                elif 'rtl' in mission["name"]:
                    self.rtl()
                elif 'land' in mission["name"]:
                    self.land()
                elif 'roi' in mission["name"]:
                    lat = mission["lat"]
                    lon = mission["lng"]
                    alt = mission["alt"]
                    self.roi(lat, lon, alt)
            self.worker.critical = False
            self.worker.mission_upload()
            self.worker.vehicle_auto_safe()
        else:
            print "Vehicle is currently taking off. Uploading missions is currently blocked."
            self.worker.sio.emit('response', {'data': "Vehicle is currently taking off. Uploading missions is currently blocked."})

    def takeoff(self, alt):
        self.worker.arm_and_takeoff(alt)

    def fly_to(self, lat, lon, alt):
        if self.worker.is_safe():
            altitude = alt
            if altitude <= 0:
                if self.worker.get_location_alt() > 1:
                    altitude = self.worker.get_location_alt()
                else:
                    altitude = config.DEFAULT_ALT
            self.worker.mission_fly_to(lat, lon, altitude)

    def rtl(self):
        if self.worker.is_safe():
            self.worker.mission_RTL()

    def land(self):
        if self.worker.is_safe():
            self.worker.mission_land()

    def roi(self, lat, lon, alt):
        if self.worker.is_safe():
            self.worker.mission_set_roi(lat, lon, alt)

