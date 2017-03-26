import config

class ParseAndWork(object):
    def __init__(self, data, vehicle):
        self.data = data
        self.vehicle = vehicle
        self.vehicle.critical = False

        if not self.vehicle.taking_off:
            self.vehicle.clear_missions()
            for mission in self.data["Mission"]:
                lat = mission["lat"]
                lon = mission["lng"]
                alt = mission["alt"]
                if 'takeoff' in mission["name"]:
                    self.takeoff(alt)
                elif 'fly_to' in mission["name"]:
                    self.fly_to(lat, lon, alt)
                elif 'rtl' in mission["name"]:
                    self.rtl()
                elif 'land' in mission["name"]:
                    self.land()
            if config.AUTO_ON and self.vehicle.mode.name == "GUIDED" and self.is_safe():
                self.vehicle.vehicle_auto()
            self.vehicle.mission_upload()
            self.vehicle.critical = False

    def is_safe(self):
        return (self.vehicle.vehicle.armed and not self.vehicle.critical)

    def takeoff(self, alt):
        self.vehicle.arm_and_takeoff(alt)

    def fly_to(self, lat, lon, alt):
        if self.is_safe():
            altitude = alt
            if altitude <= 0:
                if self.vehicle.get_location_alt() > 1:
                    altitude = self.vehicle.get_location_alt()
                else:
                    altitude = config.DEFAULT_ALT
            self.vehicle.mission_fly_to(lat, lon, altitude)

    def rtl(self):
        if self.is_safe():
            self.vehicle.mission_RTL()

    def land(self):
        if self.is_safe():
            self.vehicle.mission_land()
