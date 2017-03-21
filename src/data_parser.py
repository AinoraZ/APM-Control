import config

class ParseAndWork(object):
    def __init__(self, data, vehicle):
        self.data = data
        self.vehicle = vehicle
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
        if config.AUTO_ON:
            self.vehicle.vehicle_auto()
        self.vehicle.mission_upload()

    def takeoff(self, alt):
        self.vehicle.arm_and_takeoff(alt)

    def fly_to(self, lat, lon, alt):
        altitude = alt
        if altitude <= 0:
            if self.vehicle.get_location() > 0:
                altitude = self.vehicle.get_location()
            else:
                altitude = 2
        self.vehicle.mission_fly_to(lat, lon, altitude)

    def rtl(self):
        self.vehicle.mission_RTL()