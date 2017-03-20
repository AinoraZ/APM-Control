
class DataParser(object):
    def __init__(self, data):
        self.data = data
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

    def takeoff(self, alt):
        print "taking off: " + str(alt)

    def fly_to(self, lat, lon, alt):
        print "flying to: " + str(lat) + " " + str(lon) + " " + str(alt)

    def rtl(self):
        print 'Return to launch'

