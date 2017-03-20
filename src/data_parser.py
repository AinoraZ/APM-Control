
class DataParser(object):
    def __init__(self, data):
        self.data = data
        for mission in self.data:
            alt = mission["alt"]
            lat = mission["lat"]
            lon = mission["lng"]
            if mission["name"].contains('takeoff'):
                self.takeoff(alt)
            elif mission["name"].contains('fly_to'):
                self.fly_to(lat, lon, alt)
            elif mission["name"].contains('rtl'):
                self.rtl()

    def takeoff(self, alt):
        print "taking off: " + str(alt)

    def fly_to(self, lat, lon, alt):
        print "flying to: " + str(lat) + " " + str(lon) + " " + str(alt)

    def rtl(self):
        print 'Return to launch'

