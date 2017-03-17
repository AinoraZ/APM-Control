from dronekit import connect as drone_connect
from dronekit import *
import exceptions
import tools
import config
import eventlet

eventlet.monkey_patch()

class DroneControl(object):
    def __init__(self, local, sio, app=""):
        self.app = app
        self.sio = sio
        self.vehicle = []
        self.success = False
        self.cmds = ""
        self.tries = 0
        while not self.success and self.tries < 10:
            self.connect(local=local)
            self.tries += 1
            time.sleep(0.5)
        if self.success:
            self.download_missions()
            self.clear_missions()

    def _test_mission(self):
        if not self.success:
            #print "Drone connection unsuccessful"
            self.sio.emit('my response', {'data': "Drone connection unsuccessful"})
            return []
        self.clear_missions()
        self.arm_and_takeoff(2)
        self.fly_to(-35.362753, 149.164526, 3)
        self.mission_change_alt(2)
        self.mission_RTL()
        self.mission_upload()
        self.vehicle_auto()

    def vehicle_auto(self):
        while self.vehicle.mode != VehicleMode("AUTO"):
            self.vehicle.mode = VehicleMode("AUTO")
            #print "Changing modes"
            self.sio.emit('my response', {'data': "Changing modes"})
            time.sleep(1)

    def download_missions(self):
        self.cmds.download()
        self.cmds.wait_ready()

    def clear_missions(self):
        self.cmds.clear()
        self.mission_upload()

    def mission_upload(self):
        self.cmds.upload()
        time.sleep(0.1)

    def fly_to(self, lat, lon, alt):
        cmd = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt)
        self.cmds.add(cmd)
        self.mission_upload()
        self.sio.emit('my response', {'data': "Mission: Fly to: " + str(lat) + " " + str(lon) + " " + str(alt)})

    def mission_RTL(self):
        cmd = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.cmds.add(cmd)
        self.mission_upload()
        self.sio.emit('my response', {'data': "Mission: Return To Launch"})

    def mission_change_alt(self, alt):
        cmd = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt)
        self.cmds.add(cmd)
        self.mission_upload()
        self.sio.emit('my response', {'data': "Mission: Change altitude to: " + str(alt)})

    def mission_set_home(self, lat=0, lon=0, alt=0):
        # Should not be used as it causes errors and unexpected behaviour (Only fixed in AC 3.3)
        home = 2
        if lat == 0 and lon == 0 and alt == 0:
            home = 1
        cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, home, 0, 0, 0, 0, 0, lat, lon, alt)
        self.cmds.add(cmd)
        self.mission_upload()

    def connect(self, local):
        try:
            if local:
                self.vehicle = drone_connect("127.0.0.1:14550", wait_ready=True)
            else:
                self.vehicle = drone_connect(tools.port_return(), baud=57600, wait_ready=True)
            self.cmds = self.vehicle.commands
            self.success = True
            self.sio.emit('my response', {'data': "Drone connected"})

        # Bad TTY connection
        except exceptions.OSError as e:
            #print 'No serial exists!'
            self.sio.emit('my response', {'data': 'No serial exists!'})

        # API Error
        except APIException:
            #print 'Timeout!'
            self.sio.emit('my response', {'data': 'Timeout!'})

        # Other error
        except:
            #print 'Some other error!'
            self.sio.emit('my response', {'data': 'Some other error!'})

    def pre_arm_check(self):
        while not self.vehicle.is_armable:
            #print " Waiting for vehicle to initialise..."
            self.sio.emit('my response', {'data': "Waiting for vehicle to initialise..."})
            time.sleep(1)

    def arm_and_takeoff(self, alt):
        if not self.success:
            return []

        if self.vehicle.armed:
            if self.vehicle.location.global_relative_frame.alt > 0.01:
                #print "Already in air. Flying to altitude."
                self.sio.emit('my response', {'data': "Already in air. Flying to altitude."})
                point1 = LocationGlobalRelative(self.vehicle.location.global_relative_frame.lat,
                                                self.vehicle.location.global_relative_frame.lon,
                                                alt)
                self.vehicle.simple_goto(point1)
        else:
            self.arm()

            if abs(self.vehicle.location.global_relative_frame.alt) < config.DRONE_GPS_FIXER:
                self.vehicle.simple_takeoff(alt)
                #print "Taking off!"
                self.sio.emit('my response', {'data': "Taking off!"})

            else:
                #print "Can't take off because of bad gps. Adjust config DRONE_GPS_FIXER"
                self.sio.emit('my response', {'data': "Can't take off because of bad gps. Adjust config DRONE_GPS_FIXER"})

        self.takeoff_monitor(alt)

    def takeoff_monitor(self, alt):
        fail_counter = 0
        prev_alt = self.vehicle.location.global_relative_frame.alt
        times_looped = 0

        while True:
            if times_looped == 10:
                #print " Altitude: ", self.vehicle.location.global_relative_frame.alt
                times_looped = 0
            if self.vehicle.location.global_relative_frame.alt <= prev_alt:
                fail_counter += 1
            if fail_counter > 50:
                #print "Taking off is experiencing difficulties! Switching to LAND"
                self.sio.emit('my response', {'data': "Taking off is experiencing difficulties! Switching to LAND"})
                self.land()
            if self.vehicle.location.global_relative_frame.alt >= alt*0.98:

                #print "Reached target altitude"
                self.sio.emit('my response', {'data': "Reached target altitude"})

                break
            prev_alt = self.vehicle.location.global_relative_frame.alt
            times_looped += 1
            time.sleep(0.1)


    def arm(self):
        if not self.success:
            return []

        self.vehicle.mode = VehicleMode("GUIDED")
        self.pre_arm_check()

        self.vehicle.armed = True

        while not self.vehicle.armed:
            #print "Waiting for arming..."
            self.sio.emit('my response', {'data': "Waiting for arming..."})
            time.sleep(1)

        #print "ARMED"
        self.sio.emit('my response', {'data': "ARMED"})

    def set_airspeed(self, air_speed):
        self.vehicle.airspeed = air_speed

    def land(self):
        self.vehicle.mode = VehicleMode("LAND")
        self.sio.emit('my response',{'data': "FORCE Landing"})
        time.sleep(1)

    def return_to_home(self):
        self.vehicle.mode = VehicleMode("RTL")
        self.sio.emit('my response',{'data': "FORCE Returning To Launch"})
        time.sleep(1)

    def get_status(self):
        pass
        #print self.vehicle.system_status
        #print self.vehicle.battery
        #print self.vehicle.groundspeed

    def emergency_disarm(self):
        # APM 3.3 or later
        cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0, 0)
        self.cmds.add(cmd)
        self.mission_upload()