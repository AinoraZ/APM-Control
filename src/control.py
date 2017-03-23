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
        self.connect(local=local)
        if self.success:
            self.download_missions()
            self.clear_missions()

    def _test_mission(self):
        if not self.success:
            print "Drone connection unsuccessful"
            self.sio.emit('response', {'data': "Drone connection unsuccessful"})
            return []
        self.clear_missions()
        self.arm_and_takeoff(2)
        self.mission_fly_to(-35.362753, 149.164526, 3)
        self.mission_change_alt(2)
        self.mission_RTL()
        self.mission_upload()
        self.vehicle_auto()

    def vehicle_auto(self):
        while self.vehicle.mode != VehicleMode("AUTO"):
            self.vehicle.mode = VehicleMode("AUTO")
            print "Changing modes"
            self.sio.emit('response', {'data': "Changing modes"})
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

    def mission_fly_to(self, lat, lon, alt):
        cmd = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt)
        self.cmds.add(cmd)
        #self.mission_upload()
        self.sio.emit('response', {'data': "Mission: Fly to: " + str(lat) + " " + str(lon) + " " + str(alt)})
        time.sleep(0.2)

    def mission_RTL(self):
        cmd = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.cmds.add(cmd)
        #self.mission_upload()
        self.sio.emit('response', {'data': "Mission: Return To Launch"})
        time.sleep(0.2)

    def mission_change_alt(self, alt):
        cmd = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt)
        self.cmds.add(cmd)
        #self.mission_upload()
        self.sio.emit('response', {'data': "Mission: Change altitude to: " + str(alt)})
        time.sleep(0.2)

    def mission_set_home(self, lat=0, lon=0, alt=0):
        # Should not be used as it causes errors and unexpected behaviour (Only fixed in AC 3.3)
        home = 2
        if lat == 0 and lon == 0 and alt == 0:
            home = 1
        cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, home, 0, 0, 0, 0, 0, lat, lon, alt)
        self.cmds.add(cmd)
        #self.mission_upload()

    def connect(self, local):
        try:
            if local:
                self.vehicle = drone_connect("127.0.0.1:14550", wait_ready=True,
                                             heartbeat_timeout=config.DRONE_HEARTBEAT)
            else:
                self.vehicle = drone_connect(tools.port_return(), baud=57600, wait_ready=True,
                                             heartbeat_timeout=config.DRONE_HEARTBEAT)
            self.cmds = self.vehicle.commands
            self.success = True
            print 'Drone connected'
            self.sio.emit('response', {'data': "Drone connected"})

        # Bad TTY connection
        except exceptions.OSError as e:
            print 'No serial exists!'
            self.sio.emit('response', {'data': 'No serial exists!'})

        # API Error
        except APIException:
            print 'Timeout!'
            self.sio.emit('response', {'data': 'Timeout!'})

        # Other error
        except:
            print 'Some other error!'
            self.sio.emit('response', {'data': 'Some other error!'})

    def pre_arm_check(self):
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            self.sio.emit('response', {'data': "Waiting for vehicle to initialise..."})
            time.sleep(1)

    def arm(self):
        if not self.success:
            return []

        self.vehicle.mode = VehicleMode("GUIDED")
        #self.pre_arm_check()

        self.vehicle.armed = True

        while not self.vehicle.armed:
            print "Waiting for arming..."
            self.sio.emit('response', {'data': "Waiting for arming..."})
            time.sleep(1)

        print "ARMED"
        self.sio.emit('response', {'data': "ARMED"})

    def arm_and_takeoff(self, alt):
        if not self.success:
            return []

        ascend = True

        if self.vehicle.armed:
            if self.vehicle.location.global_relative_frame.alt > 0.01:
                print "Already in air. Flying to altitude."
                self.sio.emit('response', {'data': "Already in air. Flying to altitude."})
                point1 = LocationGlobalRelative(self.vehicle.location.global_relative_frame.lat,
                                                self.vehicle.location.global_relative_frame.lon,
                                                alt)
                self.vehicle.mode = VehicleMode("GUIDED")
                self.vehicle.simple_goto(point1)
                ascend = False
        else:
            self.arm()

            if abs(self.vehicle.location.global_relative_frame.alt) <= config.DRONE_GPS_FIXER:
                self.vehicle.simple_takeoff(alt)
                print "Taking off!"
                self.sio.emit('response', {'data': "Taking off!"})

            else:
                print "Can't take off because of bad gps. Adjust config DRONE_GPS_FIXER"
                self.sio.emit('response', {'data': "Can't take off because of bad gps. Adjust config DRONE_GPS_FIXER"})

        if self.vehicle.armed:
            self.takeoff_monitor(alt, ascend)
        else:
            print "Vehicle did not arm"

    def takeoff_monitor(self, alt, ascend):
        fail_counter = 0
        prev_alt = 0
        times_looped = 0

        while True:
            cur_alt = self.get_location_alt()
            if times_looped == 1:
                string = "Altitude: {}".format(cur_alt)
                print string
                self.sio.emit('response', {'data': string})
                times_looped = 0
            if self._fail_check(ascend, cur_alt, prev_alt):
                fail_counter += 1

            if fail_counter > config.FAIL_COUNTERS:
                if config.FAILSAFE_ON:
                    self.force_loiter()
                    self._emergency()
                    break

            if cur_alt >= alt*0.95 and cur_alt <= alt * 1.05:
                print "Reached target altitude"
                self.sio.emit('response', {'data': "Reached target altitude"})
                break

            prev_alt = cur_alt
            times_looped += 1
            time.sleep(0.6)

    def _fail_check(self, ascend, cur_alt, prev_alt):
        if ascend:
            if cur_alt <= prev_alt:
                return True
        else:
            if cur_alt >= prev_alt:
                return True
        return False

    def _print_failsafe(self):
        failsafe_string = ""
        if config.FAILSAFE_ON:
            failsafe_string = "Switching to LOITER"
        fail_string = "Taking off is experiencing difficulties!{}".format(failsafe_string)
        print fail_string
        self.sio.emit('response', {'data': '<font color="red"> Taking off is experiencing difficulties!</font>{}'.format(failsafe_string)})

    def _emergency(self):
        while self.vehicle.armed:
            self.sio.emit('response',{'data': '<font color="red"> EMERGENCY </font> Blocking commands in thread!'})
            time.sleep(1)

    def set_airspeed(self, air_speed):
        self.vehicle.airspeed = air_speed

    def force_land(self):
        self.vehicle.mode = VehicleMode("LAND")
        print "FORCE Landing"
        self.sio.emit('response',{'data': "FORCE Landing"})
        time.sleep(1)

    def force_loiter(self):
        self.vehicle.mode = VehicleMode("LOITER")
        print "FORCE Loiter"
        self.sio.emit('response',{'data': "FORCE Loiter"})
        time.sleep(1)

    def force_rtl(self):
        self.vehicle.mode = VehicleMode("RTL")
        self.sio.emit('response',{'data': "FORCE Returning To Launch"})
        time.sleep(1)

    def emergency_disarm(self):
        # APM 3.3 or later
        cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0, 0)
        self.cmds.add(cmd)
        self.mission_upload()

    def print_location(self):
        string = "Altitude: {}".format(self.get_location_alt())
        print string
        self.sio.emit('response', {'data': string})

    def get_location_alt(self):
        return self.vehicle.location.global_relative_frame.alt
