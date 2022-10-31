#! /usr/bin/env python

import rospy
import math
from math import sin, asin, cos, acos, tan, atan, atan2, exp, sqrt, pi
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
#from std_msgs import Float32

# TODO: Sensor readings are Float64, Thruster cmds are Float32. Verify that there are no issues
class ControllerClass:
    def __init__(self):
        # Initialze a ros node
        rospy.init_node('controller')
        
        # Create a subscriber to the sensor topics using as a callback function to extract values into local variables
        self.sub_imu = rospy.Subscriber('/sensors/imu/data', Imu, self.clbk_imu, queue_size=1)
        self.sub_gps = rospy.Subscriber('/sensors/gps/fix', NavSatFix, self.clbk_gps, queue_size=1)
        # TODO: Create topic for recieving lat/long setpoints
        
        # Create a publisher to the command topics
        self.pub_wl = rospy.Publisher('/ros_robot/thrusters/left_thrust_cmd', Float32, queue_size=1)
        self.pub_wr = rospy.Publisher('/ros_robot/thrusters/right_thrust_cmd', Float32, queue_size=1)
        self.pub_wt = rospy.Publisher('/ros_robot/thrusters/Center_thrust_cmd', Float32, queue_size=1)
        self.pub_wt_angle = rospy.Publisher('/ros_robot/thrusters/Center_thrust_angle', Float32, queue_size=1)


        #default values for the variables as a placeholder until the actual sensor values are recieved through from the ros topic
        self.imu_bx = 0
        self.imu_by = 0
        self.imu_bz = 0
        self.imu_ax = 0
        self.imu_ay = 0
        self.imu_az = 0
        self.gps_lat = 0
        self.gps_long = 0
        
        # setup global variables
        self.dt = 0.01          # sampling time
        self.I = 64.2           # angle between horizontal plane and magnetic vector
        self.D = 18             # deviation between magnetic and geographic north
        self.B = 5.7065e-5      # magnetic field strength
        self.g = 9.81           # acceleration due to gravity


    #Callback function for the sensor topics
    def clbk_imu(self, msg):
        self.imu_bx = msg.orientation.x                 # TODO: Verify values getting into variables
        self.imu_by = msg.orientation.y
        self.imu_bz = msg.orientation.z
        self.imu_ax = msg.linear_acceleration.x
        self.imu_ay = msg.linear_acceleration.y
        self.imu_az = msg.linear_acceleration.z
        
    def clbk_gps(self, msg):
        self.gps_lat = msg.latitude                     # TODO: Verify values getting into variables
        self.gps_long = msg.longitude

    def FilterGPS(self, lat_prev, long_prev): # exponential filter
        tau = 0.1                                                   # TODO: move to global variable
        k = 1 - exp(-self.dt/tau)
        
        lat = lat_prev + (self.gps_lat - lat_prev)*k
        long = long_prev + (self.gps_long - long_prev)*k
        
        return lat, long
    
    def FilterIMU(self, bx_prev, by_prev, bz_prev, ax_prev, ay_prev, az_prev): # exponential filter
        tau = 0.1                                                   # TODO: move to global variable
        k = 1 - exp(-self.dt/tau)
        
        bx = bx_prev + (self.imu_bx - bx_prev)*k
        by = by_prev + (self.imu_by - by_prev)*k
        bz = bz_prev + (self.imu_bz - bz_prev)*k
        ax = ax_prev + (self.imu_ax - ax_prev)*k
        ay = ay_prev + (self.imu_ay - ay_prev)*k
        az = az_prev + (self.imu_az - az_prev)*k

        return bx, by, bz, ax, ay, az
    
    def ProcessIMU(self,bx, by, bz, ax, ay, az):
        # TODO: zero angle need to be towards right. Range not important. can be [-inf,inf]
        pitch = asin(-ax/self.g)
        roll = atan(ay/az)
        yaw = atan((cos(pitch)*(bz*sin(roll) - by*cos(roll))) / (bx + self.B*sin(self.I)*sin(pitch))) + self.D
        # yaw = atan(-by / bx) + self.D # simplified yaw calculations. Must disable roll and pitch
        
        return yaw
        
    #main loop running the control logic
    def runController(self):
        #Defines the frequency in Hz in which the following loop will run
        r = rospy.Rate(100)
        finished = False
        
        # setup
        Kv = 350;       # Gain Kp distance controller
        Dv = 30;        # Gain Kd distance controller
        Iv = 0;         # Gain Ki distance controller
        Ka = 1800;      # Gain Kp angle controller
        Da = 200;       # Gain Kd angle controller
        ptc = 0.1;      # filter time constant
        offset = 0;     # offset set when point tracking
        k = 1-exp(-self.dt/ptc)
        
        # setup controller memory variables
        lat_prev, long_prev = 0
        bx_prev, by_prev, bz_prev, ax_prev, ay_prev,az_prev = 0
        ef_prev, df_prev = 0
        
        # setpoints TODO: should be recieved in topic
        lat_sp = 0
        long_sp = 0
        
        n = 0
        while not finished: # TODO: add automatic mode from topic
            # get new filtered inputs
            lat, long = self.FilterGPS(self, lat_prev, long_prev)
            lat_prev, long_prev = lat, long                                                                     # store previous values
            bx,by,bz,ax,ay,az = self.FilterIMU(self, bx_prev, by_prev, bz_prev, ax_prev, ay_prev, az_prev)
            bx_prev, by_prev, bz_prev, ax_prev, ay_prev, az_prev = bx,by,bz,ax,ay,az                            # store previous values
            
            # process IMU values
            theta = self.ProcessIMU(self, bx, by, bz, ax, ay, az)
            
            # calculate errors
            d = max(sqrt((lat_sp-lat)^2+(long_sp-long)^2) - offset,0);      # dist is positive
            angle = atan2((lat_sp-lat),(long_sp-long))                      # angle in [-pi,pi]
            e = (angle-theta) % (2*pi)
            if e > pi:                                                      # error in [-pi,pi]
                e = e - 2*pi
            
            # determine if vessel shall move forward or reverse
            if d<10 and e>pi/2:
                sign = -1
                e = e-pi
            elif d<10 and e<-pi/2:
                sign = -1
                e = e+pi
            else:
                sign = 1

            # low pass filter derivative for PD controller
            ef = ef_prev + (e-ef_prev)*k        # filter limits rate of change
            de = (ef - ef_prev) / self.dt       # calc derivative
            ef_prev = ef
            
            df = df_prev + (d-df_prev)*k        # filter limits rate of change
            dd = (df - df_prev) / self.dt       # calc derivative
            df_prev = df

            # calculate controller outputs
            ua = max(min(Ka*ef + Da*de,6000),-6000)
            uv = max(min((Kv*df + Dv*dd)*sign,5000),-5000)
            
            # change control outputs based of distance to target
            if d > 0.1:
                wl = uv * (1 - sign*ua/8000)
                wr = uv * (1 + sign*ua/8000)
                wt = ua
            else:
                wl = uv * (1 - sign*ua/2000)    # TODO: Add bias or increased gain???
                wr = uv * (1 + sign*ua/2000)    # TODO: Add bias or increased gain???
                wt = 0

            # Publish msg (scaled and inverted to [-1,1])
            self.pub_wl.publish(max(min(wl/-5000,1),-1))
            self.pub_wl.publish(max(min(wr/-5000,1),-1))
            self.pub_wl.publish(max(min(wt/-6000,1),-1))
            

            # sleeps for the time needed to ensure that the loop will be executed with the previously defined frequency
            r.sleep()

if __name__ == '__main__':
    nav = ControllerClass()
    nav.runController()
