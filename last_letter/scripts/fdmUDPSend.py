#!/usr/bin/env python
# Send simState data as a UDP stream, packed as an FG_FDM object
# Based on runsim.py by Ardupilot Tools git

import roslib
import sys
import rospy
from pymavlink.rotmat import Vector3, Matrix3
from geometry_msgs.msg import Vector3 as RosVector3
import tf.transformations
from rosgraph_msgs.msg import Clock
from last_letter_msgs.msg import SimStates, SimPWM, Environment

import socket, struct, errno

# decode a string for IP and port
def interpret_address(addrstr):
    '''interpret a IP:port string'''
    a = addrstr.split(':')
    a[1] = int(a[1])
    return tuple(a)

# The data carrier class
class fdmState(object):
    def __init__(self):
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.heading = 0
        self.velocity = Vector3()
        self.accel = Vector3()
        self.gyro = Vector3()
        self.attitude = Vector3()
        self.airspeed = 0
        self.dcm = Matrix3()
        self.timestamp_us = 1

# Passes the new state data to the fdmState structure every new state update
def state_callback(state):
    global fdm
    fdm.latitude = state.geoid.latitude
    fdm.longitude = state.geoid.longitude
    fdm.altitude = state.geoid.altitude
    fdm.velocity = Vector3(state.geoid.velocity.x,
                               state.geoid.velocity.y,
                               state.geoid.velocity.z)

    (yaw, pitch, roll) = tf.transformations.euler_from_quaternion([state.pose.orientation.x,
                                                                       state.pose.orientation.y,
                                                                       state.pose.orientation.z,
                                                                       state.pose.orientation.w],'rzyx')
    fdm.attitude = Vector3(roll, pitch, yaw)
    fdm.gyro = Vector3(state.velocity.angular.x, state.velocity.angular.y, state.velocity.angular.z)
    fdm.dcm.from_euler(roll, pitch, yaw)

# Passes new bootstrapped accelerometer data to the fdmState structure every new update
def accel_callback(accel):
    global fdm
    accel = Vector3(accel.x, accel.y, accel.z)
    accel = fdm.dcm * accel
    accel.z -= 9.80665
    accel = fdm.dcm.transposed() * accel
    fdm.accel = accel

# Passes new wind and airspeed data to the fdmState structure every new update
def env_callback(environment):
    global fdm
    wind = Vector3(environment.wind.x, environment.wind.y, environment.wind.z)
    fdm.airspeed = (fdm.velocity - wind).length()

# Passes new timestamp data to the fdmState structure every new update
def clock_callback(clock):
    global fdm
    fdm.timestamp_us = int(clock.clock.secs * 1e6 + clock.clock.nsecs/1000)

# Receive control inputs from the APM SITL and publishes them in a SimPWM topic
def receive_input(sock, fdm, pub):
    buf = sock.recv(32)
    if len(buf) != 32:
        return False
    servos = struct.unpack('<16H', buf)
    ctrls = SimPWM()
    for i in range(11):
        ctrls.value[i] = servos[i]
    ctrls.header.stamp = rospy.Time.now()
    pub.publish(ctrls)
    return True

# Packages the fdmState data and sends it to the APM SITL
def send_output(sock, fdm):
    '''send output to SITL'''
    buf = struct.pack('<Q17d',
                      fdm.timestamp_us,
                      fdm.latitude,
                      fdm.longitude,
                      fdm.altitude,
                      fdm.heading,
                      fdm.velocity.x,
                      fdm.velocity.y,
                      fdm.velocity.z,
                      fdm.accel.x,
                      fdm.accel.y,
                      fdm.accel.z,
                      fdm.gyro.x,
                      fdm.gyro.y,
                      fdm.gyro.z,
                      fdm.attitude.x,
                      fdm.attitude.y,
                      fdm.attitude.z,
                      fdm.airspeed)
    sock.send(buf)

###############
## Main Program
###############

if __name__ == '__main__':
    fdm = fdmState()

    # Setup network infrastructure
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fdmPort = 9002 + int(rospy.get_param("instance"))*10
    sock.bind(('127.0.0.1', fdmPort))
    sock.connect(('127.0.0.1', fdmPort + 1))

    # Setup ROS node infrastructure
    rospy.init_node('fdmUDPSend')
    rospy.Subscriber('states', SimStates, state_callback, queue_size=1)
    rospy.Subscriber('linearAcc', RosVector3, accel_callback, queue_size=1)
    rospy.Subscriber('environment', Environment, env_callback, queue_size=1)
    rospy.Subscriber('clock', Clock, clock_callback, queue_size=1)
    pub = rospy.Publisher('ctrlPWM',SimPWM, queue_size=10)

    rate = rospy.get_param("/world/simRate");

    timer = rospy.Rate(rate)
    rospy.loginfo('fdmUDPSend node up')

    while not rospy.is_shutdown():
        try:
            if receive_input(sock, fdm, pub):
                send_output(sock, fdm)
        except socket.error as e:
            if e.errno not in [ errno.ECONNREFUSED ]:
                print("error in sending fdm packet: %s" % e)
                raise
        timer.sleep()
    print "fdmUDPSend node now closes"
