#!/usr/bin/env python

'''
 * haya_imu_node.py
 *
 * License: BSD-3-Clause
 *
 * Copyright (c) 2012-2023 Shoun Corporation <research.robosho@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of RT Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 '''

import rospy
from haya_motor_ros.msg import *
from haya_imu_ros.msg import *
from std_msgs.msg import Float32

PI = 3.1415926

#
# Callback for subscribing /arduino_msg, also publishing pid_params
#
def arduino_info_cb(arduino_msg):
    global pid_params, pub_pid_params, only_3_times

    if(only_3_times):
        pub_pid_params.publish(pid_params)
        #only_3_times -= 1
    rospy.loginfo("[python-node] Kp Ki Kd: %.3f %.3f %.3f, target_velocity: %.2f, current_velocity: %.2f, pwm: %d", pid_params.Kp, pid_params.Ki, pid_params.Kd, pid_params.target_velocity, arduino_msg.current_velocity, arduino_msg.pwm)

#
# Callback for subscribing /imu_data, also publishing angular_velocity.z
#
def imu_data_cb(imu_msg):
    #pub_z.publish(-imu_msg.angular_velocity.z * PI / 180.0)
    pub_z.publish(-imu_msg.angular_velocity.z)
    #rospy.loginfo("[python-node] imu_data/angular_velocity/z: %.2f", imu_msg.angular_velocity.z * PI / 180.0)

#
# main for python node
#
def main():
    global pid_params, pub_pid_params, only_3_times, pub_z

    rospy.init_node('haya_motor_node')
    only_3_times = 3
    pid_params = PidMsgs()
    pid_params.Kp = rospy.get_param('/haya_motor_node/Kp')
    pid_params.Ki = rospy.get_param('/haya_motor_node/Ki')
    pid_params.Kd = rospy.get_param('/haya_motor_node/Kd')
    pid_params.target_velocity = rospy.get_param('/haya_motor_node/target_velocity')
    pub_pid_params = rospy.Publisher('/pid_params', PidMsgs, queue_size = 1)
    pub_z = rospy.Publisher('/haya_angular_velocity_z', Float32, queue_size = 1)
    rospy.Subscriber('/arduino_msg', ArduinoMsgs, arduino_info_cb)
    rospy.Subscriber('/imu_data', ImuData, imu_data_cb)
    rospy.loginfo("[python-node] haya_motor_node started up")
    rospy.spin()

if __name__ == '__main__':
    main()
