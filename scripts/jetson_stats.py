#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# Copyright (C) 2020, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ros_jetson_stats.srv import nvpmodel, nvpmodelResponse, jetson_clocks, jetson_clocksResponse, fan, fanResponse
from datetime import timedelta
import jtop
# Import Diagnostic status converters
from ros_jetson_stats.utils import (
    other_status,
    board_status,
    disk_status,
    engine_status,
    cpu_status,
    fan_status,
    gpu_status,
    ram_status,
    swap_status,
    power_status,
    temp_status,
    emc_status)


class ROSJtop:

    def __init__(self, level_options):
        self.level_options = level_options
        interval = rospy.get_param("~interval", 0.5)
        # Initialization jtop
        self.jetson = jtop.jtop(interval=interval)
        # Define Diagnostic array message
        # http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html
        self.arr = DiagnosticArray()
        # Initialization ros publisher
        self.pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        # Initialize services server
        rospy.Service('/jtop/nvpmodel', nvpmodel, self.nvpmodel_service)
        rospy.Service('/jtop/jetson_clocks', jetson_clocks, self.jetson_clocks_service)
        rospy.Service('/jtop/fan', fan, self.fan_service)

    def start(self):
        self.jetson.start()
        if self.jetson.interval != self.jetson.interval_user:
            rospy.logwarn("I cannot set {user:.2f}s. Service already running at {service:.2f}s".format(
                user=self.jetson.interval_user,
                service=self.jetson.interval))
        # Extract board information
        board = self.jetson.board
        # Define hardware name
        self.hardware = board["platform"]["Machine"]
        # Board status message
        self.board_status = board_status(self.hardware, board, 'board')
        # Set callback
        rospy.loginfo("jtop wrapper started")
        self.jetson.attach(self.jetson_callback)

    def stop(self):
        rospy.loginfo("Close jtop")
        self.jetson.close()

    def fan_service(self, req):
        # Update fan_service to use fan.profile instead of fan.mode
        fan_profile = req.mode
        fan_speed = req.fanSpeed
        try:
            self.jetson.fan.profile = fan_profile
            if fan_profile == "manual":
                self.jetson.fan.speed = fan_speed
        except jtop.JtopException as e:
            rospy.logerr(e)
            # Return current fan profile
            fan_profile = str(self.jetson.fan.profile)
        return fanResponse(fan_profile)

    def jetson_clocks_service(self, req):
        # Set new jetson_clocks
        self.jetson.jetson_clocks = req.status
        return jetson_clocksResponse(req.status)

    def nvpmodel_service(self, req):
        # Try to set new nvpmodel
        nvpmodel = req.nvpmodel
        try:
            self.jetson.nvpmodel = nvpmodel
        except jtop.JtopException as e:
            rospy.logerr(e)
            # Return same nvp model
            nvpmodel = str(self.jetson.nvpmodel)
        return nvpmodelResponse(nvpmodel)

    def jetson_callback(self, jetson):
        # Add timestamp
        self.arr.header.stamp = rospy.Time.now()
        # Status board and board info
        self.arr.status = [other_status(self.hardware, jetson, jtop.__version__)]
        # Make diagnostic message for each cpu
        self.arr.status += [cpu_status(self.hardware, name, cpu)
                            for name, cpu in enumerate(jetson.cpu['cpu'])]
        # Merge all other diagnostics
        self.arr.status += [gpu_status(self.hardware, name, jetson.gpu[name])
                            for name in self.jetson.gpu]
        # Make diagnostic message for each engine
        self.arr.status += [engine_status(self.hardware, name, engine)
                            for name, engine in jetson.engine.items()]
        # Update access to jetson.memory
        self.arr.status += [ram_status(self.hardware, jetson.memory['RAM'], 'mem')]
        self.arr.status += [swap_status(self.hardware, jetson.memory['SWAP'], 'mem')]
        if 'EMC' in jetson.memory:
            self.arr.status += [emc_status(self.hardware, jetson.memory['EMC'], 'mem')]
        # Make diagnostic message for each Temperature
        self.arr.status += [temp_status(self.hardware, name, sensor)
                            for name, sensor in jetson.temperature.items()]
        # Make diagnostic message for each power rail
        self.arr.status += [power_status(self.hardware, name, rail)
                            for name, rail in self.jetson.power['rail'].items()]
        if 'name' in self.jetson.power['tot']:
            name_total = self.jetson.power['tot']['name']
        else:
            name_total = 'ALL'
        self.arr.status += [power_status(self.hardware,
                                         name_total, jetson.power['tot'])]
        # Update fan_status usage
        if jetson.fan:
            self.arr.status += [fan_status(self.hardware, name, fan)
                                for name, fan in jetson.fan.items()]
        # Status board and board info
        self.arr.status += [self.board_status]
        # Add disk status
        self.arr.status += [disk_status(self.hardware, jetson.disk, 'board')]
        # Update status jtop
        rospy.logdebug("jtop message %s" % rospy.get_time())
        self.pub.publish(self.arr)


def wrapper():
    # Initialization ros node
    rospy.init_node('jtop_node')
    # Load level options
    level_options = {
        rospy.get_param("~level/error", 60): DiagnosticStatus.ERROR,
        rospy.get_param("~level/warning", 40): DiagnosticStatus.WARN,
        rospy.get_param("~level/ok", 20): DiagnosticStatus.OK,
    }
    # Initialization ROS jtop wrapper
    jetson = ROSJtop(level_options)
    rospy.loginfo(level_options)
    # Start jetson
    jetson.start()
    # Spin ros
    rospy.spin()
    # Close jtop
    jetson.stop()


if __name__ == '__main__':
    wrapper()
# EOF
