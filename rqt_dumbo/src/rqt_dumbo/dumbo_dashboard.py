#!/usr/bin/env python

#   dumbo_dashboard
#
#   Created on: Nov 24, 2015
#   Authors:   Francisco Vina
#             fevb <at> kth.se
#

#  Copyright (c) 2015, Francisco Vina, CVAP, KTH
#    All rights reserved.

#    Redistribution and use in source and binary forms, with or without
#    modification, are permitted provided that the following conditions are met:
#       * Redistributions of source code must retain the above copyright
#         notice, this list of conditions and the following disclaimer.
#       * Redistributions in binary form must reproduce the above copyright
#         notice, this list of conditions and the following disclaimer in the
#         documentation and/or other materials provided with the distribution.
#       * Neither the name of KTH nor the
#         names of its contributors may be used to endorse or promote products
#         derived from this software without specific prior written permission.

#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#    DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
#    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QGraphicsView, QMainWindow

from .dumbo_dashboard_widget import DumboDashboardWidget
from robotiq_s_model_control_client import robotiq_s_model_control_client.RobotiqSModelControlClient

from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Bool
from control_msgs.msg import GripperCommand
from controller_manager.controller_manager_interface import SwitchController, SwitchControllerRequest
import moveit_commander
import numpy as np


class DumboDashboardGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super(DumboDashboardGraphicsView, self).__init__()

class DumboDashboard(Plugin):

    def __init__(self, context):
        super(DumboDashboard, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Dumbo')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        #self._widget = DumboDashboardWidget(context, args.clock)
        self._widget = QWidget()
        # # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_dumbo'), 'resource', 'dumbo_dashboard_widget.ui')
        # # Extend the widget with all attributes and children from UI file
        # loadUi(ui_file, self._widget)
        loadUi(ui_file, self._widget, {'DumboDashboardGraphicsView': DumboDashboardGraphicsView})
        # Give QObjects reasonable names
        self._widget.setObjectName('Dumbo Dashboard')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.

        print('test...')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.register_callbacks()
        self.init_srvs()
        self._robotiq = RobotiqSModelControlClient()
        self.init_moveit()
        self.init_topics()


    def register_callbacks(self):

        # connect/disconnect/recover/stop buttons
        self._widget.soft_connect_button.clicked[bool].connect(self._handle_soft_connect_button_clicked)
        self._widget.connect_button.clicked[bool].connect(self._handle_connect_button_clicked)
        self._widget.disconnect_button.clicked[bool].connect(self._handle_disconnect_button_clicked)
        self._widget.recover_button.clicked[bool].connect(self._handle_recover_button_clicked)
        self._widget.stop_button.clicked[bool].connect(self._handle_stop_button_clicked)

        # joint controllers
        self._widget.traj_controller_on_button.clicked[bool].connect(self._handle_traj_controller_on_button_clicked)
        self._widget.traj_controller_off_button.clicked[bool].connect(self._handle_traj_controller_off_button_clicked)

        self._widget.vel_controller_on_button.clicked[bool].connect(self._handle_vel_controller_on_button_clicked)
        self._widget.vel_controller_off_button.clicked[bool].connect(self._handle_vel_controller_off_button_clicked)


        # moveit
        self._widget.left_arm_home_pos_button.clicked[bool].connect(self._handle_left_arm_home_pos_button_clicked)
        self._widget.left_arm_default_pos_button.clicked[bool].connect(self._handle_left_arm_default_pos_button_clicked)
        self._widget.left_arm_pointing_forward_pos_button.clicked[bool].connect(self._handle_left_arm_pointing_forward_pos_button_clicked)

        self._widget.right_arm_home_pos_button.clicked[bool].connect(self._handle_right_arm_home_pos_button_clicked)
        self._widget.right_arm_default_pos_button.clicked[bool].connect(self._handle_right_arm_default_pos_button_clicked)
        self._widget.right_arm_pointing_forward_pos_button.clicked[bool].connect(self._handle_right_arm_pointing_forward_pos_button_clicked)

        
        # parallel gripper
        self._widget.parallel_gripper_open_button.clicked[bool].connect(self._handle_parallel_gripper_open_button_clicked)
        self._widget.parallel_gripper_send_pos_button.clicked[bool].connect(self._handle_parallel_gripper_send_pos_button_clicked)

        # robotiq
        self._widget.robotiq_activate_button.clicked[bool].connect(self._handle_robotiq_activate_button_clicked)
        self._widget.robotiq_reset_button.clicked[bool].connect(self._handle_robotiq_reset_button_clicked)
        self._widget.robotiq_open_button.clicked[bool].connect(self._handle_robotiq_open_button_clicked)
        self._widget.robotiq_close_button.clicked[bool].connect(self._handle_robotiq_close_button_clicked)

    
    def init_srvs(self):
        # services for connecting/disconnecting/recovering/stopping dumbo
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('/dumbo/soft_connect', 1.0)
                rospy.wait_for_service('/dumbo/connect', 1.0)
                rospy.wait_for_service('/dumbo/disconnect', 1.0)
                rospy.wait_for_service('/dumbo/recover', 1.0)
                rospy.wait_for_service('/dumbo/stop', 1.0)
                break
            except:
                rospy.logwarn('[dumbo dashboard]: waiting for dumbo connect/disconnect/recover/stop services')
                continue

        self._dumbo_soft_connect_srv = rospy.ServiceProxy('/dumbo/soft_connect', Empty)
        self._dumbo_connect_srv = rospy.ServiceProxy('/dumbo/connect', Empty)
        self._dumbo_disconnect_srv = rospy.ServiceProxy('/dumbo/disconnect', Empty)
        self._dumbo_recover_srv = rospy.ServiceProxy('/dumbo/recover', Empty)
        self._dumbo_stop_srv = rospy.ServiceProxy('/dumbo/stop', Empty)

        # service for switching joint controllers
        switch_controller_srv_name = '/controller_manager/switch_controller'
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(switch_controller_srv_name, 1.0)
                break
            except:
                rospy.logwarn('[dumbo dashboard]: waiting for switch_controller srv of controller manager')
                continue

        self._switch_controller_srv = rospy.ServiceProxy(switch_controller_srv_name, SwitchController)            


    def init_topics(self):
        # initialize subscribers for hardware status
        self._left_arm_status_sub = rospy.Subscriber('/left_arm/connected', Bool, self._left_arm_status_cb)
        self._right_arm_status_sub = rospy.Subscriber('/right_arm/connected', Bool, self._right_arm_status_cb)
        self._parallel_gripper_status_sub = rospy.Subscriber('/PG70_gripper/connected', Bool, self._parallel_gripper_status_cb)

        self._left_arm_ft_sensor_status_sub = rospy.Subscriber('/left_arm_ft_sensor/connected', Bool, self._left_arm_ft_sensor_status_cb)
        self._right_arm_ft_sensor_status_sub = rospy.Subscriber('/right_arm_ft_sensor/connected', Bool, self._right_arm_ft_sensor_status_cb)

        self._parallel_gripper_pos_command_pub = rospy.Publisher('/PG70_gripper/pos_command', GripperCommand)

        
    def init_moveit(self):
        self._left_arm = moveit_commander.MoveGroupCommander('left_arm')
        self._right_arm = moveit_commander.MoveGroupCommander('right_arm')

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


    # callbacks for buttons
    def _handle_soft_connect_button_clicked(self):
        self._dumbo_soft_connect_srv.call(EmptyRequest())

    def _handle_connect_button_clicked(self):
        self._dumbo_connect_srv.call(EmptyRequest())

    def _handle_disconnect_button_clicked(self):
        self._dumbo_disconnect_srv.call(EmptyRequest())

    def _handle_recover_button_clicked(self):
        self._dumbo_recover_srv.call(EmptyRequest())

    def _handle_stop_button_clicked(self):
        self._dumbo_stop_srv.call(EmptyRequest())

    def _handle_traj_controller_on_button_clicked(self):
        req = SwitchControllerRequest()
        req.stop_controllers = ['left_arm_joint_velocity_controller', 'right_arm_joint_velocity_controller']
        req.start_controllers = ['left_arm_joint_trajectory_controller', 'right_arm_joint_trajectory_controller']
        req.strictness = 1

        try:
            self._switch_controller_srv.call(req)

        except rospy.ServiceException, e:
            rospy.logerr('[dumbo dashboard]: error starting joint trajectory controllers')

    def _handle_traj_controller_off_button_clicked(self):
        req = SwitchControllerRequest()
        req.stop_controllers = ['left_arm_joint_trajectory_controller', 'right_arm_joint_trajectory_controller']
        req.strictness = 1

        try:
            self._switch_controller_srv.call(req)

        except rospy.ServiceException, e:
            rospy.logerr('[dumbo dashboard]: error stopping joint trajectory controllers')


    def _handle_vel_controller_on_button_clicked(self):
        req = SwitchControllerRequest()
        req.start_controllers = ['left_arm_joint_velocity_controller', 'right_arm_joint_velocity_controller']
        req.stop_controllers = ['left_arm_joint_trajectory_controller', 'right_arm_joint_trajectory_controller']
        req.strictness = 1

        try:
            self._switch_controller_srv.call(req)

        except rospy.ServiceException, e:
            rospy.logerr('[dumbo dashboard]: error starting joint velocity controllers')

    def _handle_vel_controller_off_button_clicked(self):
        req = SwitchControllerRequest()
        req.stop_controllers = ['left_arm_joint_velocity_controller', 'right_arm_joint_velocity_controller']
        req.strictness = 1

        try:
            self._switch_controller_srv.call(req)

        except rospy.ServiceException, e:
            rospy.logerr('[dumbo dashboard]: error stopping joint velocity controllers')


    def _handle_left_arm_home_pos_button_clicked(self):
        q = [0] * 7
        self._left_arm.set_joint_value_target(q)
        self._left_arm.go(q, wait=True)

    def _handle_left_arm_default_pos_button_clicked(self):
        q = [0] * 7
        q[1] = -0.9256
        q[3] = -0.9487
        q[5] = -0.5177
        self._left_arm.set_joint_value_target(q)
        self._left_arm.go(q, wait=True)

    def _handle_left_arm_pointing_forward_pos_button_clicked(self):
        q = [0] * 7
        q[1] = -np.pi*0.5
        self._left_arm.set_joint_value_target(q)
        self._left_arm.go(q, wait=True)


    def _handle_right_arm_home_pos_button_clicked(self):
        q = [0] * 7
        self._right_arm.set_joint_value_target(q)
        self._right_arm.go(q, wait=True)

    def _handle_right_arm_default_pos_button_clicked(self):
        q = [0] * 7
        q[1] = 0.8099
        q[3] = 0.8562
        q[5] = 0.6903
        self._right_arm.set_joint_value_target(q)
        self._right_arm.go(q, wait=True)

    def _handle_right_arm_pointing_forward_pos_button_clicked(self):
        q = [0] * 7
        q[1] = np.pi*0.5
        self._right_arm.set_joint_value_target(q)
        self._right_arm.go(q, wait=True)

    def _handle_parallel_gripper_open_button_clicked(self):
        cmd = GripperCommand()
        cmd.position = 59.0/1000.0
        self._parallel_gripper_pos_command_pub.publish(cmd)


    def _handle_parallel_gripper_send_pos_button_clicked(self):
        cmd = GripperCommand
        cmd.position = self._widget._parallel_gripper_pos.text().toDouble()/1000.0
        self._parallel_gripper_pos_command_pub.publish(cmd)


    def _handle_robotiq_activate_button_clicked(self):
        self._robotiq.activate()

    def _handle_robotiq_reset_button_clicked(self):
        self._robotiq.reset()

    def _handle_robotiq_open_button_clicked(self):
        self._robotiq.open()

    def _handle_robotiq_close_button_clicked(self):
        self._robotiq.close()


    def _left_arm_status_cb(self, msg):
        if msg.data:
            self._widget.label_LA.setText('Left arm connected')

        else:
            self._widget.label_LA.setText('Left arm disconnected')

    def _right_arm_status_cb(self, msg):
        if msg.data:
            self._widget.label_RA.setText('Right arm connected')

        else:
            self._widget.label_RA.setText('Right arm disconnected')


    def _parallel_gripper_status_cb(self, msg):
        if msg.data:
            self._widget.label_gripper.setText('Parallel gripper connected')

        else:
            self._widget.label_gripper.setText('Parallel gripper disconnected')

    def _left_arm_ft_sensor_status_cb(self, msg):
        if msg.data:
            self._widget.label_l_ft.setText('Left arm F/t sensor connected')

        else:
            self._widget.label_l_ft.setText('Left arm F/t sensor disconnected')


    def _right_arm_ft_sensor_status_cb(self, msg):
        if msg.data:
            self._widget.label_r_ft.setText('Right arm F/t sensor connected')

        else:
            self._widget.label_r_ft.setText('Right arm F/t sensor disconnected')





