#!/usr/bin/env python

#   dumbo_dashboard_widget
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
import time

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QFileDialog, QGraphicsView, QIcon, QWidget




class DumboDashboardGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super(DumboDashboardGraphicsView, self).__init__()


class DumboDashboardWidget(QWidget):
    """
    Widget for use with DumboDashboard class to display and replay bag files
    Handles all widget callbacks and contains the instance of BagTimeline for storing visualizing bag data
    """

    set_status_text = Signal(str)

    def __init__(self, context, publish_clock):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(DumboDashboardWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_dumbo'), 'resource', 'dumbo_dashboard_widget.ui')
        loadUi(ui_file, self, {'DumboDashboardGraphicsView': DumboDashboardGraphicsView})

        self.setObjectName('DumboDashboardWidget')


        self.graphics_view.resizeEvent = self._resizeEvent
        self.graphics_view.setMouseTracking(True)