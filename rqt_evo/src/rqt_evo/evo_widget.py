#!/usr/bin/env python
import os
import rospy
import rospkg
import numpy as np
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QTimer, Slot
#from evo_msgs.msg import Info
from std_msgs.msg import String, Bool
import subprocess


class EvoWidget(QWidget):
    """
    EVO rqt widget actions
    """
    #_last_info_msg = Info()
    _publisher = None
    _subscriber = None
    _publisher_copilot = None
    _num_received_msgs = 0
    _evo_namespace = None

    def __init__(self, evo_namespace='evo'):

        # Init QWidget
        super(EvoWidget, self).__init__()
        self.setObjectName('EvoWidget')

        # load UI
        ui_file = os.path.join(rospkg.RosPack().get_path(
            'rqt_evo'), 'resource', 'widget.ui')
        loadUi(ui_file, self)

        # init and start update timer for data, the timer calls the function update_info all 40ms
        #self._update_info_timer = QTimer(self)
        # self._update_info_timer.timeout.connect(self.update_info)
        # self._update_info_timer.start(40)

        # set the functions that are called when a button is pressed
        self.button_bootstrap.pressed.connect(self.on_bootstrap_button_pressed)
        self.button_start.pressed.connect(self.on_start_button_pressed)
        self.button_update.pressed.connect(self.on_update_button_pressed)
        self.button_switch.pressed.connect(self.on_switch_button_pressed)
        self.checkbox_map_expansion.stateChanged.connect(
            self.on_map_expansion_changed)
        self.checkbox_copilot.stateChanged.connect(
            self.on_copilot_state_changed)

        # set callback for changed topic

        if not evo_namespace:
            evo_namespace = 'evo'

        self._evo_namespace = evo_namespace
        self.topic_line_edit.textChanged.connect(self._on_topic_changed)
        self.register(evo_namespace)
        self.topic_line_edit.setText(evo_namespace)

        # TODO: set a timer when the last message was received and give a warning if it is too long ago!

    @Slot(str)
    def _on_topic_changed(self, topic):
        """
        Switches publishers namespaces
        """
        self._evo_namespace = str(topic)
        self.unregister()
        self.register(self._evo_namespace)

    def register(self, evo_namespace):
        """
        Subscribes to ROS Info topic and registers callbacks
        """
        # self._subscriber = rospy.Subscriber(evo_namespace+'/info', Info, self.info_cb)

        # Initialize Publisher
        self._publisher = rospy.Publisher(
            evo_namespace+'/remote_key', String, queue_size=1)
        self._publisher_copilot = rospy.Publisher(
            evo_namespace+'/copilot_remote', Bool, queue_size=1)

    def unregister(self):
        """
        Unregisters publishers
        """
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

        if self._publisher_copilot is not None:
            self._publisher_copilot.unregister()
            self._publisher_copilot = None

        if self._subscriber is not None:
            self._subscriber.unregister()
            self._subscriber = None

    # def info_cb(self, msg):
    #  self._last_info_msg = msg
    #  self._num_received_msgs += 1

    # def update_info(self):
    #  info_text = 'Not Connected'
    #  if self._num_received_msgs > 0:
    #    pass

        # set info text
        # self.evo_info_label.setText(info_text)

        # set progress bar
        # self.num_tracked_bar.setValue(self._last_info_msg.num_matches)

    def on_bootstrap_button_pressed(self):
        """
        Triggers bootstrap
        """
        print('BOOTSTRAPPING')
        self.send_command('bootstrap')

    def on_start_button_pressed(self):
        """
        Triggers reset button
        """
        print('START/RESET')
        self.send_command('reset')

    def on_update_button_pressed(self):
        """
        Triggers map update
        """
        print('UPDATE')
        self.send_command('update')

    def on_switch_button_pressed(self):
        """
        Turns on tracking thread
        """
        print('SWITCH TO TRACKING')
        self.send_command('switch')

    def on_map_expansion_changed(self):
        """
        Switches on and off the map expansion algorithm based on checkbox_map_expansion 
        """
        if self.checkbox_map_expansion.isChecked():
            print('ENABLE MAP EXPANSION')
            self.send_command('enable_map_expansion')
        else:
            print('DISABLE EXPANSION')
            self.send_command('disable_map_expansion')

    def on_copilot_state_changed(self):
        """
        Switch from bootstrapping tracker to evo tracker
        """
        useEVO = self.checkbox_copilot.isChecked()
        print('SWITCH COPILOT TO ' + ('EVO' if useEVO else 'INITIAL PILOT'))
        self._publisher_copilot.publish(Bool(useEVO))

    def send_command(self, cmd):
        """
        Utils to send remote command
        """
        if self._publisher is None:
            return
        self._publisher.publish(String(cmd))
