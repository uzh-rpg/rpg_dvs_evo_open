#!/usr/bin/env python
import os
import rospy
import argparse
from qt_gui.plugin import Plugin
from .evo_widget import EvoWidget


class Evo(Plugin):
    """
    Subclass of Plugin to display EVO status
    """

    def __init__(self, context):

        # Init Plugin
        super(Evo, self).__init__(context)
        self.setObjectName('EvoPlugin')

        # Load arguments
        # TODO load topic name from args
        args = self._parse_args(context.argv())

        # Create QWidget
        self._widget = EvoWidget(evo_namespace=args.topic)

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('--topic', help='Evo Info Topic to display')
        args, unknown = parser.parse_known_args()
        return args
