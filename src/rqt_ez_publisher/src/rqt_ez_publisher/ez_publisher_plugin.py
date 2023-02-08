import os
import rospy
import yaml
from . import ez_publisher_widget
from . import publisher
from . import config_dialog
from . import quaternion_module
from rqt_py_common.plugin_container_widget import PluginContainerWidget
from qt_gui.plugin import Plugin


class EzPublisherPlugin(Plugin):

    '''Plugin top class for rqt_ez_publisher'''

    def __init__(self, context):
        super(EzPublisherPlugin, self).__init__(context)
        self.setObjectName('EzPublisher')
        modules = [quaternion_module.QuaternionModule()]
        self._widget = ez_publisher_widget.EzPublisherWidget(modules=modules)
        self._widget.setObjectName('EzPublisherPluginUi')
        self.mainwidget = PluginContainerWidget(self._widget, True, False)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        from argparse import ArgumentParser
        parser = ArgumentParser(prog='rqt_ez_publisher')
        EzPublisherPlugin.add_arguments(parser)
        args, unknowns = parser.parse_known_args(context.argv())
        self._loaded_settings = None
        self.configurable = True
        if args.slider_file is not None:
            self.load_from_file(args.slider_file)

    def shutdown_plugin(self):
        pass

    def save_to_file(self, file_path):
        pass
    def load_from_file(self, file_path):
        pass
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass
    def restore_from_dict(self, settings):
        pass
    def save_to_dict(self):
        pass
    def trigger_configuration(self):
        dialog = config_dialog.ConfigDialog(self)
        dialog.exec_()
        self.configurable = dialog.configurable_checkbox.isChecked()
        self._widget.set_configurable(self.configurable)

    @staticmethod
    def _isfile(parser, arg):
        if os.path.isfile(arg):
            return arg
        else:
            parser.error("Setting file %s does not exist" % arg)

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group(
            'Options for rqt_ez_publisher plugin')
        group.add_argument('--slider-file',
                           type=lambda x: EzPublisherPlugin._isfile(parser, x),
                           help="YAML setting file")
