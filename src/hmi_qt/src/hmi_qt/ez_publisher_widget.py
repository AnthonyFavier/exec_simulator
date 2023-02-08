import rospy
from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets
from python_qt_binding.QtWidgets import QWidget
from hmi_qt import ez_publisher_model as ez_model
from hmi_qt import widget as ez_widget
from hmi_qt import publisher
from std_msgs.msg import Int32, String
from sim_msgs.msg import VHA
from copy import deepcopy


class EzPublisherWidget(QWidget):

    '''Main widget of this GUI'''

    sig_sysmsg = QtCore.Signal(str)

    def __init__(self, parent=None, modules=[]):
        QWidget.__init__(self, parent=parent)
        self._publishers = {}
        self._publishers["human_choice"] = rospy.Publisher("human_choice", Int32, queue_size=1)
        self._subscribers = []
        self._subscribers.append( rospy.Subscriber("hmi_vha", VHA, self.incoming_vha_cb, queue_size=1) )
        self.nb_human_action = 0
        self.setup_ui()

    def set_configurable(self, value):
        self._topic_label.setVisible(value)
        for slider in self._sliders:
            slider.set_configurable(value)

    def incoming_vha_cb(self, msg):
        for ha in msg.valid_human_actions:
            print(ha)

    def publish_with_button(self):
        msg = Int32()
        msg.data = 3
        self._publishers["testing_publish"].publish(msg)

        self._human_actions.append( QtWidgets.QPushButton('HA') )
        self._sub_horizontal_layout.addWidget(self._human_actions[-1])

        if self.alternating:
            self._pub_button.setStyleSheet("background-color : yellow")
        else:
            self._pub_button.setStyleSheet("")
        self.alternating = not self.alternating
    
    def clear_human_actions(self):
        self._buttons_human_actions.clear()
        for i in reversed(range(self._h_layout_HA.count())): 
            self._h_layout_HA.itemAt(i).widget().setParent(None)
        self.nb_human_action=0

    def add_human_action_button(self):
        text = "HA"+str(self.nb_human_action+1)
        button = QtWidgets.QPushButton(text)
        button.setFixedSize(100, 70)
        msg = String()
        msg.data = text
        # f = lambda : self._publishers["human_choice"].publish(msg)
        id_action = deepcopy(self.nb_human_action)+1
        f = lambda : self.pressed_publish_human_choice(id_action)
        button.clicked.connect(f)
        self._buttons_human_actions.append( button )
        self._h_layout_HA.addWidget(button)
        self.nb_human_action += 1

    def pressed_add_ha_button(self):
        self.add_human_action_button()

    def pressed_clear_ha_buttons(self):
        self.clear_human_actions()

    def pressed_publish_human_choice(self, choice):
        msg = Int32()
        msg.data = choice
        self._publishers["human_choice"].publish(msg)

    def setup_ui(self):
        self._v_layout_main = QtWidgets.QVBoxLayout()
        self._v_layout_main.setAlignment(QtCore.Qt.AlignCenter)
        
        self._label_ha = QtWidgets.QLabel('Possible Human Actions:')
        self._label_ha.setAlignment(QtCore.Qt.AlignCenter)
        self._v_layout_main.addWidget(self._label_ha)
        
        self._h_layout_HA = QtWidgets.QHBoxLayout()
        self._h_layout_HA.setAlignment(QtCore.Qt.AlignCenter)

        self._buttons_human_actions = []
        
        self.add_human_action_button()

        self._v_layout_main.addLayout(self._h_layout_HA)

        self._button_skip_ha = QtWidgets.QPushButton('Skip')
        self._button_skip_ha.setFixedSize(100, 70)
        msg = Int32()
        msg.data = -1
        f = lambda : self._publishers["human_choice"].publish(msg)
        # self._button_skip_ha.clicked.connect(f)
        self._button_skip_ha.clicked.connect(lambda : self.pressed_publish_human_choice(-1))
        self._v_layout_main.addWidget(self._button_skip_ha)

        self._h_layout_manage_buttons = QtWidgets.QHBoxLayout()
        self._button_add_human_action = QtWidgets.QPushButton("Add")
        self._button_add_human_action.clicked.connect(self.pressed_add_ha_button)
        self._button_clear_human_action = QtWidgets.QPushButton("Clear")
        self._button_clear_human_action.clicked.connect(self.pressed_clear_ha_buttons)
        self._h_layout_manage_buttons.addWidget(self._button_add_human_action)
        self._h_layout_manage_buttons.addWidget(self._button_clear_human_action)
        self._v_layout_main.addLayout(self._h_layout_manage_buttons)

        self.setLayout(self._v_layout_main)
    
    def shutdown(self):
        self._model.shutdown()


def main():
    import sys
    app = QtWidgets.QApplication(sys.argv)
    main_window = QtWidgets.QMainWindow()
    main_widget = EzPublisherWidget()
    main_window.setCentralWidget(main_widget)
    main_window.show()
    app.exec_()

if __name__ == '__main__':
    rospy.init_node('ez_publisher')
    main()
