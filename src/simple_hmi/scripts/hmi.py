#!/usr/bin/env python3
import rospy
from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import Int32, String
from std_msgs.msg import Empty as EmptyM
from sim_msgs.msg import VHA
from copy import deepcopy
from std_srvs.srv import EmptyResponse
from std_srvs.srv import Empty as EmptyS


class EzPublisherWidget(QWidget):

    '''Main widget of this GUI'''

    sig_sysmsg = QtCore.Signal(str)

    def __init__(self, parent=None, modules=[]):
        QWidget.__init__(self, parent=parent)
        self._publishers = {}
        self._publishers["human_choice"] = rospy.Publisher("human_choice", Int32, queue_size=1)
        self._publishers["step_over"] = rospy.Publisher("step_over", EmptyM, queue_size=1)

        self._subscribers = []
        self._subscribers.append( rospy.Subscriber("hmi_vha", VHA, self.incoming_vha_cb, queue_size=1) )

        self._started_service = rospy.Service("hmi_started", EmptyS, lambda req: EmptyResponse())
        
        self.nb_human_action = 0
        self.first = True
        self.setup_ui()


    def incoming_vha_cb(self, msg):
        if msg.type == msg.START:
            self.clear_human_actions()
            for ha in msg.valid_human_actions:
                QtCore.QMetaObject.invokeMethod(self, 'add_human_action_button', QtCore.Qt.QueuedConnection)

        while self.nb_human_action<len(msg.valid_human_actions):
            rospy.sleep(0.05)
            
        for i, ha in enumerate(msg.valid_human_actions):
            button = self._buttons_human_actions[i]
            button.setText(ha)

    def clear_human_actions(self):
        self._buttons_human_actions.clear()
        for i in reversed(range(self._h_layout_HA.count())): 
            self._h_layout_HA.itemAt(i).widget().setParent(None)
        self.nb_human_action=0

    @QtCore.pyqtSlot()
    def add_human_action_button(self):
        text = "HA"+str(self.nb_human_action+1)
        button = QtWidgets.QPushButton(text)
        # button.setFixedSize(100, 70)
        # msg = String()
        # msg.data = text
        id_action = deepcopy(self.nb_human_action)+1
        f = lambda : self.pressed_publish_human_choice(id_action)
        button.clicked.connect(f)
        self._buttons_human_actions.append( button )
        self._h_layout_HA.addWidget(button)
        self.nb_human_action += 1

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
        
        # self.add_human_action_button()

        self._v_layout_main.addLayout(self._h_layout_HA)

        self._button_skip_ha = QtWidgets.QPushButton('Skip')
        self._button_skip_ha.setFixedSize(100, 70)
        # msg = Int32()
        # msg.data = -1
        self._button_skip_ha.clicked.connect(lambda : self.pressed_publish_human_choice(-1))
        self._v_layout_main.addWidget(self._button_skip_ha)

        self._h_layout_manage_buttons = QtWidgets.QHBoxLayout()
        self._button_add_human_action = QtWidgets.QPushButton("Add")
        self._button_add_human_action.clicked.connect(self.add_human_action_button)
        self._button_clear_human_action = QtWidgets.QPushButton("Clear")
        self._button_clear_human_action.clicked.connect(self.clear_human_actions)
        self._button_step_over = QtWidgets.QPushButton("Step Over")
        self._button_step_over.clicked.connect(lambda: self._publishers["step_over"].publish(EmptyM()))
        self._h_layout_manage_buttons.addWidget(self._button_add_human_action)
        self._h_layout_manage_buttons.addWidget(self._button_clear_human_action)
        self._h_layout_manage_buttons.addWidget(self._button_step_over)
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
    rospy.init_node('hmi_qt')
    main()
