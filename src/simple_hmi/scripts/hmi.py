#!/usr/bin/env python3
import rospy
from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import Int32, String, Bool
from std_msgs.msg import Empty as EmptyM
from sim_msgs.msg import VHA
from sim_msgs.srv import Int, IntResponse
from copy import deepcopy
from std_srvs.srv import EmptyResponse
from std_srvs.srv import Empty as EmptyS
from std_srvs.srv import SetBool, SetBoolResponse

class MyQtWidget(QWidget):

    '''Main widget of this GUI'''

    sig_sysmsg = QtCore.Signal(str)
    sig_timeout_val = QtCore.Signal(int)
    sig_timeout_max = QtCore.Signal(int)
    sig_enable_buttons = QtCore.Signal(bool)
    sig_finish = QtCore.Signal()
    sig_clear_ha = QtCore.Signal()
    sig_add_ha = QtCore.Signal(str)
    sig_info_label = QtCore.Signal(str)
    sig_wait_label = QtCore.Signal(str)

    def __init__(self, parent=None, modules=[]):
        QWidget.__init__(self, parent=parent)
        self._publishers = {}
        # self._publishers["human_choice"] = rospy.Publisher("human_choice", Int32, queue_size=1)
        self._publishers["human_decision"] = rospy.Publisher("human_decision", Int32, queue_size=1)
        self._publishers["step_over"] = rospy.Publisher("step_over", EmptyM, queue_size=1)

        self._subscribers = []
        self._subscribers.append( rospy.Subscriber("hmi_vha", VHA, self.incoming_vha_cb, queue_size=1) )
        self._subscribers.append( rospy.Subscriber("hmi_timeout_value", Int32, self.timeout_value_cb, queue_size=1) )
        self._subscribers.append( rospy.Subscriber("hmi_timeout_reached", EmptyM, self.timeout_reached_cb, queue_size=1) )
        self._subscribers.append( rospy.Subscriber("hmi_enable_buttons", Bool, self.enable_buttons_cb, queue_size=1) )
        self._subscribers.append( rospy.Subscriber("hmi_finish", EmptyM, self.finish_cb, queue_size=1) )

        self._started_service = rospy.Service("hmi_started", EmptyS, lambda req: EmptyResponse())
        self._timeout_max_service = rospy.Service("hmi_timeout_max", Int, self.set_timeout_max)
        self._r_idle_service = rospy.Service("hmi_r_idle", SetBool, self.set_r_idle)
        self.r_idle = False

        self._start_human_action_prox = rospy.ServiceProxy("start_human_action", Int)
        
        self.sig_timeout_val.connect(self.timeout_value)
        self.sig_timeout_max.connect(self.timeout_max)
        self.sig_enable_buttons.connect(self.enable_buttons)
        self.sig_finish.connect(self.finish)
        self.sig_clear_ha.connect(self.clear_human_actions)
        self.sig_add_ha.connect(self.add_ha_button)
        self.sig_info_label.connect(self.update_info_label)
        self.sig_wait_label.connect(self.update_wait_label)

        self.max_timeout = -1
        self.nb_human_action = 0
        self.first = True
        self.vha = None
        self.setup_ui()

    def shutdown(self):
        self._model.shutdown()

    def setup_ui(self):
        self._v_layout_main = QtWidgets.QVBoxLayout()
        self._v_layout_main.setAlignment(QtCore.Qt.AlignCenter)
        
        # Label
        self._label_ha = QtWidgets.QLabel('Possible Human Actions:')
        self._label_ha.setAlignment(QtCore.Qt.AlignCenter)
        self._v_layout_main.addWidget(self._label_ha)
        
        # Human Action Buttons #
        self._h_layout_HA = QtWidgets.QHBoxLayout()
        self._h_layout_HA.setAlignment(QtCore.Qt.AlignCenter)
        self._buttons_human_actions = []
        self._v_layout_main.addLayout(self._h_layout_HA)

        # PASS button #
        self._h_layout_pass_button = QtWidgets.QHBoxLayout()
        self._h_layout_pass_button.setAlignment(QtCore.Qt.AlignCenter)
        self._button_pass = QtWidgets.QPushButton('PASS')
        self._button_pass.setFixedSize(100, 70)
        self._button_pass.clicked.connect(lambda : self.pressed_publish_human_choice(-1))
        self._h_layout_pass_button.addWidget(self._button_pass)
        self._v_layout_main.addLayout(self._h_layout_pass_button)
        # self._v_layout_main.addWidget(self._button_pass)

        # Manage buttons 
        self._h_layout_manage_buttons = QtWidgets.QHBoxLayout()
        # Add button #
        self._button_add_human_action = QtWidgets.QPushButton("Add")
        self._button_add_human_action.clicked.connect(self.add_human_action_button)
        self._h_layout_manage_buttons.addWidget(self._button_add_human_action)
        # Clear button #
        self._button_clear_human_action = QtWidgets.QPushButton("Clear")
        self._button_clear_human_action.clicked.connect(self.clear_human_actions)
        self._h_layout_manage_buttons.addWidget(self._button_clear_human_action)
        # Step Over button #
        self._button_step_over = QtWidgets.QPushButton("Step Over")
        self._button_step_over.clicked.connect(lambda: self._publishers["step_over"].publish(EmptyM()))
        self._h_layout_manage_buttons.addWidget(self._button_step_over)
        # Add to main layout
        self._v_layout_main.addLayout(self._h_layout_manage_buttons)
        # Progress bar
        self._progress_bar = QtWidgets.QProgressBar()
        self._progress_bar.setFormat("%v/%m s")
        self._v_layout_main.addWidget(self._progress_bar)
        # Step minus button #
        self._button_minus = QtWidgets.QPushButton("-")
        self._button_minus.clicked.connect(lambda: self.sig_timeout_val.emit(self._progress_bar.value()-1))
        self._h_layout_manage_buttons.addWidget(self._button_minus)
        # Step plus button #
        self._button_plus = QtWidgets.QPushButton("+")
        self._button_plus.clicked.connect(lambda: self.sig_timeout_val.emit(self._progress_bar.value()+1))
        self._h_layout_manage_buttons.addWidget(self._button_plus)

        # Info Label #
        self._info_label = QtWidgets.QLabel("")
        self._v_layout_main.addWidget(self._info_label)
        # Waiting Label #
        self._wait_label = QtWidgets.QLabel("Waiting simulation to start...")
        self._v_layout_main.addWidget(self._wait_label)

        self.setLayout(self._v_layout_main)

    #########


    # hmi_timeout_value CB + Sig #
    def timeout_value_cb(self, msg):
        self.sig_timeout_val.emit(msg.data)
    @QtCore.pyqtSlot(int)
    def timeout_value(self, val):
        self._progress_bar.setValue(val)

    # hmi_timeout_reached CB + Sig #
    def timeout_reached_cb(self, msg):
        self.sig_timeout_max.emit(0)
        self.sig_info_label.emit("Timeout reached, Robot starts acting. Human can still perform a compliant action..")
        self.sig_wait_label.emit("Waiting for both agents' actions to be over...")
        self._button_pass.setEnabled(False)

    @QtCore.pyqtSlot(int)
    def timeout_max(self, val):
        self._progress_bar.setMaximum(val)

    # hmi_enable_buttons CB + Sig #
    def enable_buttons_cb(self, msg):
        self.sig_enable_buttons.emit(msg.data)
    @QtCore.pyqtSlot(bool)
    def enable_buttons(self, val):
        self._button_pass.setEnabled(val)
        for b in self._buttons_human_actions:
            b.setEnabled(val)

    # hmi_finish CB + Sig #
    def finish_cb(self, msg):
        self.sig_finish.emit()
        self.sig_info_label.emit("")
        self.sig_wait_label.emit("Simulation Over.")
    @QtCore.pyqtSlot()
    def finish(self):
        self.clear_human_actions()
        self._progress_bar.setMaximum(self.max_timeout)
        self._progress_bar.reset()

    # hmi_r_idle Srv #
    def set_r_idle(self, req):
        self.r_idle = req.data
        return SetBoolResponse()

    # hmi_timeout_max Srv #
    def set_timeout_max(self, req):
        self.max_timeout = req.data
        return IntResponse()

    # hmi_vha CB #
    def incoming_vha_cb(self, msg):
        self.sig_clear_ha.emit()
        
        if msg.type == msg.START:
            self.sig_timeout_max.emit(self.max_timeout)
            self.sig_enable_buttons.emit(True)
            self.sig_info_label.emit("")
            self.sig_wait_label.emit("Waiting for human choice...")

        self.vha = msg
        for ha in msg.valid_human_actions:
            self.sig_add_ha.emit(ha)
        while self.nb_human_action<len(msg.valid_human_actions):
            rospy.sleep(0.05)

    # Add human action button Sig #
    @QtCore.pyqtSlot(str)
    def add_ha_button(self, str):
        button = QtWidgets.QPushButton(str)
        id_action = deepcopy(self.nb_human_action)+1
        f = lambda : self.pressed_publish_human_choice(id_action)
        button.clicked.connect(f)
        self._buttons_human_actions.append( button )
        self._h_layout_HA.addWidget(button)
        self.nb_human_action += 1

    # _button_add_human_action.clicked.connect
    def add_human_action_button(self):
        text = "HA"+str(self.nb_human_action+1)
        self.sig_add_ha.emit(text)

    # _button_clear_human_action.clicked.connect
    @QtCore.pyqtSlot()
    def clear_human_actions(self):
        self._buttons_human_actions.clear()
        for i in reversed(range(self._h_layout_HA.count())): 
            self._h_layout_HA.itemAt(i).widget().setParent(None)
        self.nb_human_action=0

    # button_human_action.clicked.connect
    @QtCore.pyqtSlot(int)
    def pressed_publish_human_choice(self, choice):
        # msg = Int32()
        # msg.data = choice
        # self._publishers["human_choice"].publish(msg)
        self._start_human_action_prox(choice)

        self.sig_timeout_max.emit(0)

        if choice!=-1:
            print(f"ACT {self.vha.valid_human_actions[choice-1]}")
            self.sig_enable_buttons.emit(False)
            self.sig_info_label.emit("Human is active.")
        else:
            print("PASS")
            if self.r_idle:
                self._button_pass.setEnabled(False)
            else:
                self.sig_enable_buttons.emit(False)
            self.sig_info_label.emit("Human is passive, Robot starts acting. Human can still perform a compliant action.")
        self.sig_wait_label.emit("Waiting for both agents' actions to be over...")

    # Update info label Sig #
    @QtCore.pyqtSlot(str)
    def update_info_label(self, text):
        self._info_label.setText(text)

    # Update wait label Sig #
    @QtCore.pyqtSlot(str)
    def update_wait_label(self, text):
        self._wait_label.setText(text)

def main():
    import sys
    app = QtWidgets.QApplication(sys.argv)
    main_window = QtWidgets.QMainWindow()
    main_widget = MyQtWidget()
    main_window.setCentralWidget(main_widget)
    main_window.show()
    app.exec_()

if __name__ == '__main__':
    rospy.init_node('hmi_qt')
    main()
