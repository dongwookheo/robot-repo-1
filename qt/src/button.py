import sys
import rclpy
from std_msgs.msg import String
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic

from_class = uic.loadUiType("/home/ane21/pinkbot/src/pinklab_minibot_robot/qt/src/button.ui")[0]

class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("path")

        self.a_button.clicked.connect(self.Abutton_Clicked)
        self.b_button.clicked.connect(self.Bbutton_Clicked)
        self.i_button.clicked.connect(self.Ibutton_Clicked)
        self.d_button.clicked.connect(self.Dbutton_Clicked)

        # ROS2 노드 초기화
        rclpy.init()
        self.node = rclpy.create_node('button_publisher')

        # 토픽 publisher 생성
        self.publisher = self.node.create_publisher(String, 'table_topic', 10)

    def Abutton_Clicked(self):
        self.textEdit.setText("moving to a table")
        print("moving to a table")

        # 'a table' 메시지를 ROS2 토픽으로 전송
        msg = String()
        msg.data = 'a table'
        self.publisher.publish(msg)

    def Bbutton_Clicked(self):
        self.textEdit.setText("moving to b table")
        print("moving to b table")

        # 'b table' 메시지를 ROS2 토픽으로 전송
        msg = String()
        msg.data = 'b table'
        self.publisher.publish(msg)

    def Ibutton_Clicked(self):
        self.textEdit.setText("moving to initial position")
        print("moving to initial position")

        # 'initial position' 메시지를 ROS2 토픽으로 전송
        msg = String()
        msg.data = 'initial position'
        self.publisher.publish(msg)

    def Dbutton_Clicked(self):
        self.textEdit.setText("docking")
        print("docking")

        # 'docking' 메시지를 ROS2 토픽으로 전송
        msg = String()
        msg.data = 'docking'
        self.publisher.publish(msg)

    def closeEvent(self, event):
        # 애플리케이션 종료 시 ROS2 노드 정리
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())
