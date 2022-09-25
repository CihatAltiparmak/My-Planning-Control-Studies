import numpy as np

import sys
import random
import matplotlib
matplotlib.use("Qt5Agg")

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow,QWidget, \
                            QApplication, QLabel, QHBoxLayout, QVBoxLayout, QSizePolicy
from PyQt5.QtCore import QThread, Qt, QTimer, pyqtSignal, pyqtSlot, QObject, QCoreApplication

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Int64
from jarcar_msgs.msg import CarControl

GPS_TOPIC = '/jarcar/gps'
IMU_TOPIC = '/jarcar/imu'
REAL_ODOMETRY_TOPIC = '/jarcar/real_odometry'

STEERING_INPUT_TOPIC = '/jarcar/steering'
VELOCITY_INPUT_TOPIC = '/jarcar/velocity'

KALMAN_FILTER_TOPIC = '/jarcar/kalman_filtered_odom'
CAR_CONTROL_TOPIC   = '/jarcar/ackermann_control'

def normalize_angle(angle):
    # if angle >= np.pi:
    return (angle + np.pi) % (2 * np.pi) - np.pi
    # elif angle <= -np.pi:

    # return angle

class CarCanvas(FigureCanvas):
    def __init__(self, parent = None, width = 5, height = 5, dpi = 100, xlim = (0, 100), ylim = (0, 100)):
        self.default_xlim = xlim
        self.default_ylim = ylim

        self.figure = Figure(figsize = (width, height), dpi = dpi)
        self.axes = self.figure.add_subplot(111)
        self.axes.set_aspect('equal', adjustable='box')
        self.axes.set_xlim(self.default_xlim)
        self.axes.set_ylim(self.default_ylim)
        
        self.x = [0.0]
        self.y = [0.0]

        self.line, = self.axes.plot(self.x, self.y, 'r-')
        super().__init__(self.figure)

    def addPoint(self, x, y):
        """
            Given a pair x, y, adds the point to the plot.
            The plot has a limit of 300 points.
        """
        if (len(self.x) > 300):
            self.y.pop(0)
            self.x.pop(0)
        self.x.append(x)
        self.y.append(y)
        self.update_data()

    def update_data(self):
        """
            Updates the x and y data of the plot. If a scroll gap is
            specified, then the plot starts to scroll before it reaches this gap.
        """

        # Update the data of the plot to match that of the arrays.
        self.line.set_xdata(self.x)
        self.line.set_ydata(self.y)
        
        lastx = self.x[-1]
        lasty = self.y[-1]

        self.axes.set_xlim(lastx - (self.default_xlim[1] - self.default_xlim[0]) / 2, lastx + (self.default_xlim[1] - self.default_xlim[0]) / 2)
        self.axes.set_ylim(lasty - (self.default_ylim[1] - self.default_ylim[0]) / 2, lasty + (self.default_ylim[1] - self.default_ylim[0]) / 2)
        
        # Redraw plot (slow)
        self.figure.canvas.draw()
        #self.figure.canvas.flush_events()

class CarNodeObject(Node, QObject):
    kalman_filter_signal = pyqtSignal(PoseStamped)
    car_control_signal   = pyqtSignal(CarControl)

    def __init__(self):
        rclpy.init(args = None)
        super().__init__("node_thread")
        self.gps_odometry_pub = self.create_publisher(PoseStamped, GPS_TOPIC, 10)
        self.imu_odometry_pub = self.create_publisher(PoseStamped, IMU_TOPIC, 10)
        self.real_odometry_pub = self.create_publisher(PoseStamped, REAL_ODOMETRY_TOPIC, 10)

        self.steering_input_pub = self.create_publisher(Float64, STEERING_INPUT_TOPIC, 10)
        self.velocity_input_pub = self.create_publisher(Int64, VELOCITY_INPUT_TOPIC, 10)

        self.kalman_filter_sub = self.create_subscription(PoseStamped, KALMAN_FILTER_TOPIC, self.kalman_filter_callback, 10)
        self.car_control_sub   = self.create_subscription(CarControl, CAR_CONTROL_TOPIC, self.car_control_callback, 10)

        super(Node, self).__init__()

    def start_node(self):
        rclpy.spin(self)
        self.destroy_node()
        rclpy.shutdown()

    def kalman_filter_callback(self, msg):
        self.kalman_filter_signal.emit(msg)

    def car_control_callback(self, msg):
        self.car_control_signal.emit(msg)

class Car(CarCanvas): # it's better to inherite firstly Node, secondly CarCanvas. be careful about __mro__
    """
        The car which starts at point (0, 0) when heading = 0, steering = 0, velocity = 0
    """
    IMU_VARIANCE = 1 #0.1
    GPS_VARIANCE = 2 # 0.1
    def __init__(self, *args, **kwargs):

        super().__init__(*args, **kwargs)

        self.velocity = 0.0
        self.steering = 0.0
        self.heading = 0.0

        self.L = 2
        self.sim_dt = 0.1

        self.x_imu_measured = []
        self.y_imu_measured = []

        self.x_gps_measured = []
        self.y_gps_measured = []

        self.x_kalman = [0]
        self.y_kalman = [0]

        # self.imu_sensor()
        # self.gps()

        self.imu_line,    = self.axes.plot(self.x_imu_measured, self.y_imu_measured, 'g-', label = "imu")
        self.gps_line,    = self.axes.plot(self.x_gps_measured, self.y_gps_measured, 'b-', label = "gps")
        self.kalman_line, = self.axes.plot(self.x_kalman, self.y_kalman, color = "purple", linestyle = "--", label = "kalman")

        self.figure.legend()

    def init_ros(self):
        self.ros_thread = QThread()
        self.ros_object = CarNodeObject()
        self.ros_object.kalman_filter_signal.connect(self.kalman_filter_callback)
        self.ros_object.car_control_signal.connect(self.car_control_callback)
        self.ros_object.moveToThread(self.ros_thread)
        self.ros_thread.started.connect(self.ros_object.start_node)
        self.ros_thread.start()

    def set_steering(self, steering):
        if steering > 0.5235987755983: # np.pi / 4:
            self.steering = 0.5235987755983 # np.pi / 4
        elif steering < -0.5235987755983: # -np.pi / 4:
            self.steering = -0.5235987755983 # -np.pi / 4
        else:
            self.steering = steering

    def set_velocity(self, velocity):
        if velocity > 10:
            self.velocity = 10
        elif velocity < -10:
            self.velocity = -10
        else:
            self.velocity = velocity

    def imu_sensor(self):
        if len(self.x_imu_measured) > 300:
            self.x_imu_measured = self.x_imu_measured[1::]
        x_imu = np.random.normal(self.x[-1], self.IMU_VARIANCE)
        # self.x_imu_measured.append(x_imu)

        if len(self.y_imu_measured) > 300:
            self.y_imu_measured = self.y_imu_measured[1::]
        y_imu = np.random.normal(self.y[-1], self.IMU_VARIANCE)
        #self.y_imu_measured.append(y_imu)

        imu_odometry_msg = PoseStamped()
        imu_odometry_msg.pose.position.x = x_imu
        imu_odometry_msg.pose.position.y = y_imu

        self.ros_object.imu_odometry_pub.publish(imu_odometry_msg)
        
    def gps(self):
        if len(self.x_gps_measured) > 300:
            self.x_gps_measured = self.x_gps_measured[1::]
        x_gps = np.random.normal(self.x[-1], self.GPS_VARIANCE)
        self.x_gps_measured.append(x_gps)

        if len(self.y_gps_measured) > 300:
            self.y_gps_measured = self.y_gps_measured[1::]
        y_gps = np.random.normal(self.y[-1], self.GPS_VARIANCE)
        self.y_gps_measured.append(y_gps)

        gps_odometry_msg = PoseStamped()
        gps_odometry_msg.pose.position.x = x_gps
        gps_odometry_msg.pose.position.y = y_gps

        self.ros_object.gps_odometry_pub.publish(gps_odometry_msg)

    def real_odometry(self):
        real_odometry_msg = PoseStamped()
        real_odometry_msg.pose.position.x = self.x[-1]
        real_odometry_msg.pose.position.y = self.y[-1]
        real_odometry_msg.pose.orientation.w = self.heading

        self.ros_object.real_odometry_pub.publish(real_odometry_msg)

    def update_data(self):
        # self.imu_line.set_xdata(self.x_imu_measured)
        # self.imu_line.set_ydata(self.y_imu_measured)

        # self.gps_line.set_xdata(self.x_gps_measured)
        # self.gps_line.set_ydata(self.y_gps_measured)
        
        super().update_data()
        
    def update_simulation(self):
        x_new, y_new = self.x[-1], self.y[-1]
        x_new += self.velocity * np.cos(self.heading) * self.sim_dt
        y_new += self.velocity * np.sin(self.heading) * self.sim_dt
        self.heading += self.velocity / self.L * np.tan(self.steering) * self.sim_dt
        self.heading = normalize_angle(self.heading)

        velocity_msg = Int64()
        velocity_msg.data = int(self.velocity)
        self.ros_object.velocity_input_pub.publish(velocity_msg)

        steering_msg = Float64()
        steering_msg.data = self.steering
        self.ros_object.steering_input_pub.publish(steering_msg)

        self.imu_sensor()
        self.gps()
        self.real_odometry()

        self.addPoint(x_new, y_new)

    @pyqtSlot(PoseStamped)
    def kalman_filter_callback(self, msg):
        if len(self.x_kalman) > 3000:
            self.x_kalman = self.x_kalman[1::]

        if len(self.y_kalman) > 3000:
            self.y_kalman = self.y_kalman[1::]

        self.x_kalman.append(msg.pose.position.x)
        self.y_kalman.append(msg.pose.position.y)

        self.kalman_line.set_xdata(self.x_kalman)
        self.kalman_line.set_ydata(self.y_kalman)

        self.figure.canvas.draw()
        

    @pyqtSlot(CarControl)
    def car_control_callback(self, msg):
        self.set_velocity(msg.velocity)
        self.set_steering(msg.steering)

class CarMainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.car = Car()
        self.car.init_ros()
        self.steering_label = QLabel("Steering [degree] : 0")
        self.velocity_label = QLabel("Velocity [m/s] : 0")

        self.information_layout = QHBoxLayout()
        self.information_layout.addWidget(self.velocity_label)
        self.information_layout.addWidget(self.steering_label)

        self.graph_layout = QVBoxLayout()
        self.graph_layout.addWidget(self.car, 0, Qt.AlignTop)
        self.graph_layout.addLayout(self.information_layout, Qt.AlignBottom)

        #widget = QWidget()
        self.setLayout(self.graph_layout)
        #self.setCentralWidget(widget)

        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy.setHeightForWidth(True)
        self.setSizePolicy(sizePolicy)
        
        self.show()

        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.car.update_simulation)
        self.timer.start()

    def heightForWidth(self, width):
        return width

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_W:
            self.car.set_velocity(self.car.velocity + 1)
            self.velocity_label.setText(f"Velocity [m/s] : {self.car.velocity}")
        elif event.key() == Qt.Key_S:
            self.car.set_velocity(self.car.velocity - 1)
            self.velocity_label.setText(f"Velocity [m/s] : {self.car.velocity}")
        elif event.key() == Qt.Key_A:
            self.car.set_steering(self.car.steering + np.deg2rad(5))
            self.steering_label.setText(f"Steering [degree] : {self.car.steering}")
        elif event.key() == Qt.Key_D:
            self.car.set_steering(self.car.steering - np.deg2rad(5))
            self.steering_label.setText(f"Steering [degree] : {self.car.steering}")

# rclpy.init(args=None)
app = QApplication(sys.argv)
w = CarMainWindow()

app.exec_()

# w.car.destroy_node()
# rclpy.shutdown()
#app.destroy_node()
#rclpy.shutdown()

