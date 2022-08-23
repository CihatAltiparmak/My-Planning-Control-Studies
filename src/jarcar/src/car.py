import numpy as np

import sys
import random
import matplotlib
matplotlib.use("Qt5Agg")

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow,QWidget, \
                            QApplication, QLabel, QHBoxLayout, QVBoxLayout, QSizePolicy
from PyQt5.QtCore import QThread, Qt, QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

GPS_TOPIC = '/jarcar/gps'
IMU_TOPIC = '/jarcar/imu'
REAL_ODOMETRY_TOPIC = '/jarcar/real_odometry'

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

class Car(Node, CarCanvas): # it's better to inherite firstly Node, secondly CarCanvas. be careful about __mro__
    """
        The car which starts at point (0, 0) when heading = 0, steering = 0, velocity = 0
    """
    IMU_VARIANCE = 1 #0.1
    GPS_VARIANCE = 2 # 0.1
    def __init__(self, *args, **kwargs):
        super().__init__("car_publisher")
        self.gps_odometry_pub = self.create_publisher(PoseStamped, GPS_TOPIC, 10)
        self.imu_odometry_pub = self.create_publisher(PoseStamped, IMU_TOPIC, 10)
        self.real_odometry_pub = self.create_publisher(PoseStamped, REAL_ODOMETRY_TOPIC, 10)

        super(Node, self).__init__(*args, **kwargs)

        self.velocity = 0.0
        self.steering = 0.0
        self.heading = 0.0

        self.L = 2
        self.sim_dt = 0.1

        self.x_imu_measured = []
        self.y_imu_measured = []

        self.x_gps_measured = []
        self.y_gps_measured = []

        self.imu_sensor()
        self.gps()

        self.imu_line, = self.axes.plot(self.x_imu_measured, self.y_imu_measured, 'g-', label = "imu")
        self.gps_line, = self.axes.plot(self.x_gps_measured, self.y_gps_measured, 'b-', label = "gps")

        self.figure.legend()

    def set_steering(self, steering):
        if steering > np.pi / 4:
            self.steering = np.pi / 4
        elif steering < -np.pi / 4:
            self.steering = -np.pi / 4
        else:
            self.steering = steering

    def set_velocity(self, velocity):
        self.velocity = velocity

    def imu_sensor(self):
        if len(self.x_imu_measured) > 300:
            self.x_imu_measured = self.x_imu_measured[1::]
        x_imu = np.random.normal(self.x[-1], self.IMU_VARIANCE)
        self.x_imu_measured.append(x_imu)

        if len(self.y_imu_measured) > 300:
            self.y_imu_measured = self.y_imu_measured[1::]
        y_imu = np.random.normal(self.y[-1], self.IMU_VARIANCE)
        self.y_imu_measured.append(y_imu)

        imu_odometry_msg = PoseStamped()
        imu_odometry_msg.pose.position.x = x_imu
        imu_odometry_msg.pose.position.y = y_imu

        self.imu_odometry_pub.publish(imu_odometry_msg)
        
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

        self.gps_odometry_pub.publish(gps_odometry_msg)

    def real_odometry(self):
        real_odometry_msg = PoseStamped()
        real_odometry_msg.pose.position.x = self.x[-1]
        real_odometry_msg.pose.position.y = self.y[-1]

        self.real_odometry_pub.publish(real_odometry_msg)

    def update_data(self):
        self.imu_line.set_xdata(self.x_imu_measured)
        self.imu_line.set_ydata(self.y_imu_measured)

        self.gps_line.set_xdata(self.x_gps_measured)
        self.gps_line.set_ydata(self.y_gps_measured)
        
        super().update_data()
        
    def update_simulation(self):
        r = None
        if self.steering == 0:
            r = float("inf")
        else:
            r = self.L / np.tan(self.steering)

        delta_heading = self.velocity * self.sim_dt / r
        x_new = self.x[-1]
        y_new = self.y[-1]

        if r == float("inf"):
            x_new += self.velocity * self.sim_dt * np.cos(self.heading)
            y_new += self.velocity * self.sim_dt * np.sin(self.heading)
        else:
            x_new += 2 * r * np.sin(delta_heading / 2) * np.cos(self.heading - delta_heading / 2)
            y_new += 2 * r * np.sin(delta_heading / 2) * np.sin(self.heading - delta_heading / 2)
            self.heading += delta_heading


        self.imu_sensor()
        self.gps()
        self.real_odometry()

        self.addPoint(x_new, y_new)

class CarMainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.car = Car()
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

rclpy.init(args=None)
app = QApplication(sys.argv)
w = CarMainWindow()
app.exec_()

w.car.destroy_node()
rclpy.shutdown()
#app.destroy_node()
#rclpy.shutdown()

