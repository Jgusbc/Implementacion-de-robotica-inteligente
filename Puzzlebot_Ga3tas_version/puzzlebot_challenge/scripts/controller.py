#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32

# Acotamos angulo
def angle_delimit(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))

# Iniciamos la clase controlador
class Controller:
    # Constructor de la clase
    def __init__(self):
        # Variables de posiciones
        self.x_pos = 0
        self.y_pos = 0
        self.theta_angle = 0
        # Constantes de PID
        self.linear_kp = rospy.get_param("/linear_Kp", 0)
        self.angular_kp = rospy.get_param("/angular_Kp", 0)
        # Mensajes
        self.msg = Twist()
        self.pose = Pose2D()
        # Para  metros del Puzzlebot
        self.wheel_radius = 0.05
        self.axle_track = 0.19
        self.max_w = 8
        # Posiciones deseadas
        self.target_x = 0
        self.target_y = 0
        # Velocidades de las llantas
        self.wl = 0
        self.wr = 0
        # Tiempo
        self.dt = 1/float(rospy.get_param("/noderate", 100))
        # Subscribers
        self.retroL = rospy.Subscriber("wl", Float32, self.wl_callback)
        self.retroR = rospy.Subscriber("wr", Float32, self.wr_callback)
        self.retroPose = rospy.Subscriber("setPoint", Pose2D, self.pose_callback)
        # Publisher
        self.cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.trek_done = rospy.Publisher("route_end", Float32, queue_size=10)
        # Finalizar ruta
        self.done = False
        rospy.loginfo(self.dt)

    # Callbacks
    def wl_callback(self, msg):
        self.wl = msg.data

    def wr_callback(self, msg):
        self.wr = msg.data

    def pose_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.done = False
    # Calculo de odometria
    def update(self):
        lv = self.wheel_radius * (self.wr + self.wl) / 2
        wv = (self.wr - self.wl) / self.axle_track * self.wheel_radius
        self.x_pos += lv * self.dt * np.cos(self.theta_angle)
        self.y_pos += lv * self.dt * np.sin(self.theta_angle)
        self.theta_angle += wv * self.dt
        if self.theta_angle < 0:
            self.theta_angle += 2 * np.pi
        if self.theta_angle > 2 * np.pi:
            self.theta_angle -= 2 * np.pi
    # Accion de control
    def control(self):
        # Calculo de errores
        l_error = np.sqrt((self.target_x - self.x_pos) ** 2 + (self.target_y - self.y_pos) ** 2)
        w_error = angle_delimit(np.arctan2(self.target_y - self.y_pos, self.target_x - self.x_pos) - self.theta_angle)
        # Checar si estamos cerca del punto
        if l_error > 0.1:
            u_l = self.linear_kp * l_error
            u_w = self.angular_kp * w_error
            # Acotamos las salidas de control
            if abs(u_l) > 13:
                u_w = np.copysign(13, u_l)
            if abs(u_w) > 9:
                u_w = np.copysign(9, u_w)
            self.msg.linear.x = u_l
            self.msg.angular.z = u_w
            if abs(w_error) >= np.pi:
                rospy.loginfo("ERROR ANGULAR EXCEDIDO: %f", w_error)
        else:
            # Mandamos mensaje que ya llegamos a la posicion deseada
            if not self.done:
                self.done = True
                self.trek_done.publish(1)
                rospy.loginfo("X pos: %f", self.x_pos)
                rospy.loginfo("Y pos: %f", self.y_pos)
                rospy.loginfo("Theta angle: %f", self.theta_angle)
            self.msg.linear.x = 0
            self.msg.angular.z = 0
        # Publicamos la velocidad para el Puzzlebot
        self.cmd_vel.publish(self.msg)


    def shutdown(self):
        self.msg.linear.x = 0
        self.msg.angular.z = 0
        self.cmd_vel.publish(self.msg)

if __name__ == "__main__":
    rospy.init_node("Controller")
    controller = Controller()
    rate = rospy.Rate(rospy.get_param("/noderate", 100))
    rospy.on_shutdown(controller.shutdown)
    while not rospy.is_shutdown():
        controller.update()
        controller.control()
        rate.sleep()