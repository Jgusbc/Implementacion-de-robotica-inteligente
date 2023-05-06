#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32


def stop():
    msg.linear.x = 0
    msg.angular.z = 0
    cmd_vel.publish(msg)
    rospy.loginfo("posicion en x: %f", posicion_real_x)
    rospy.loginfo("posicion en y: %f", posicion_real_y)


msg = Twist()
pose = Pose2D()

# Datos del puzzlebot
r = 0.05
l = 0.19

# Posiciones del puzzlebot
posicion_real_x = 0
posicion_real_y = 0
posicion_real_w = 0

TargetX = 0
TargetY = 0

# Constantes del controlador
Kv = 0.45
Kw = 1.4867
Kiw = 1

# tiempos
tiempo_inicial = 0
tiempo_anterior = 0

# Velocidades de llantas
wl = 0
wr = 0


def controller():
    global posicion_anterior_w, posicion_real_w, tiempo_anterior, TargetX, TargetY, posicion_real_x, posicion_real_y, Kw, wr, wl, r, l

    rate.sleep()

    posicion_anterior_x = 0
    posicion_anterior_y = 0
    tiempo_inicial = rospy.get_time()
    tiempo_anterior = 0
    e = 1
    while((abs(posicion_real_x - TargetX) > 0.05) or (abs(posicion_real_y - TargetY) > 0.05)):
        rospy.loginfo("wr: %f", wr)
        rospy.loginfo("wl: %f", wl)
        ew_suma = 0

        tiempo_actual = rospy.get_time() - tiempo_inicial
        dt = tiempo_actual - tiempo_anterior
        # rospy.loginfo("DIFERENCIAL DE TIEMPO: %f", dt)

        if (abs(wr - wl) > 0.05):
            R = (l * ((r * wr) + (r * wl))) / (2 * ((r * wr) - (r * wl)))
            w = ((r * wr) - (r * wl)) / l

            posicion_real_x = R * np.cos(w * dt) * np.sin(posicion_real_w) + R * np.cos(posicion_real_w) * np.sin(
                w * dt) + posicion_real_x - R * np.sin(posicion_real_w)
            posicion_real_y = R * np.sin(w * dt) * np.sin(posicion_real_w) - R * np.cos(posicion_real_w) * np.cos(
                w * dt) + posicion_real_y + R * np.cos(posicion_real_w)
            posicion_real_w = posicion_real_w + w * dt
        else:
            posicion_real_x = posicion_real_x + ((wr+wl)/2) * r * dt * np.cos(posicion_real_w)
            posicion_real_y = posicion_real_y + ((wr+wl)/2) * r * dt * np.sin(posicion_real_w)

        rospy.loginfo("posicion X: %f", posicion_real_x)
        rospy.loginfo("target X: %f", TargetX)

        rospy.loginfo("posicion Y: %f", posicion_real_y)
        rospy.loginfo("target Y: %f", TargetY)

        rospy.loginfo("posicion W: %f", posicion_real_w)

        tiempo_anterior = tiempo_actual

        x1 = (TargetX - posicion_real_x) ** 2
        y1 = (TargetY - posicion_real_y) ** 2
        e = np.sqrt(x1 + y1)
        rospy.loginfo("Error d: %f", e)

        y2 = TargetY - posicion_real_y
        x2 = TargetX - posicion_real_x
        ew = np.arctan2(y2, x2) - posicion_real_w
        rospy.loginfo("Error w: %f", ew)

        msg.linear.x = 0.2 #Kv * e

        ew_suma += ew*dt

        msg.angular.z = (Kw * ew) + Kiw*ew_suma

        if (Kv * e >= 13):
            msg.linear.x = 13
        elif (Kv * e <= -13):
            msg.linear.x = -13

        if (Kw * ew >= 13):
            msg.angular.z = 13
        elif (Kw * ew <= -13):
            msg.angular.z = -13

        cmd_vel.publish(msg)

    msg.linear.x = 0
    msg.angular.z = 0
    cmd_vel.publish(msg)


def callback2(msg):
    global wr
    wr = msg.data


def callback(msg):
    global wl
    wl = msg.data


if __name__ == '__main__':
    # global cmd_vel, TargetX, TargetY, tiempo_inicial

    # Crea el topico donde se publica la velocidad e inicializa el nodo
    cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    rospy.init_node("Controller")

    retroL = rospy.Subscriber("wl", Float32, callback)
    retroR = rospy.Subscriber("wr", Float32, callback2)

    rate = rospy.Rate(1000)
    rate.sleep()

    tiempo_inicial = rospy.get_time()

    rospy.on_shutdown(stop)

    while not rospy.is_shutdown():
        # Pide la posicion en X y Y del nuevo punto al que quiere llegar
        rospy.loginfo("Ingrese nueva posicion en X:")
        TargetX = float(input())

        rospy.loginfo("Ingrese nueva posicion en Y:")
        TargetY = float(input())

        controller()

        msg.linear.x = 0
        msg.angular.z = 0
        cmd_vel.publish(msg)

        rate.sleep()
