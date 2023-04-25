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

#Datos del puzzlebot
r = 0.05
l = 0.19

#Posiciones del puzzlebot
posicion_real_x = 0
posicion_real_y = 0
posicion_real_w = 0

TargetX = 0
TargetY = 0

#Constantes del controlador
Kv = 0.25
Kw = 2.6

#tiempos
tiempo_inicial = 0
tiempo_anterior = 0

#Velocidades de llantas
wl = 0
wr = 0

def controller():
    global posicion_anterior_w, posicion_real_w, tiempo_anterior, TargetX, TargetY, posicion_real_x, posicion_real_y, Kw, wr, wl, r,l

    rate.sleep()

    tiempo_inicial = rospy.get_time()
    tiempo_anterior = 0
    e = 1
    while (e > 0.1 or ew > 0.03 or ew < -0.03):
        tiempo_actual = rospy.get_time() - tiempo_inicial
        dt = tiempo_actual - tiempo_anterior

        rospy.loginfo("DIFERENCIAL DE TIEMPO: %f", dt)

        vel_w = r * ((wr - wl) / l)
        vel_lineal = r * ((wr + wl) / 2)
        vel_x = vel_lineal * np.cos(posicion_real_w)
        vel_y = vel_lineal * np.sin(posicion_real_w)

        posicion_real_w = posicion_real_w + (vel_w * dt)

        rospy.loginfo("vel x: %f", vel_x)
        rospy.loginfo("vel y: %f", vel_y)

        posicion_real_x = posicion_real_x + (vel_x * dt)

        rospy.loginfo("posicion X: %f", posicion_real_x)
        rospy.loginfo("target X: %f", TargetX)

        posicion_real_y = posicion_real_y + (vel_y * dt)

        rospy.loginfo("posicion Y: %f", posicion_real_y)
        rospy.loginfo("target Y: %f", TargetY)

        tiempo_anterior = tiempo_actual

        x = (TargetX - posicion_real_x) ** 2
        y = (TargetY - posicion_real_y) ** 2
        e = np.sqrt(x + y)
        rospy.loginfo("Error d: %f", e)

        y = TargetY - posicion_real_y
        x = TargetX - posicion_real_x
        ew = np.arctan2(y, x) - posicion_real_w
        rospy.loginfo("Error w: %f", ew)

        msg.linear.x = Kv * e
        msg.angular.z = Kw * ew

        if(Kv*e >=1.2):
            msg.linear.x=1.2

        if(Kw*ew >= np.pi/2):
            msg.angular.z = np.pi/2
        elif(Kw*ew <= -np.pi/2):
            msg.angular.z = -np.pi/2

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
    global cmd_vel, TargetX, TargetY, tiempo_inicial

    #Crea el topico donde se publica la velocidad e inicializa el nodo
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
        TargetX = float(raw_input())

        rospy.loginfo("Ingrese nueva posicion en Y:")
        TargetY = float(raw_input())

        controller()

        msg.linear.x = 0
        msg.angular.z = 0
        cmd_vel.publish(msg)

        rate.sleep()


