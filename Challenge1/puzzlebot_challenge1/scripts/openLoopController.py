#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist

#mensaje para enviar velocidades
msg = Twist()

#velocidades a las que queremos mover el puzzlebot
vel_linear_i = 1.5
vel_linear = vel_linear_i
vel_linear_anterior = 0
vel_angular_i = 1.5
vel_angular = vel_angular_i
vel_angular_anterior = 0

#posicion y direccion del puzzlebot
posicionX = 0
posicionY = 0
tetha = 0

#posicion a donde quiere llegar
posicionX2 = 2
posicionY2 = 2

#direccion a donde debe ir
alpha = 0

#Funcion que detiene el puzzlebot cuando se termina el programa
def stop():
    global msg, cmd_vel
    msg.linear.x = 0
    msg.linear.z = 0
    cmd_vel.publish(msg)

#Aplica pitagoras para obtener distancia entre punto actual y punto deseado
def calcularDistancia():
    global posicionX, posicionX2, posicionY, posicionY2
    x = posicionX2 - posicionX
    y = posicionY2 - posicionY
    suma = x**2 + y**2
    d = np.sqrt(suma)

    return d

#Calcula el angulo hacia donde debe mirar el puzzlebot antes de avanzar
def calcularAngulo():
    global alpha, posicionY, posicionY2, posicionX2, posicionX

    # Checa si el punto esta sobre el mismo eje Y que el puzzlebot para no tener problemas con
    # calcular la arctang
    if(posicionY2 == posicionY):

#Checa si esta a la derecha o la izquierda del puzzlebot y asigna el valor de alpha
        if(posicionX2 > posicionX):
            alpha = 0
        else:
            alpha = np.pi
    #Checa si el punto esta sobre el mismo eje X que el puzzlebot
    elif (posicionX2 == posicionX):
        #Checa si esta mas arriba o abajo para calcular alpha
        if(posicionY2 > posicionY):
            alpha = np.pi/2
        else:
            alpha = 3*np.pi/2
    else:
        #Calcula el tamanio de los catetos y usa arctan para sacar alpha
        x = np.abs(posicionX2 - posicionX)
        y = np.abs(posicionY2 - posicionY)
        alpha = np.arctan(y/x)
        #Checa en cual cuadrante esta el punto con respecto al puzzlebot y en base a eso
        #calcula el valor total de alpha dependiendo del cuadrante
        if(posicionX2 < posicionX and posicionY2 > posicionY):
            alpha = np.pi - alpha
        elif(posicionX2 < posicionX and posicionY2 < posicionY):
            alpha = np.pi + alpha
        elif(posicionX2 > posicionX and posicionY2 < posicionY):
            alpha = 2*np.pi - alpha


    return alpha

#Hace girar phi grados al puzzlebot a la derecha o a la izq
#Velocidad positiva izquierda, negativa derecha
def Girar(phi,flag):
    global msg, cmd_vel, vel_angular, vel_angular_anterior

    #Multiplica la velocidad por la direccion
    rate.sleep()

    #calcula el timepo inicial y el tiempo actual
    tiempo_ini = rospy.get_time()
    tiempo = rospy.get_time() - tiempo_ini

    vel_angular = vel_angular_i

    #Espera a que el tiempo sea el necesario para que el puzzlebot haya terminado de girar
    while(tiempo < (phi/vel_angular)):
        vel_angular_guardar =vel_angular
        vel_angular = vel_angular + (vel_angular - vel_angular_anterior) * -0.7 * (np.exp(-1 * tiempo))
        msg.angular.z = vel_angular*flag
        msg.linear.x = 0
        cmd_vel.publish(msg)
        vel_angular_anterior = vel_angular_guardar
        tiempo =rospy.get_time() - tiempo_ini
        rospy.loginfo("Girando, Grados girados: %s", tiempo*vel_angular)

    vel_angular_anterior = 0
    #Detiene el puzzlebot
    msg.angular.z = 0
    msg.linear.x = 0
    cmd_vel.publish(msg)
    rate.sleep()
    rospy.loginfo("Giro terminado")

    return

#Determina hacia que direccion y cuantos grados debe girar el puzzlebot
def GirarPuzzlebot():
    global tetha, alpha

    #Si la direccion esta bien no gira
    if(alpha == tetha):
        return

    #Checa si el angulo a donde voltea el puzzlebot es mayor que el angulo a donde debe voltear
    if(tetha > alpha):
        #Checa si es mejor girar a la derecha o izquierda
        if(tetha - alpha <= np.pi):
            Girar(tetha-alpha,-1)
        else:
            Girar(2*np.pi - tetha + alpha,1)

    # Checa si el angulo a donde voltea el puzzlebot es menor que el angulo a donde debe voltear
    elif(tetha < alpha):

        # Checa si es mejor girar a la derecha o izquierda
        if(alpha - tetha <= np.pi):
            Girar(alpha-tetha,1)
        else:
            Girar(2*np.pi + tetha - alpha,-1)

    #Actualiza la direccion a la que esta volteando el puzzlebot
    tetha = alpha
    return

def Forward(d):
    global msg, cmd_vel, vel_linear, posicionX, posicionX2, posicionY, posicionY2, vel_linear_anterior

    #Le da una velocidad linear al puzzlbot

    msg.angular.z = 0
    msg.linear.x = vel_linear
    cmd_vel.publish(msg)
    vel_linear = vel_linear_i
    rate.sleep()

    # calcula el timepo inicial y el tiempo actual
    tiempo_ini = rospy.get_time()
    tiempo = rospy.get_time() - tiempo_ini

    msg.angular.z = 0

    # Espera a que el tiempo sea el necesario para que el puzzlebot haya terminado de avanzar
    while (tiempo < d/vel_linear ):
        vel_linear_guardar = vel_linear
        vel_linear = vel_linear + (vel_linear - vel_linear_anterior) * -0.7 * (np.exp(-1 * tiempo))
        msg.linear.x = vel_linear
        cmd_vel.publish(msg)
        vel_linear_anterior = vel_linear_guardar
        rospy.loginfo("Vel anterior: %s", vel_linear_anterior)
        rospy.loginfo("Vel: %s", vel_linear)
        rospy.loginfo("Avanzando, metros avanzados: %s", tiempo * vel_linear)
        tiempo = rospy.get_time() - tiempo_ini

    #Detiene el puzzlebot
    vel_linear_anterior=0
    msg.angular.z = 0
    msg.linear.x = 0
    cmd_vel.publish(msg)
    rospy.loginfo("Distancia alcanzada")

    posicionX = posicionX2
    posicionY = posicionY2
    return

def MoverPuzzlebot(d):
    #Primero gira el puzzlebot y luego lo hace avanzar
    GirarPuzzlebot()
    rate.sleep()
    rate.sleep()
    Forward(d)
    return


if __name__ == '__main__':
    global cmd_vel, alpha, posicionY, posicionY2, posicionX, posicionX2

    #Crea el topico donde se publica la velocidad e inicializa el nodo
    cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    rospy.init_node("openLoopController")
    rate = rospy.Rate(10)

    rospy.on_shutdown(stop)

    while not rospy.is_shutdown():
        #Pide la posicion en X y Y del nuevo punto al que quiere llegar
        rospy.loginfo("Ingrese nueva posicion en X:" )
        posicionX2 = float(raw_input())

        rospy.loginfo("Ingrese nueva posicion en Y:")
        posicionY2 = float(raw_input())

        rospy.loginfo("Ingrese nueva velocidad angular:")
        vel_angular_i = float(raw_input())

        rospy.loginfo("Ingrese nueva velocidad linear:")
        vel_linear_i = float(raw_input())

        #Checa si el punto es valido
        if(posicionX2 >= 0 and posicionX2 <= 2 and posicionY2 >= 0 and posicionY2 <=2):

            #Checa si el punto no es en el que se encuentra
            if(posicionX2 != posicionX or posicionY2 != posicionY):

                #Calcula la distancia que se debe mover
                d = calcularDistancia()

                #Calcula la direccion a la que debe caminar
                calcularAngulo()

                #Mueve al puzzlebot
                MoverPuzzlebot(d)

                #Indica que ya llego a la nueva posicon
                rospy.loginfo("Nueva posicion: %f, %f", posicionX, posicionY)

        rate.sleep()
