#!/usr/bin/env python

import cv2
import rospy
from std_msgs.msg import Float32
import numpy as np

video_path = 'video_pista2.mp4'

msg = Float32()

rospy.init_node("line")
pub = rospy.Publisher("/line", Float32, queue_size = 1)
rate = rospy.Rate(100)


def detectar_lineas(frame):
    # Convertir la imagen a espacio de color HLS
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

    # Definir el rango de valores para el componente de luz (L)
    luz_baja = 0  # Valor s oscuras
    luz_alta = 100  # Valoreas oscuras
    umbral_bajo = np.array([0, luz_baja, 0])
    umbral_alto = np.array([255, luz_alta, 255])

    mascara = cv2.inRange(hls, umbral_bajo, umbral_alto)
    resultado = cv2.bitwise_and(frame, frame, mask=mascara)

    # Convertir la imagen resultante a escala de grises
    gris = cv2.cvtColor(resultado, cv2.COLOR_BGR2GRAY)

    # Aplicar desenfoque gaussiano para reducir el ruido
    blur = cv2.GaussianBlur(gris, (5, 5), 0)

    # Aplicar  de bordes con el algoritmo Canny
    bordes = cv2.Canny(blur, 50, 150)

    lineas = cv2.HoughLinesP(bordes, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=50)

    lineas_filtradas = []
    if lineas is not None:
        for linea in lineas:
            x1, y1, x2, y2 = linea[0]
            pendiente = (y2 - y1) / (x2 - x1 + 1e-6)
            if abs(pendiente) > 0.1:
                if((320< x1 <640) and (320 < x2 <640) and y1 > 80 and y2 > 80):
                    lineas_filtradas.append((x1, y1, x2, y2))

    return lineas_filtradas

# Funcis detectadas en la imagen original
def dibujar_lineas(frame, lineas):
    global msg, pub
    #print("estoy en dibujar_lineas")
    if lineas is not None:
        for linea in lineas:
            x1, y1, x2, y2 = linea
            msg = np.arctan2((y2-y1),(x2-x1))
            #msg = np.degrees(msg)
            #if(((y2 - y1) / (x2 - x1 + 1e-6)) < -0.1):
                #msg = np.pi - msg
            pub.publish(msg)
            #rospy.loginfo(msg)
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

    return frame

def merge_similar_lines(lines, threshold):
    merged_lines = []

    for line in lines:
        x1, y1, x2, y2 = line

        # Calcula los puntos medios de la linea
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2

        # Bandera para indicar si se encontro una linea similar existente
        line_merged = False

        for merged_line in merged_lines:
            merged_x1, merged_y1, merged_x2, merged_y2 = merged_line

            # Calcula los puntos medios de la linea fusionada existente
            merged_mid_x = (merged_x1 + merged_x2) / 2
            merged_mid_y = (merged_y1 + merged_y2) / 2

            # Calcula la distancia euclidiana entre los puntos medios
            distance = np.sqrt((mid_x - merged_mid_x) ** 2 + (mid_y - merged_mid_y) ** 2)

            # Comprueba si la linea es similar a la linea fusionada existente
            if distance < threshold:
                # Fusiona la linea con la linea existente
                merged_x1 = min(x1, merged_x1)
                merged_y1 = min(y1, merged_y1)
                merged_x2 = max(x2, merged_x2)
                merged_y2 = max(y2, merged_y2)
                merged_line = [merged_x1, merged_y1, merged_x2, merged_y2]
                line_merged = True
                break

        if not line_merged:
            # Agrega la linea como una linea fusionada nueva
            merged_lines.append(line)

    return merged_lines

# Funcieas en un video
def seguir_lineas():
    global video_path
    #print("estoy en seguir_lineas")
    cap = cv2.VideoCapture('/home/jgusbc/catkin_ws/src/line_detection/scripts/vuelta_derecha.mp4')

    while cap.isOpened():
        
        ret, frame = cap.read()
        #print("antes del if")
        if not ret:
            break
        
        #print("pase el if")
        frame = frame[250:540, 0:960]

        lineas = detectar_lineas(frame)
        lineas = merge_similar_lines(lineas,100)
        frame_con_lineas = dibujar_lineas(frame, lineas)

        cv2.imshow('Seguimiento de Lineas', frame_con_lineas)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# Ruta del video a procesar
if __name__ == '__main__':
    #global msg 
    while not rospy.is_shutdown():
        #print("estoy en el while")
    # Realizar el seguimiento de leas en el video
        seguir_lineas()
        rate.sleep()





