#!/usr/bin/env python

import cv2
import rospy
from std_msgs.msg import Float32
import numpy as np
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from line_detector.msg import values

video_path = 'video_pista2.mp4'

msg = values()
#bridge = CvBridge()

rospy.init_node("line")
pub = rospy.Publisher("/line", values, queue_size = 1)
#img_pub = rospy.Publisher("/view", Image, queue_size = 10)
#rospy.Subscriber("/video_source/raw", Image, image_callback)

rate = rospy.Rate(100)

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


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
    blur = cv2.GaussianBlur(gris, (3, 3), 0)

    # Aplicar  de bordes con el algoritmo Canny
    
    bordes = cv2.Canny(blur, 10, 80)
    vertices = np.array([[(80, frame.shape[0]), (300, 300), (700, 300), (frame.shape[1]-80, frame.shape[0])]], dtype=np.int32)
    mask = np.zeros_like(bordes)
    polly = cv2.fillPoly(mask, vertices, 255)

    cv2.imshow("polly", polly)

    # hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    # lower = np.array([0, 0, 0], np.uint8)
    # upper = np.array([180, 255, 200], np.uint8)

    masked_edges = cv2.bitwise_and(bordes, mask)
    masked_image = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('Mascara', masked_edges)

    lineas = cv2.HoughLinesP(masked_edges, 3, np.pi / 180, 20, minLineLength=90, maxLineGap=20)

    lineas_filtradas = []
    if lineas is not None:
        for linea in lineas:
            x1, y1, x2, y2 = linea[0]
            pendiente = (y2 - y1) / (x2 - x1 + 1e-6)
            if abs(pendiente) > 0.1:
                lineas_filtradas.append((x1, y1, x2, y2))

    return lineas_filtradas

# Funcis detectadas en la imagen original
def dibujar_lineas(frame, lineas):
    global msg, pub
    #print("estoy en dibujar_lineas")
    if lineas is not None:
        for linea in lineas:
            x1, y1, x2, y2 = linea
            msg.angulo = np.arctan2((y2-y1),(x2-x1))
            msg.xr = (x1 + x2)/2
            msg.yr = (y1 + y2)/2
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
    dispW = 320
    dispH = 240
    flip = 2
    camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    #cap = cv2.VideoCapture('/home/jgusbc/catkin_ws/src/line_detection/scripts/vuelta_derecha.mp4')

    while cap.isOpened():
        
        ret, frame = cap.read()
        #print("antes del if")
        if not ret:
            break
        
        #img_pub.publish(bridge.cv2_to_imgmsg(frame, "rgb8"))
        #print("pase el if")
        #frame = frame[250:540, 0:960]

        lineas = detectar_lineas(frame)
        lineas = merge_similar_lines(lineas,100)
        frame_con_lineas = dibujar_lineas(frame, lineas)
        
        cv2.imshow('Seguimiento de Lineas', frame_con_lineas)

        if cv2.waitKey(50) & 0xFF == ord('q'):
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





