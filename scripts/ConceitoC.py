#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import cv2.aruco as aruco
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from biblioteca import *
import cormodule

print("EXECUTE ANTES da 1.a vez: ")
print("wget https://github.com/Insper/robot21.1/raw/main/projeto/ros_projeto/scripts/MobileNetSSD_deploy.caffemodel")
print("PARA TER OS PESOS DA REDE NEURAL")

import visao_module


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
centro_robo = ()
media_creep = []
maior_area = 0

v = 0.1
w = math.pi/20.0

zero = Twist(Vector3(0,0,0), Vector3(0,0,0.0))
frente = Twist(Vector3(v,0,0), Vector3(0,0,0.0))
direita = Twist(Vector3(0,0,0), Vector3(0,0, -w ))
esquerda = Twist(Vector3(0,0,0), Vector3(0,0,w))

area = 0.0 # Variavel com a area do maior contorno

goal1 = ("blue", 22, "dog")

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()
DIREITA = True


def procesa_imagem(bgr):
    global centro
    bgr_copy = bgr.copy()
    if DIREITA:
        black = np.array([0,0,0])
        bgr_copy = cv2.rectangle(bgr_copy, (0,0), ((bgr.shape[1]//2)-100, bgr.shape[0]) , (0,0,255), -1)
        cv2.imshow("teste", bgr_copy)

    mask = segmenta_linha_amarela(bgr_copy)
    centro = center_of_mass(mask)

    cv2.imshow("mask", mask)

    return None
    
# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro
    global resultados
    global centro_robo
    global media_creep
    global maior_area

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

        img_copy = cv_image.copy()
        img_copy_aruco = cv_image.copy()
        img_copy_cor = cv_image.copy()
        procesa_imagem(img_copy)
        media_creep, maior_area, frame =  cormodule.identifica_cor(img_copy_cor, goal1[0])
        aruco_read(img_copy_aruco)
        centro_robo = (img_copy.shape[1]//2, img_copy.shape[0]//2)
        print("aquiii{}" .format(media_creep))

        if centro is not None:
            crosshair(cv_image, centro, 4, (0,0,255))

        cv2.imshow("cv_image", cv_image)
        cv2.imshow("cor", frame)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

def aruco_read(img_copy):
    aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    # parameters  = aruco.DetectorParameters_create()
    # parameters.minDistanceToBorder = 0
    # parameters.adaptiveThreshWinSizeMax = 1000
    gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict) #, parameters=parameters)
    # if ids is None:
    #     return None
    # for i in range(len(ids)):
    #     print('ID: {}'.format(ids[i]))
        
    #     for c in corners[i]: 
    #         for canto in c:
    #             print("Corner {}".format(canto))

    aruco.drawDetectedMarkers(img_copy, corners, ids)


    cv2.imshow('frame', img_copy)

def segue_linha():
    limiar = 50
    velocidade_saida.publish(zero)

    if centro is None:
        velocidade_saida.publish(esquerda)
        return None
    
    if len(centro) == 0 or len(centro_robo) == 0:
        velocidade_saida.publish(esquerda)
        return None

    if (centro[0] < centro_robo[0]+limiar) and (centro[0] > centro_robo[0]-limiar):
        velocidade_saida.publish(frente)
            
    elif (centro[0] > centro_robo[0]):
        velocidade_saida.publish(direita)
            
    elif (centro[0] < centro_robo[0]):
        velocidade_saida.publish(esquerda)

    return None
    
def choca_creep():
    limiar = 50
    velocidade_saida.publish(zero)

    if media_creep is None:
        velocidade_saida.publish(esquerda)
        return None
    
    if len(media_creep) == 0 or len(centro_robo) == 0:
        velocidade_saida.publish(esquerda)
        return None

    if (media_creep[0] < centro_robo[0]+limiar) and (media_creep[0] > centro_robo[0]-limiar):
        velocidade_saida.publish(frente)
            
    elif (media_creep[0] > centro_robo[0]):
        velocidade_saida.publish(direita)
            
    elif (media_creep[0] < centro_robo[0]):
        velocidade_saida.publish(esquerda)

    return None

if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)


    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    try:
        while not rospy.is_shutdown():
            choca_creep()
            # segue_linha()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


