#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import tf
import math
import cv2
import cv2.aruco as aruco
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
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
import ConceitoC_states
import processaImg as procImg

print("EXECUTE ANTES da 1.a vez: ")
print("wget https://github.com/Insper/robot21.1/raw/main/projeto/ros_projeto/scripts/MobileNetSSD_deploy.caffemodel")
print("PARA TER OS PESOS DA REDE NEURAL")

import visao_module


bridge = CvBridge()

cv_image = None
media = []
centro_pista = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
centro_robo = ()
media_creep = []
maior_area = 0
estado = 1
ranges = []

area = 0.0 # Variavel com a area do maior contorno

goal1 = ("orange", 22, "dog")

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

def scaneou(dado):
    global ranges

    ranges = np.array(dado.ranges).round(decimals=2)

def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro_pista
    global resultados
    global centro_robo
    global media_creep
    global maior_area

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

        img_copy = cv_image.copy()
        img_copy_pista = cv_image.copy()
        img_copy_aruco = cv_image.copy()
        img_copy_cor = cv_image.copy()

        centro_pista, antolho, mask = procImg.procesa_imagem_pista(img_copy_pista)
        # cv2.imshow("antolho", antolho)
        cv2.imshow("mask", mask)

        media_creep, maior_area, frame =  cormodule.identifica_cor(img_copy_cor, goal1[0])

        aruco_imshow = procImg.aruco_read(img_copy_aruco)
        # cv2.imshow('frame', aruco_imshow)

        centro_robo = (img_copy.shape[1]//2, img_copy.shape[0]//2)

        if centro_pista is not None:
            crosshair(cv_image, centro_pista, 4, (0,0,255))

        cv2.imshow("cv_image", cv_image)
        cv2.imshow("cor", frame)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

estado2 = False
estado2_trava = False

def main():
    global ranges
    global estado2
    global estado2_trava

    try:
        middle_sensor_mean = (ranges[355] + ranges[5])/2
        print(middle_sensor_mean, ranges[0])
    except:
        pass

    if maior_area > 1300:
        estado2 = True
        try:
            if middle_sensor_mean <= 0.2:
                estado2_trava = True
        except:
            pass

        
    if estado2 == False or estado2_trava == True:
        maquina_estados[1](velocidade_saida, centro_pista, centro_robo)
    
    if estado2 == True and estado2_trava == False:
        maquina_estados[2](velocidade_saida, media_creep, centro_robo)
    

if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    maquina_estados = {
        1: ConceitoC_states.segue_linha,
        2: ConceitoC_states.choca_creep
    }

    try:
        while not rospy.is_shutdown():
            main()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
