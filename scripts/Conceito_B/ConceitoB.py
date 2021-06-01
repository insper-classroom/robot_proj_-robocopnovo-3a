#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import math
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3
import biblioteca as bibliot
import ConceitoC_states

# Parâmetros de Início
bridge = CvBridge()
check_delay = False 
cv_image = None
media = []
centro_pista = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
centro_robo = ()
media_creep = []
maior_area = 0
id_check = False
estado = 1
ranges = []
area = 0.0 # Variavel com a area do maior contorno
x = 0
y = 0
z = 0 
id = 0
font = cv2.FONT_HERSHEY_SIMPLEX
frame = "camera_link"
tfl = 0
tf_buffer = tf2_ros.Buffer()
DIREITA = True
estado2 = False
estado2_trava = False
init = True
init2 = True
resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

# Parâmetros do ROS de Publicação
v = 0.1
w = math.pi/20.0
zero = Twist(Vector3(0,0,0), Vector3(0,0,0.0))
frente = Twist(Vector3(v,0,0), Vector3(0,0,0.0))
frentehalf = Twist(Vector3(v/2,0,0), Vector3(0,0,0.0))
direita = Twist(Vector3(0,0,0), Vector3(0,0, -w ))
esquerda = Twist(Vector3(0,0,0), Vector3(0,0,w))

#Objetivo de Busca
goal1 = ("orange", 11, "dog")

#Funções

# Segue a linha amarela
def processa_imagem_pista(bgr):   
    bgr_copy = bgr.copy()
    bgr_copy = cv2.rectangle(bgr_copy, (0,0), ((bgr.shape[1]//2)-100, bgr.shape[0]) , (0,0,255), -1)
    bgr_copy = cv2.rectangle(bgr_copy, ((bgr.shape[1]//2) + 175,0), (bgr.shape[1], bgr.shape[0]) , (0,0,255), -1)
    mask = bibliot.segmenta_linha_amarela(bgr_copy)
    centro_pista = bibliot.center_of_mass(mask)
    return centro_pista, bgr_copy, mask

# Escreve texto no topo
def texto(img, a, p, color=(255,255,255), font=font, width=2, size=1 ):
    """Escreve na img RGB dada a string a na posição definida pela tupla p"""
    cv2.putText(img, str(a), p, font,size,color,width,cv2.LINE_AA)

# Verifica leituras do Laser Scan
def scaneou(dado):
    global ranges
    ranges = np.array(dado.ranges).round(decimals=2)

# Faz as leituras do Aruco 
def aruco_read(img_copy):
    aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict) 
    aruco.drawDetectedMarkers(img_copy, corners, ids)
    return corners, ids 

# Processa Imagens
def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro_pista
    global resultados
    global centro_robo
    global media_creep
    global maior_area
    global id_check

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

        img_copy = cv_image.copy()
        img_copy_pista = cv_image.copy()
        img_copy_aruco = cv_image.copy()
        img_copy_cor = cv_image.copy()

        centro_pista, antolho, mask = processa_imagem_pista(img_copy_pista)
        # cv2.imshow("antolho", antolho)
        #cv2.imshow("mask", mask)

        media_creep, maior_area, frame = bibliot.identifica_cor(img_copy_cor, goal1[0])

        # quando nao tiver ids = None, corners segue a ordem de ids pelo index
        corners, ids = aruco_read(img_copy_aruco)
        if ids is not None:
            for i in ids:
                if goal1[1] == i:
                    print("True")
                    id_check = True
        #cv2.imshow('frame', img_copy_aruco)

        centro_robo = (img_copy.shape[1]//2, img_copy.shape[0]//2)

        if centro_pista is not None:
            bibliot.crosshair(cv_image, centro_pista, 4, (0,0,255))
        if estado2 == False or estado2_trava == True:
            texto(cv_image, "Estado:", (270, 50) )
            texto(cv_image, "Andando na pista", (190, 100) )
        if estado2 == True and estado2_trava == False:
            texto(cv_image, "Estado:", (270, 50) )
            texto(cv_image, "Captura do creeper", (140, 100) )    
        cv2.imshow("cv_image", cv_image)
        #cv2.imshow("cor", frame)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

# Função principal
def main():
    global ranges
    global estado2
    global estado2_trava
    global init
    global init2

    try:
        middle_sensor_mean = (ranges[355] + ranges[5])/2
        print(middle_sensor_mean, ranges[0])
    except:
        pass

    if maior_area > 1300 and (id_check == True):
        estado2 = True
        try:
            if middle_sensor_mean < 1.2 and init == True:
                if init == True:
                    rospy.sleep(0.5)
                    ombro.publish(0.0) ## para frente
                    rospy.sleep(0.5)
                    garra.publish(-1.0) ## Aberto
                    rospy.sleep(0.5)
                    init = False      
            if middle_sensor_mean <= 0.5 and init2 == True and init==False:
                velocidade_saida.publish(frentehalf)
                rospy.sleep(0.3)
                velocidade_saida.publish(zero)
                rospy.sleep(0.3)
            if middle_sensor_mean <= 0.2 and init2 == True and init==False:
                garra.publish(0.0)  ## Fechado
                rospy.sleep(0.5)
                ombro.publish(1.5) ## para cima
                rospy.sleep(0.5)
                init2 = False
                estado2_trava = True
        except:
            pass

    if estado2 == False or estado2_trava == True:
        maquina_estados[1](velocidade_saida, centro_pista, centro_robo)
    
    if estado2 == True and estado2_trava == False:
        maquina_estados[2](velocidade_saida, media_creep, centro_robo)
    
# Mainloop
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
    garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)
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
