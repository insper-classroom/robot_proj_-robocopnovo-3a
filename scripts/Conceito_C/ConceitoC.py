#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from geometry_msgs.msg import Twist
import biblioteca as bibliot
import ConceitoC_states

# Parâmetros de Início
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
check_delay = False 
resultados = [] # Criacao de uma variavel global para guardar os resultados vistos
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

#Objetivo de Busca
goal1 = ("orange", 22, "dog") 

#Funções

# Escreve Textos na Tela
def texto(img, a, p, color=(0,255,255), font=font, width=2, size=1 ):
    cv2.putText(img, str(a), p, font,size,color,width,cv2.LINE_AA)

# Verifica leituras do Laser Scan
def scaneou(dado): 
    global ranges
    ranges = np.array(dado.ranges).round(decimals=2)

# Segue a linha amarela
def processa_imagem_pista(bgr):
    bgr_copy = bgr.copy()
    bgr_copy = cv2.rectangle(bgr_copy, (0,0), ((bgr.shape[1]//2)-100, bgr.shape[0]) , (0,0,255), -1)
    bgr_copy = cv2.rectangle(bgr_copy, ((bgr.shape[1]//2) + 175,0), (bgr.shape[1], bgr.shape[0]) , (0,0,255), -1)
    mask = bibliot.segmenta_linha_amarela(bgr_copy)
    centro_pista = bibliot.center_of_mass(mask)
    return centro_pista, bgr_copy, mask      

# Processa Imagens
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

        centro_pista, antolho, mask = processa_imagem_pista(img_copy_pista)
        cv2.imshow("mask", mask)
        media_creep, maior_area, frame =  bibliot.identifica_cor(img_copy_cor, goal1[0])
        centro_robo = (img_copy.shape[1]//2, img_copy.shape[0]//2)

        if centro_pista is not None:
            bibliot.crosshair(cv_image, centro_pista, 4, (0,0,255))

        if estado2 == False or estado2_trava == True:
            texto(cv_image, "Andando na pista", (340, 100) )
        if estado2 == True and estado2_trava == False:
            texto(cv_image, "Se chocando contra o creeper", (100, 100) )    
        cv2.imshow("cv_image", cv_image)
        cv2.imshow("cor", frame)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

# Função principal
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
    

# Mainloop
if __name__=="__main__":
    # Iniciando os Subscribers (Feed de câmera e LaserScan)
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
