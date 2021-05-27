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
import mobilenet_simples as mbn
import biblioteca as bibliot
import ConceitoB_states

# Parâmetros de Início
bridge = CvBridge()
check_delay = False 
cv_image = None
media = []
centro_pista = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
centro_robo = ()
media_creep = []
media_figura = ()
maior_area = 0
id_check = False
estado = 1
ranges = []
area = 0.0 # Variavel com a area do maior contorno
x = 0
y = 0
z = 0 
id = 0
count = 0 
font = cv2.FONT_HERSHEY_SIMPLEX
frame = "camera_link"
tfl = 0
tf_buffer = tf2_ros.Buffer()
DIREITA = True
estado2 = False
estado2_trava = False
estado3 = False
estado3_trava = False
estado4 = False
estado4_trava = False
init = True
lock = False
slow = False
calib_path  = "/home/borg/catkin_ws/src/robot202/ros/exemplos202/scripts/"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')
tvec = []
marker_size  = 20
middle_sensor_mean = 0
resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

# Parâmetros do ROS de Publicação
v = 0.1
w = math.pi/20.0
zero = Twist(Vector3(0,0,0), Vector3(0,0,0.0))
frente = Twist(Vector3(v,0,0), Vector3(0,0,0.0))
frentehalf = Twist(Vector3(v/3,0,0), Vector3(0,0,0.0))
direita = Twist(Vector3(0,0,0), Vector3(0,0, -w ))
esquerda = Twist(Vector3(0,0,0), Vector3(0,0,w))
tras = Twist(Vector3(-0.15, 0, 0), Vector3(0, 0, 0))

#Objetivo de Busca
goal1 = ("orange", 11, "cow")

#Funções

# Segue a linha amarela
def processa_imagem_pista(bgr):   
    bgr_copy = bgr.copy()
    bgr_copy = cv2.rectangle(bgr_copy, (0,0), ((bgr.shape[1]//2)-100, bgr.shape[0]) , (0,0,255), -1)
    bgr_copy = cv2.rectangle(bgr_copy, ((bgr.shape[1]//2) + 175,0), (bgr.shape[1], bgr.shape[0]) , (0,0,255), -1)
    mask = bibliot.segmenta_linha_amarela(bgr_copy)
    #mask_linha, x_bounds, y_bounds = bibliot.ajuste_linear_grafico_x_fy(mask)
    #centro_pista = (x_bounds[1],y_bounds[1])
    centro_pista = bibliot.center_of_mass(mask)
    return centro_pista, bgr_copy, mask

# Verifica o MobileNet
def processa_machine_learning(bgr): 
    bgr_copy = bgr.copy()
    bgr_copy = cv2.rectangle(bgr_copy, (0,(bgr.shape[0]//2)+50), (bgr.shape[1], bgr.shape[1]) , (0,0,0), -1)
    detectframe, results = mbn.detect(bgr_copy)
    return detectframe, results

# Escreve texto no topo
def texto(img, a, p, color=(0,0,0), font=font, width=2, size=1 ):
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
    global media_figura
    global maior_area
    global id_check
    global count
    global calib_path  
    global camera_matrix   
    global camera_distortion  
    global tvec

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

        img_copy = cv_image.copy()
        img_copy_pista = cv_image.copy()
        img_copy_aruco = cv_image.copy()
        img_copy_cor = cv_image.copy()
        img_copy_mobilenet = cv_image.copy()

        centro_pista, antolho, mask = processa_imagem_pista(img_copy_pista)

        media_creep, maior_area, frame = bibliot.identifica_cor(img_copy_cor, goal1[0])

        corners, ids = aruco_read(img_copy_aruco)
        if ids is not None:
            for i in ids:
                if goal1[1] == i:
                    ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                    id_check = True

        detectframe, results = processa_machine_learning(img_copy_mobilenet)
        if results != []:
            if goal1[2] == results[0][0] and float(results[0][1]) > 90:
                count+=1
            else:
                count= 0      
            media_figura = (int((results[0][3][0]+results[0][2][0])/2),int((results[0][3][1]+results[0][2][1])/2))
            bibliot.crosshair(detectframe, media_figura, 4, (0,0,255))
        cv2.imshow('mobilenet', detectframe)

        centro_robo = (img_copy.shape[1]//2, img_copy.shape[0]//2)

        if centro_pista is not None:
            bibliot.crosshair(img_copy_aruco, centro_pista, 4, (0,0,255))
        texto(img_copy_aruco, "Estado:", (50, 50) )
        texto(img_copy_aruco, "Laser Scan:", (400, 50) )
        texto(img_copy_aruco, "{:2f}".format(middle_sensor_mean), (400, 100) )
        if estado2 == True and estado2_trava == False and estado3_trava == False:
            texto(img_copy_aruco, "Captura do creeper", (50, 100) ) 
        elif estado3 == True and estado3_trava == False and estado2_trava == True:
            texto(img_copy_aruco, "Indo ao objetivo", (50, 100) ) 
        else:
            texto(img_copy_aruco, "Andando na pista", (50, 100) ) 
        #cv2.line(img_copy_aruco, (x_box[0], y_box[0]), (x_box[1], y_box[1]), color=(0,0,255), thickness=11);                
        cv2.imshow("Main", img_copy_aruco)
        #cv2.imshow("cor", frame)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

# Função principal
def main():
    global ranges
    global estado2
    global estado2_trava
    global estado3
    global estado3_trava
    global estado4
    global estado4_trava
    global init
    global middle_sensor_mean
    global slow

    try:
        middle_sensor_mean = (ranges[358] + ranges[2])/2
    except:
        pass

    if maior_area > 900 and (id_check == True) and estado2_trava == False:
        estado2 = True
        try:
            if middle_sensor_mean < 1.2 and init == True:
                ombro.publish(0.0) ## para frente
                garra.publish(-1.0) ## Aberto
                init = False
            if middle_sensor_mean <= 0.185 and init==False:
                garra.publish(0.0)  ## Fechado
                rospy.sleep(0.5)
                ombro.publish(1.5) ## para cima
                rospy.sleep(0.5)
                estado2_trava = True
                slow = False          
            elif middle_sensor_mean <= 0.25 and init==False:
                slow = True
        except:
            pass
    
    if count > 4 and estado3_trava == False:
        estado3 = True
        try:
            if middle_sensor_mean <= 0.9:
                estado4 = True
                estado3_trava = True
        except:
            pass
    
    if estado2 == True and slow == False and estado2_trava == False and estado3_trava == False and estado4_trava == False:
        maquina_estados[2](velocidade_saida, media_creep, centro_robo, middle_sensor_mean)
    elif estado2 == True and slow == True and estado2_trava == False and estado3_trava == False and estado4_trava == False: 
        maquina_estados[2](velocidade_saida, media_creep, centro_robo, middle_sensor_mean, slow)  
    elif estado3 == True and estado2_trava == True and estado3_trava == False and estado4_trava == False:
        maquina_estados[2](velocidade_saida, media_figura, centro_robo, middle_sensor_mean)
    elif estado4 == True and estado3_trava == True and estado2_trava == True and estado4_trava == False:
        velocidade_saida.publish(zero)
        rospy.sleep(1)
        ombro.publish(0.0) ## para frente
        rospy.sleep(0.5)
        garra.publish(-1.0) ## Aberto
        rospy.sleep(0.5)
        velocidade_saida.publish(tras)
        rospy.sleep(2)
        garra.publish(0.0)  ## Fechado
        rospy.sleep(0.5)
        ombro.publish(-1) ## para baixo
        rospy.sleep(0.5)
        velocidade_saida.publish(esquerda)
        rospy.sleep(2)
        estado4_trava = True        
    else:
        maquina_estados[1](velocidade_saida, centro_pista, centro_robo)    
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
        1: ConceitoB_states.segue_linha,
        2: ConceitoB_states.choca_creep,
    }

    try:
        while not rospy.is_shutdown():
            main()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
