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
import visao_module

v = 0.125
w = math.pi/20.0

zero = Twist(Vector3(0,0,0), Vector3(0,0,0.0))
frente = Twist(Vector3(v,0,0), Vector3(0,0,0.0))
direita = Twist(Vector3(0,0,0), Vector3(0,0, -w ))
esquerda = Twist(Vector3(0,0,0), Vector3(0,0,w))


def segue_linha(velocidade_saida, centro_pista, centro_robo):

    limiar = 40
    velocidade_saida.publish(zero)

    if centro_pista is None:
        velocidade_saida.publish(esquerda)
        return None
    
    if len(centro_pista) == 0 or len(centro_robo) == 0:
        velocidade_saida.publish(esquerda)
        return None

    if (centro_pista[0] < centro_robo[0]+limiar) and (centro_pista[0] > centro_robo[0]-limiar):
        velocidade_saida.publish(frente)
            
    elif (centro_pista[0] > centro_robo[0]):
        velocidade_saida.publish(direita)
            
    elif (centro_pista[0] < centro_robo[0]):
        velocidade_saida.publish(esquerda)

    return None

def choca_creep(velocidade_saida, media_creep, centro_robo):
    frente_choca = Twist(Vector3(0.1,0,0), Vector3(0,0,0.0))

    limiar = 50
    velocidade_saida.publish(zero)

    if media_creep is None:
        velocidade_saida.publish(esquerda)
        return None
    
    if len(media_creep) == 0 or len(centro_robo) == 0:
        velocidade_saida.publish(esquerda)
        return None

    if (media_creep[0] < centro_robo[0]+limiar) and (media_creep[0] > centro_robo[0]-limiar):
        velocidade_saida.publish(frente_choca)
            
    elif (media_creep[0] > centro_robo[0]):
        velocidade_saida.publish(direita)
            
    elif (media_creep[0] < centro_robo[0]):
        velocidade_saida.publish(esquerda)

    return None

