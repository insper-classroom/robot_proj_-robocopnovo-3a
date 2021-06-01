#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import math
from simple_pid import PID
from geometry_msgs.msg import Twist, Vector3
import rospy

v = 0.2
w = math.pi/20.0

zero = Twist(Vector3(0,0,0), Vector3(0,0,0.0))
frente = Twist(Vector3(v,0,0), Vector3(0,0,0.0))
frentehalf = Twist(Vector3(v/2,0,0), Vector3(0,0,0.0))
direita = Twist(Vector3(0,0,0), Vector3(0,0, -w ))
esquerda = Twist(Vector3(0,0,0), Vector3(0,0,w))

pid = PID(0.7, 0.05, 0.1, setpoint=0)
pid.proportional_on_measurement = True
pid.output_limits = (-50, 50) 

def segue_linha(velocidade_saida, centro_pista, centro_robo,init=False):

    limiar = 70
    velocidade_saida.publish(zero)

    if centro_pista is None and init == False:
        velocidade_saida.publish(esquerda)
        return None

    if centro_pista is None and init == True:
        velocidade_saida.publish(direita)
        return None    
    
    if len(centro_pista) == 0 or len(centro_robo) == 0:
        velocidade_saida.publish(esquerda)
        return None

    error = centro_pista[0] - centro_robo[0]
    output = pid(error) 

    if (centro_pista[0] < centro_robo[0]+limiar) and (centro_pista[0] > centro_robo[0]-limiar):
        velocidade_saida.publish(frente)
            
    else:
        saida = output*w/100
        velocidade_saida.publish(Twist(Vector3(0,0,0), Vector3(0,0,saida)))

    return None


def choca_creep(velocidade_saida, media_creep, centro_robo, middle_sensor_mean,slow=False, init=True):

    limiar = 40
    velocidade_saida.publish(zero)

    if media_creep is None:
        velocidade_saida.publish(esquerda)
        return None
    
    if len(media_creep) == 0 or len(centro_robo) == 0:
        velocidade_saida.publish(esquerda)
        return None

    if (media_creep[0] < centro_robo[0]+limiar) and (media_creep[0] > centro_robo[0]-limiar) and slow == False:
        velocidade_saida.publish(Twist(Vector3(v,0,0), Vector3(0,0,0.0)))
        return None 
    
    elif middle_sensor_mean < 0.55:
        velocidade_saida.publish(frentehalf)
        return None 

    elif (media_creep[0] < centro_robo[0]+limiar) and (media_creep[0] > centro_robo[0]-limiar) and slow == True:
        velocidade_saida.publish(frentehalf)
            
    elif (media_creep[0] > centro_robo[0]):
        velocidade_saida.publish(direita)
            
    elif (media_creep[0] < centro_robo[0]):
        velocidade_saida.publish(esquerda)

    return None    

    

