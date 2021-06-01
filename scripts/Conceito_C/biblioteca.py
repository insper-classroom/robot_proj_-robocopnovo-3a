#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import statsmodels.api as sm
import cv2.aruco as aruco
import mobilenet_simples as mbn
    
def segmenta_linha_amarela(bgr):
    img_bgr = bgr.copy()
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    hsv1 = np.array([20, 100, 140], dtype=np.uint8)
    hsv2 = np.array([ 35, 255, 255], dtype=np.uint8)
    res = cv2.inRange(img_hsv, hsv1, hsv2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
    mask = cv2.morphologyEx( res, cv2.MORPH_OPEN, kernel )
    mask = cv2.morphologyEx( mask, cv2.MORPH_CLOSE, kernel )  
    return mask

def encontrar_contornos(mask):
    contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contornos

# Segue a linha amarela
def processa_imagem_pista(bgr):   
    bgr_copy = bgr.copy()
    bgr_copy = cv2.rectangle(bgr_copy, (0,0), ((bgr.shape[1]//2)-100, bgr.shape[0]) , (0,0,255), -1)
    bgr_copy = cv2.rectangle(bgr_copy, ((bgr.shape[1]//2) + 175,0), (bgr.shape[1], bgr.shape[0]) , (0,0,255), -1)
    mask = segmenta_linha_amarela(bgr_copy)
    centro_pista = center_of_mass(mask)
    return centro_pista, bgr_copy, mask

# Verifica o MobileNet
def processa_machine_learning(bgr): 
    bgr_copy = bgr.copy()
    bgr_copy = cv2.rectangle(bgr_copy, (0,(bgr.shape[0]//2)+50), (bgr.shape[1], bgr.shape[1]) , (0,0,0), -1)
    detectframe, results = mbn.detect(bgr_copy)
    return detectframe, results

# Escreve texto no topo
def texto(img, a, p, color=(0,0,0), font=cv2.FONT_HERSHEY_SIMPLEX, width=2, size=1 ):
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

def ajuste_linear_x_fy(mask):
    """Recebe uma imagem já limiarizada e faz um ajuste linear
        retorna coeficientes linear e angular da reta
        e equação é da forma
        y = coef_angular*x + coef_linear
    """ 
    pontos = np.where(mask==255)
    ximg = pontos[1]
    yimg = pontos[0]
    yimg_c = sm.add_constant(yimg)
    model = sm.OLS(ximg,yimg_c)
    results = model.fit()
    coef_angular = results.params[1] # Pegamos o beta 1
    coef_linear =  results.params[0] # Pegamso o beta 0
    return coef_angular, coef_linear

def ajuste_linear_grafico_x_fy(mask):
    """Faz um ajuste linear e devolve uma imagem rgb com aquele ajuste desenhado sobre uma imagem"""
    coef_angular, coef_linear = ajuste_linear_x_fy(mask)
    pontos = np.where(mask==255) # esta linha é pesada e ficou redundante
    ximg = pontos[1]
    yimg = pontos[0]
    y_bounds = np.array([min(yimg), max(yimg)])
    x_bounds = coef_angular*y_bounds + coef_linear
    x_int = x_bounds.astype(dtype=np.int64)
    y_int = y_bounds.astype(dtype=np.int64)
    #mask_rgb =  cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    cv2.line(mask, (x_int[0], y_int[0]), (x_int[1], y_int[1]), color=(0,0,255), thickness=11);    
    return mask, x_int, y_int    
   

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    x = int(x)
    y = int(y)
    cv2.line(img,(x - size,y),(x + size,y),color,2)
    cv2.line(img,(x,y - size),(x, y + size),color,2)

def desenhar_linha_entre_pontos(img, X, Y, color):
    """Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, e retornar uma imagem com uma linha entre os centros EM SEQUENCIA do mais proximo.
    """
    for i in range(len(X) - 1):
        cv2.line(img,(X[i],Y[i]),(X[i+1],Y[i+1]),(0, 0, 255), 4)
    
    return img

def calcular_angulo_com_vertical(img, lm):
    """Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, e estimar a melhor reta, utilizando o metodo preferir, que passa pelos centros. Retorne a imagem com a reta.
        
        Dica: cv2.line(img,ponto1,ponto2,color,2) desenha uma linha que passe entre os pontos, mesmo que ponto1 e ponto2 não pertençam a imagem.
    """
    m = lm[0]
    
    omega = math.atan(m)
    gama = abs(math.degrees(omega))
    angulo = 90 - gama
    
    return angulo

def center_of_mass(mask):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(mask)
    if M["m00"] == 0:
        return None
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return [int(cX), int(cY)]

def identifica_cor(frame, cor):
    '''
    Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
    '''

    # No OpenCV, o canal H vai de 0 até 179, logo cores similares ao 
    # vermelho puro (H=0) estão entre H=-8 e H=8. 
    # Precisamos dividir o inRange em duas partes para fazer a detecção 
    # do vermelho:
    # frame = cv2.flip(frame, -1) # flip 0: eixo x, 1: eixo y, -1: 2 eixos
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Primeira cor = menor, segunda = maior:
    cores = {"blue": [np.array([80, 50, 100]), np.array([105, 255, 255])],
            "green": [np.array([45, 50, 100]), np.array([70, 255, 255])],
            "orange": [np.array([0, 180, 180]), np.array([15, 255, 255])]
            }

    cor_menor = cores[cor][0]
    cor_maior = cores[cor][1]
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

    # Note que a notacão do numpy encara as imagens como matriz, portanto o enderecamento é
    # linha, coluna ou (y,x)
    # Por isso na hora de montar a tupla com o centro precisamos inverter, porque 


    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (int( point[0] - length/2 ), point[1] ),  (int( point[0] + length/2 ), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], int(point[1] - length/2) ), (point[0], int( point[1] + length/2 ) ),color ,width, length) 



    # A operação MORPH_CLOSE fecha todos os buracos na máscara menores 
    # que um quadrado 7x7. É muito útil para juntar vários 
    # pequenos contornos muito próximos em um só.
    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

    # Encontramos os contornos na máscara e selecionamos o de maior área
    #contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)	
    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    maior_contorno = None
    maior_contorno_area = 0

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if (area > maior_contorno_area) and (area>50):
            maior_contorno = cnt
            maior_contorno_area = area

    # Encontramos o centro do contorno fazendo a média de todos seus pontos.
    if not maior_contorno is None :
        cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
        # cross(frame, centro, [255,0,0], 1, 17)
    else:
        media = (0, 0)

    # Representa a area e o centro do maior contorno no frame
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    cv2.putText(frame,"{:d} {:d}".format(*media),(20,100), 1, 4,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame,"{:0.1f}".format(maior_contorno_area),(20,50), 1, 4,(255,255,255),2,cv2.LINE_AA)


    return media, maior_contorno_area, frame