# -*- coding: utf-8 -*-
from math import cos, sin, acos, asin, tan, atan, atan2
import math

# Cinématique indirecte
# Auteurs : Nathan Tutard - Louis Douriez - Florian Kauder

def leg_ik( x ,y ,z ,L1 = 51,L2 = 63.7, L3 =93, alpha=None, beta=None):
    ## Calcul des coefficients d'applications au réel
    if alpha == None:
        alpha = asin(22.5/L2)
    if beta == None:
        beta = asin(8.2/L3)

    ## Calcul des variables "utiles"    
    Lproj = math.sqrt(x*x + y*y)
    
    d13 = Lproj-L1
    if (d13 < 0):
        d13 = 0
        print ("Destination trop proche")
        
    
    d = math.sqrt(z*z+d13*d13)
    if (d > L2+L3):
        d = L2+L3
        print ("Destination trop éloignée")
    ## Calcul de T1 : simple tangeante
    if x==0:
        if y>0 :
            T1=math.pi/2
        elif y==0:
            T1=0
        else:
            T1 = -math.pi/2
    else :
        T1 = atan2(y,x)

    ## Calcul de T2
    # Calcul de a : On prend en compte les cas extrêmes
    if d13==0:
        if z>0 :
                a=math.pi/2
        elif z== 0:
                a = 0
        else :
                a = -math.pi/2
    else :
        a = atan(z/d13)

    # Calcul de b via Al-kashi
    b = acos((L2*L2 + d*d - (L3*L3))/(2 * L2 * d))
    T2 = a + b

    ## Calcul de T3 = PI - e (et e calculé via Al-kashi)
    e = acos((L2*L2 + L3*L3 - d*d)/(2 * L2 * L3))
    T3 = math.pi - e
    
    return [math.degrees(T1),-math.degrees(T2+alpha),-math.degrees(T3-math.pi/2+alpha+beta)]
