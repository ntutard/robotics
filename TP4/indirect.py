# -*- coding: utf-8 -*-
from math import cos, sin, acos, asin, tan, atan
import math

def leg_id( x ,y ,z ,L1 = 51,L2 = 63.7, L3 =93, alpha=None, beta=None):

    if alpha == None:
        alpha = asin(22.5/L2)
    if beta == None:
        beta = asin(8.2/L3) - math.pi/2

    if x==0:
        if y>0 :
            T1=math.pi/2
        elif y==0:
            T1 = 0
        else :
            T1 = -math.pi/2
    else :
        T1 = atan(y/x)
    
    Lproj = math.sqrt(x*x + y*y)

    if x >= 0:
        d13 = Lproj-L1
    else:
        d13 = Lproj+L1
        z=abs(z)

    d = math.sqrt(z*z+d13*d13)

    if d13==0:
        if z>0 :
            a=math.pi/2
        elif z== 0:
            a = 0
        else :
            a = -math.pi/2
    else :
        a = atan(z/d13)

    b = acos( (L2*L2 + d*d - (L3*L3) ) / ( 2 * L2 * d ) )
    bsecond = -b

    T2 = a +b + alpha
    T2second =  a + bsecond + alpha

    e=acos ( (L2*L2 + L3*L3 - d*d ) / (2 * L2 * L3) )
    esecond = -e

    T3 = -(math.pi - e +alpha +beta)
    T3second = -(math.pi - esecond +alpha +beta)

    if(abs(a+alpha)>abs(b)):
        return [ math.degrees(T1), math.degrees(T2), math.degrees(T3)]
    else:
        return  [math.degrees(T1),math.degrees(T2second), math.degrees(T3second)]
    
   
