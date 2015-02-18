# -*- coding: utf-8 -*-
from math import cos, sin, acos, asin, tan, atan
import math

##L1=51
##L2=63.7
##L3=93
##
##
##x=L1+L2+L3
##y=0
##z=0
##
##
##T1 = atan(y/x)
##Lproj = math.sqrt(x*x + y*y)
##d13 = Lproj-L1
##
##a = atan(z/d13)
##d = math.sqrt(z*z+d13*d13)
##print 'd: ',d, 'd13 :',d13
##print '(L2*L2 - L3*L3 + d*d )' , (L2*L2 - L3*L3 + d*d )
##print '( 2 * L2 * d )',( 2 * L2 * d )
##b = acos( (L2*L2 + d*d - (L3*L3) ) / ( 2 * L2 * d ) )
##T2 = a + b
##
##
##e=acos ( (L2*L2 + L3*L3 - d*d ) / (2 * L2 * L3) )
##T3 = math.pi - e 
##print 'T1:', math.degrees(T1), ' T2 :', math.degrees(T2), 'T3 :', math.degrees(T3)

def leg_id( x ,y ,z ,L1 = 51,L2 = 63.7, L3 =93, alpha=None, beta=None):
    if alpha == None:
        alpha = asin(22.5/L2)
    if beta == None:
        beta = asin(8.2/L3)


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
    print Lproj
    d13 = Lproj-L1
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
    T2 = a + b

    e=acos ( (L2*L2 + L3*L3 - d*d ) / (2 * L2 * L3) )
    T3 = math.pi - e
    print 'a: ',math.degrees(a) , 'b :',math.degrees(b)
    return [math.degrees(T1),math.degrees(T2)+20.69,-(math.degrees(T3)-90+20.69+5.06)]
