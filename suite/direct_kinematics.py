# -*- coding: utf-8 -*-
from math import cos, sin, acos, asin
import math

## Cinématique Directe
## Auteurs : Nathan Tutard - Louis Douriez - Florian Kauder
## Date : 11/02/2015

def leg_dk(T1,T2,T3,L1 = 51,L2 = 63.7, L3 =93, a=None, b=None ):
    if a == None:
        a = asin(22.5/L2)
    if b == None:
        b = asin(8.2/L3)
        
    T1f=math.radians(T1)
    T2f=-math.radians(T2)-a
    T3f=math.radians(T3) + b - math.radians(90) + a
    return [cos(T1f) * (L1 + L2*cos(T2f) + L3*cos(T3f+T2f) ),
          sin(T1f) * (L1 + L2*cos(T2f) + L3*cos(T3f+T2f)),
          L2 * sin(T2f) + L3*sin(T3f+T2f)]
