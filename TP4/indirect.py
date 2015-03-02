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
        z=-z

    d = math.sqrt(z*z+d13*d13)

    # if d13==0:
    #     if z>0 :
    #         a=math.pi/2
    #     elif z== 0:
    #         a = 0
    #     else :
    #         a = -math.pi/2
    # else :
    a = atan(z/d13)

    b = acos( (L2*L2 + d*d - (L3*L3) ) / ( 2 * L2 * d ) )
    bsecond = -b

    T2 = a + b + alpha
    T2second = a + bsecond + alpha

    e=acos ( (L2*L2 + L3*L3 - d*d ) / (2 * L2 * L3) )
    esecond = -e

    T3 = -(math.pi - e +alpha +beta)
    T3second = -(math.pi - esecond +alpha +beta)

    # T3 = -(T3 + alpha + beta)

    print "alpha :", math.degrees(alpha), "beta :", math.degrees(beta)
    print "d:", d, "Lproj: ", Lproj, "d13:", d13
    print 'a: ',math.degrees(a) , 'b :',math.degrees(b)
    print "Angles :", math.degrees(T1), math.degrees(T2), math.degrees(T3)
    print "Angles 2 :", math.degrees(T2second), math.degrees(T3second)
    T2 = a - b
    T2 = T2 + alpha
    T3 = math.pi + e
    T3 = -(T3 + alpha + beta )



    # T2 = a + math.pi + b
    # T2 = T2 + alpha
    # T3 = e
    # T3 = -(T3 + alpha + beta )

    # print 'a: ',math.degrees(a) , 'b :',math.degrees(b)
    # print "Angles 3 :", math.degrees(T1), math.degrees(T2), math.degrees(T3)

    # T2 = a + (math.pi - b)
    # T2 = T2 + alpha
    # T3 = - e
    # T3 = -(T3 + alpha + beta)

    # print 'a: ',math.degrees(a) , 'b :',math.degrees(b)
    # print "Angles 4 :", math.degrees(T1), math.degrees(T2), math.degrees(T3)


# print "## ",3,0,0
# leg_id(3,0,0,1,1,1,0,0)
# print "Devait être trouvé : 0 0 0"
# print "\n## ",0,3,0
# leg_id(0,3,0,1,1,1,0,0)
# print "Devait être trouvé : 90 0 0"
# print "\n## ",1,0,2
# leg_id(1,0,2,1,1,1,0,0)
# print "Devait être trouvé : 0 90 0"
# print "\n## ",0,0,-1
# leg_id(0,0,-2,1,1,1,0,0)
# print "Devait être trouvé : 0 90 0"
# print "\n## ", 2,0,1
# leg_id(2,0,1,1,1,1,0,0)
# print "Devait être trouvé : 0 0 90"
print "## ",118.79, 0, -115.14
leg_id(118.79, 0, -115.14)
print "Devait être trouvé : 0 0 0"
print "\n## ",0, 118.79, -115.14
leg_id(0, 118.79, -115.74)
print "Devait être trouvé : 90 0 0"
print "\n## ",-64.14, 0, -67.79
leg_id(-64.14, 0, -67.79)
print "Devait être trouvé : 0 90 0"
print "\n## ", 203.23, 0, -14.30
leg_id(203.23, 0, -14.30)
print "Devait être trouvé : 0 0 90"