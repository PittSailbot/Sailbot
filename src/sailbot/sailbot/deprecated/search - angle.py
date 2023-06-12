import math
import random


def equ_2pnts(x1,y1,x2,y2):
    a = x1 - x2
    b = y1 - y2
    return str(round(b/a,3))+"(x "+ ("- " if (x1>=0) else "+ ") + str(round(abs(x1),3)) + " ) "+ ("+ " if (y1>=0) else "- ") + str(round(abs(y1),3))+ "    { "+( (str(round(x1,3))+"< x < "+str(round(x2,3))+" }") if (x1<x2) else (str(round(x2,3))+"< x < "+str(round(x1,3))+" }") )

"""
gpslat = random.randrange(-10,10)   #x
gpslong = random.randrange(-10,10)  #y

buoylat = random.randrange(-10,10)     #x
buoylong = random.randrange(-10,10)    #y
"""
gpslat = 1
gpslong = 4
buoylat = 3
buoylong = 1.5

radius = 1


#find angle of boat from center of radius
a = gpslat - buoylat
b = gpslong - buoylong
ang = math.atan(b/a)
ang *= 180/math.pi

if( a<0):
    ang += 180


tar_angs = [ang,ang+72,ang-72,ang-(72*3),ang-(72*2)]

tarx = [0] * 5
tary = [0] * 5

#print("ur mom2: ", ang, tar_angs[0], math.cos( tar_angs[0] ) )
desmos_str = ""
desmos_str=\
      ("desmos:\n"
      +"("+str(buoylat)+", "+str(buoylong)+")\n"    #coord of buoy center
      +"("+str(gpslat)+", "+str(gpslong)+")\n"  #coord of boat
      #+str(round(b/a,5))+"(x "+ ("- " if (gpslat>=0) else "+ ") + str(abs(gpslat)) + " ) "+ ("+ " if (gpslong>=0) else "- ") + str(abs(gpslong))+"\n"
      #making circle
      +"(x-h)^2 + (y-k)^2 = m^2\n"
      +"h="+str(buoylat)+"\n"
      +"k="+str(buoylong)+"\n"
      +"m="+str(radius)
      )
print(desmos_str)
for i in range(0,5):


        #print( tar_angs[i] * (math.pi/180), tarx[1] )
    tarx[i] =  buoylat  + radius*math.cos( tar_angs[i] * (math.pi/180) )
    tary[i] =  buoylong + radius*math.sin( tar_angs[i] * (math.pi/180) )

            #print("tarx[", i, "]: ", tarx[i])
            #print("tary[", i, "]: ", tary[i],"\n")
    print("("+ str(round(tarx[i],4)) +", "+ str(round(tary[i],4)) +")" )    #coords along circle

print("\n" + equ_2pnts(gpslat,gpslong, tarx[0],tary[0]) )
for i in range(0,4):
    print( equ_2pnts(tarx[i],tary[i], tarx[i+1],tary[i+1] ) )
