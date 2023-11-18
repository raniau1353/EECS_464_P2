# In this file I'll try to implement a simple forward kinematics exaple and simulate it in a 3D plot
# 
# 
# 
# the coordinates are defined as [x, y, z]

import math
from ntpath import join
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

# The home coordinates will be [0,0,0]
coord_home = [0,0,0]

#len_lift= 2
#link1 = 3
#link2 = 6
#link5 = 4
#link4 = 5
#link2_ext = 8
#link3 = 5

len_coxa = 2 #len_lift
len_femur = 3 #link1
len_tibia = 6 #link2
len_ground = 4 #link5
len_3 = 5 #link4
len_3ext = 8 #link2_ext
len_3g = 5 #link3 

joint1 = 0
horizmotor = 0
vertmotor1 = 0
vertmotor2= 0
vertpassive1 = 0
vertpassive2 = 0
joint2 = 0
joint3 = 0

point1 = [0,0,0]
point2 = [0,0,0]
point3 = [0,0,0]
pointg = [0,0,0]
point2g = [0,0,0]
point3_ext = [0,0,0]
point3g = [0,0,0]


coord_end = [5, 5,-5]



fig = plt.figure(figsize=(7,7))

ax = fig.add_subplot(111, projection='3d')

axSlider = plt.axes([0.2, 0.1, 0.65, 0.03] )
aySlider = plt.axes([0.2, 0.065, 0.65, 0.03] )
azSlider = plt.axes([0.2, 0.03, 0.65, 0.03] )
ap1Slider = plt.axes([0.2, -0.01, 0.65, 0.03] )
ap2Slider = plt.axes([0.2, 0.15, 0.65, 0.03] )
horiz = Slider(axSlider, 'horizplanemotor', -180.0, 180.0, valinit=0, valstep = 1)
vert1 = Slider(aySlider, 'vertplanemotor1', -180.0, 180.0, valinit=0, valstep = 1)
vert2 = Slider(azSlider, 'vertplanemotor2', -180.0, 180.0, valinit=0, valstep = 1)
vert_passive1 = Slider(ap1Slider, 'vertpassive1', -180.0, 180.0, valinit=0, valstep = 1)
vert_passive2 = Slider(ap2Slider, 'vertpassive2', -180.0, 180.0, valinit=0, valstep = 1)



def forwardKinematics(ang1, ang2, ang3, ang4, ang5):
    # point 1
    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,len_coxa],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point1[0] = trans[(0,0)]
    point1[1] = trans[(1,0)]
    point1[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point1[0],point1[1],point1[2],1]).transpose())
    
    point1[0] = rot[(0,0)]
    point1[1] = rot[(1,0)]
    point1[2] = rot[(2,0)]


    # point 2
    trans = np.dot(np.matrix([[1,0,0,len_femur],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point2[0] = trans[(0,0)]
    point2[1] = trans[(1,0)]
    point2[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang2)),0,math.sin(math.radians(ang2)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang2)),0,math.cos(math.radians(ang2)),0],
                        [0,0,0,1]]),np.matrix([point2[0],point2[1],point2[2],1]).transpose())
    
    point2[0] = rot[(0,0)]
    point2[1] = rot[(1,0)]
    point2[2] = rot[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,len_coxa],
                        [0,0,0,1]]),np.matrix([point2[0],point2[1],point2[2],1]).transpose())
    point2[0] = trans[(0,0)]
    point2[1] = trans[(1,0)]
    point2[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point2[0],point2[1],point2[2],1]).transpose())
    
    point2[0] = rot[(0,0)]
    point2[1] = rot[(1,0)]
    point2[2] = rot[(2,0)]

    # point g
    trans = np.dot(np.matrix([[1,0,0,len_ground],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    pointg[0] = trans[(0,0)]
    pointg[1] = trans[(1,0)]
    pointg[2] = trans[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,len_coxa],
                        [0,0,0,1]]),np.matrix([pointg[0],pointg[1],pointg[2],1]).transpose())
    pointg[0] = trans[(0,0)]
    pointg[1] = trans[(1,0)]
    pointg[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([pointg[0],pointg[1],pointg[2],1]).transpose())
    
    pointg[0] = rot[(0,0)]
    pointg[1] = rot[(1,0)]
    pointg[2] = rot[(2,0)]

    # point 2g
    trans = np.dot(np.matrix([[1,0,0,len_3],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point2g[0] = trans[(0,0)]
    point2g[1] = trans[(1,0)]
    point2g[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang3)),0,math.sin(math.radians(ang3)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang3)),0, math.cos(math.radians(ang3)),0],
                        [0,0,0,1]]),np.matrix([point2g[0],point2g[1],point2g[2],1]).transpose())
    
    point2g[0] = rot[(0,0)]
    point2g[1] = rot[(1,0)]
    point2g[2] = rot[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,len_ground],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point2g[0],point2g[1],point2g[2],1]).transpose())
    point2g[0] = trans[(0,0)]
    point2g[1] = trans[(1,0)]
    point2g[2] = trans[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,len_coxa],
                        [0,0,0,1]]),np.matrix([point2g[0],point2g[1],point2g[2],1]).transpose())
    point2g[0] = trans[(0,0)]
    point2g[1] = trans[(1,0)]
    point2g[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point2g[0],point2g[1],point2g[2],1]).transpose())
    
    point2g[0] = rot[(0,0)]
    point2g[1] = rot[(1,0)]
    point2g[2] = rot[(2,0)]

    # point3 
    trans = np.dot(np.matrix([[1,0,0,len_tibia],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point3[0] = trans[(0,0)]
    point3[1] = trans[(1,0)]
    point3[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang4)),0,math.sin(math.radians(ang4)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang4)),0,math.cos(math.radians(ang4)),0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    
    point3[0] = rot[(0,0)]
    point3[1] = rot[(1,0)]
    point3[2] = rot[(2,0)]
    
    trans = np.dot(np.matrix([[1,0,0,len_femur],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    point3[0] = trans[(0,0)]
    point3[1] = trans[(1,0)]
    point3[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang2)),0,math.sin(math.radians(ang2)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang2)),0,math.cos(math.radians(ang2)),0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    
    point3[0] = rot[(0,0)]
    point3[1] = rot[(1,0)]
    point3[2] = rot[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,len_coxa],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    point3[0] = trans[(0,0)]
    point3[1] = trans[(1,0)]
    point3[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    
    point3[0] = rot[(0,0)]
    point3[1] = rot[(1,0)]
    point3[2] = rot[(2,0)]

    # point3_ext
    trans = np.dot(np.matrix([[1,0,0,-len_3ext],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point3_ext[0] = trans[(0,0)]
    point3_ext[1] = trans[(1,0)]
    point3_ext[2] = trans[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,len_tibia],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3_ext[0],point3_ext[1],point3_ext[2],1]).transpose())
    point3_ext[0] = trans[(0,0)]
    point3_ext[1] = trans[(1,0)]
    point3_ext[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang4)),0,math.sin(math.radians(ang4)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang4)),0,math.cos(math.radians(ang4)),0],
                        [0,0,0,1]]),np.matrix([point3_ext[0],point3_ext[1],point3_ext[2],1]).transpose())
    
    point3_ext[0] = rot[(0,0)]
    point3_ext[1] = rot[(1,0)]
    point3_ext[2] = rot[(2,0)]
    
    trans = np.dot(np.matrix([[1,0,0,len_femur],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3_ext[0],point3_ext[1],point3_ext[2],1]).transpose())
    point3_ext[0] = trans[(0,0)]
    point3_ext[1] = trans[(1,0)]
    point3_ext[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang2)),0,math.sin(math.radians(ang2)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang2)),0,math.cos(math.radians(ang2)),0],
                        [0,0,0,1]]),np.matrix([point3_ext[0],point3_ext[1],point3_ext[2],1]).transpose())
    
    point3_ext[0] = rot[(0,0)]
    point3_ext[1] = rot[(1,0)]
    point3_ext[2] = rot[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,len_coxa],
                        [0,0,0,1]]),np.matrix([point3_ext[0],point3_ext[1],point3_ext[2],1]).transpose())
    point3_ext[0] = trans[(0,0)]
    point3_ext[1] = trans[(1,0)]
    point3_ext[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3_ext[0],point3_ext[1],point3_ext[2],1]).transpose())
    
    point3_ext[0] = rot[(0,0)]
    point3_ext[1] = rot[(1,0)]
    point3_ext[2] = rot[(2,0)]

    # point 3g
    trans = np.dot(np.matrix([[1,0,0,len_3g],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point3g[0] = trans[(0,0)]
    point3g[1] = trans[(1,0)]
    point3g[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang5)),0,math.sin(math.radians(ang5)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang5)),0,math.cos(math.radians(ang5)),0],
                        [0,0,0,1]]),np.matrix([point3g[0],point3g[1],point3g[2],1]).transpose())
    
    point3g[0] = rot[(0,0)]
    point3g[1] = rot[(1,0)]
    point3g[2] = rot[(2,0)]


    trans = np.dot(np.matrix([[1,0,0,len_3],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3g[0],point3g[1],point3g[2],1]).transpose())
    point3g[0] = trans[(0,0)]
    point3g[1] = trans[(1,0)]
    point3g[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang3)),0,math.sin(math.radians(ang3)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang3)),0, math.cos(math.radians(ang3)),0],
                        [0,0,0,1]]),np.matrix([point3g[0],point3g[1],point3g[2],1]).transpose())
    
    point3g[0] = rot[(0,0)]
    point3g[1] = rot[(1,0)]
    point3g[2] = rot[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,len_ground],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3g[0],point3g[1],point3g[2],1]).transpose())
    point3g[0] = trans[(0,0)]
    point3g[1] = trans[(1,0)]
    point3g[2] = trans[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,len_coxa],
                        [0,0,0,1]]),np.matrix([point3g[0],point3g[1],point3g[2],1]).transpose())
    point3g[0] = trans[(0,0)]
    point3g[1] = trans[(1,0)]
    point3g[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3g[0],point3g[1],point3g[2],1]).transpose())
    
    point3g[0] = rot[(0,0)]
    point3g[1] = rot[(1,0)]
    point3g[2] = rot[(2,0)]

    '''
    # point 3
    trans = np.dot(np.matrix([[1,0,0,len_tibia],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point3[0] = trans[(0,0)]
    point3[1] = trans[(1,0)]
    point3[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang3)),0,math.sin(math.radians(ang3)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang3)),0,math.cos(math.radians(ang3)),0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    
    point3[0] = rot[(0,0)]
    point3[1] = rot[(1,0)]
    point3[2] = rot[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,len_femur],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    point3[0] = trans[(0,0)]
    point3[1] = trans[(1,0)]
    point3[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang2)),0,math.sin(math.radians(ang2)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang2)),0,math.cos(math.radians(ang2)),0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())

    point3[0] = rot[(0,0)]
    point3[1] = rot[(1,0)]
    point3[2] = rot[(2,0)] 
    
    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,len_coxa],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    point3[0] = trans[(0,0)]
    point3[1] = trans[(1,0)]
    point3[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    
    point3[0] = rot[(0,0)]
    point3[1] = rot[(1,0)]
    point3[2] = rot[(2,0)]
    '''

    return point1, point2, pointg, point2g, point3, point3_ext, point3g
    



def HorizMotor(val = 0):
    global horizmotor
    horizmotor = val
    ax.clear()

    point1, point2, pointg, point2g, point3, point3_ext, point3g = forwardKinematics(horizmotor, vertmotor1, vertmotor2, vertpassive1, vertpassive2)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    #ax.plot([coord_home[0],pointg[0]],[coord_home[1],pointg[1]],[coord_home[2],pointg[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])
    ax.plot([point3[0],point3_ext[0]],[point3[1],point3_ext[1]],[point3[2],point3_ext[2]])
    ax.plot([point2g[0],point3g[0]],[point2g[1],point3g[1]],[point2g[2],point3g[2]])

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')

def VertMotor1(val = 0):
    global vertmotor1
    vertmotor1 = val
    ax.clear()
   
    point1, point2, pointg, point2g, point3, point3_ext, point3g = forwardKinematics(horizmotor, vertmotor1,vertmotor2, vertpassive1, vertpassive2)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    #ax.plot([coord_home[0],pointg[0]],[coord_home[1],pointg[1]],[coord_home[2],pointg[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])
    ax.plot([point3[0],point3_ext[0]],[point3[1],point3_ext[1]],[point3[2],point3_ext[2]])
    ax.plot([point2g[0],point3g[0]],[point2g[1],point3g[1]],[point2g[2],point3g[2]])

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')
    
def VertMotor2(val = 0):
    global vertmotor2
    vertmotor2 = val
    ax.clear()
    
    point1, point2, pointg, point2g, point3, point3_ext, point3g = forwardKinematics(horizmotor, vertmotor1,vertmotor2, vertpassive1, vertpassive2)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    #ax.plot([coord_home[0],pointg[0]],[coord_home[1],pointg[1]],[coord_home[2],pointg[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])
    ax.plot([point3[0],point3_ext[0]],[point3[1],point3_ext[1]],[point3[2],point3_ext[2]])
    ax.plot([point2g[0],point3g[0]],[point2g[1],point3g[1]],[point2g[2],point3g[2]])

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')

def VertPassive1(val = 0):
    global vertpassive1
    vertpassive1 = val
    ax.clear()

    point1, point2, pointg, point2g, point3, point3_ext, point3g = forwardKinematics(horizmotor, vertmotor1,vertmotor2, vertpassive1, vertpassive2)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    #ax.plot([coord_home[0],pointg[0]],[coord_home[1],pointg[1]],[coord_home[2],pointg[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])
    ax.plot([point3[0],point3_ext[0]],[point3[1],point3_ext[1]],[point3[2],point3_ext[2]])
    ax.plot([point2g[0],point3g[0]],[point2g[1],point3g[1]],[point2g[2],point3g[2]])

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')

def VertPassive2(val = 0):
    global vertpassive2
    vertpassive2 = val
    ax.clear()

    point1, point2, pointg, point2g, point3, point3_ext, point3g = forwardKinematics(horizmotor, vertmotor1,vertmotor2, vertpassive1, vertpassive2)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    #ax.plot([coord_home[0],pointg[0]],[coord_home[1],pointg[1]],[coord_home[2],pointg[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])
    ax.plot([point3[0],point3_ext[0]],[point3[1],point3_ext[1]],[point3[2],point3_ext[2]])
    ax.plot([point2g[0],point3g[0]],[point2g[1],point3g[1]],[point2g[2],point3g[2]])

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')


horiz.on_changed(HorizMotor)
vert1.on_changed(VertMotor1)
vert2.on_changed(VertMotor2)
vert_passive1.on_changed(VertPassive1)
vert_passive2.on_changed(VertPassive2)

plt.show()