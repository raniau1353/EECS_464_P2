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
from scipy.optimize import fsolve

# The home coordinates will be [0,0,0]
coord_home = [0,0,0]

len_lift= 2
link1 = 3
link2 = 6
link5 = 4
link4 = 5
link2_ext = 8
link3 = 5

#joint1 = 0
horizmotor = 0
theta1 = -60
theta4 = -40

'''
#calculate theta3 (see diagram)
A = 2*link3*link4*math.sin(math.radians(-theta4)) - 2*link1*link3*math.sin(math.radians(-theta1))
B = 2*link3*link5 - 2*link1*link3*math.cos(math.radians(-theta1)) + 2*link3*link4*math.cos(math.radians(-theta4))
C = link1**2 - link2**2 + link3**2 + link4**2 + link5**2 - 2*link1*link4*(math.sin(math.radians(-theta1)))*(math.sin(math.radians(-theta4)))
- 2*link1*link5*math.cos(math.radians(-theta1)) + 2*link4*link5*math.cos(math.radians(-theta4)) - 2*link1*link4*(math.cos(math.radians(-theta1)))*(math.cos(math.radians(-theta4)))

theta3 = 2*math.atan((A + math.sqrt(A**2 + B**2 - C**2))/(B - C))
theta3_deg = math.degrees(theta3)
print("theta3 = ", theta3_deg)
'''

#vertpassive1 = 0
#vertpassive2 = 0
#joint2 = 0
#joint3 = 0

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


def forwardKinematics(ang1, ang2, ang3):
    '''
    #calculate theta3 (see diagram)
    BD_x = link5 + link4 * np.cos(theta4 * np.pi/180) - link1 * np.cos(theta1* np.pi/180)
    BD_y = link4 * np.sin(theta4 * np.pi/180) - link1 * np.sin(theta1* np.pi/180)
    BD_len = np.sqrt(BD_x * BD_x + BD_y * BD_y)
    theta_BDF = np.arctan2(BD_x, BD_y)
    theta_BDC = np.arccos((link3*link3 + BD_len*BD_len - link2*link2)/(2*link3*BD_len))
    theta3 = 2*np.pi - np.pi/2 - theta_BDC - theta_BDF
    #print("theta3 = ", math.degrees(theta3))
    ang5 =  -math.degrees(theta3) - theta4
    #print(vertpassive2)

    theta2 = math.asin((link3*math.sin(theta3) + link4*math.sin(math.radians(-theta4)) - link1*math.sin(math.radians(-theta1)))/ link2)
    #print("theta2: ", theta2_deg)
    ang4 = -(math.degrees(theta2) + theta1)
    #print(vertpassive1)
    '''


    # point 1
    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,len_lift],
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
    trans = np.dot(np.matrix([[1,0,0,link1],
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
                        [0,0,1,len_lift],
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
    trans = np.dot(np.matrix([[1,0,0,link5],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    pointg[0] = trans[(0,0)]
    pointg[1] = trans[(1,0)]
    pointg[2] = trans[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,len_lift],
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
    trans = np.dot(np.matrix([[1,0,0,link4],
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

    trans = np.dot(np.matrix([[1,0,0,link5],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point2g[0],point2g[1],point2g[2],1]).transpose())
    point2g[0] = trans[(0,0)]
    point2g[1] = trans[(1,0)]
    point2g[2] = trans[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,len_lift],
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
    trans = np.dot(np.matrix([[1,0,0,link2],
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
    
    trans = np.dot(np.matrix([[1,0,0,link1],
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
                        [0,0,1,len_lift],
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
    trans = np.dot(np.matrix([[1,0,0,-link2_ext],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point3_ext[0] = trans[(0,0)]
    point3_ext[1] = trans[(1,0)]
    point3_ext[2] = trans[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,link2],
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
    
    trans = np.dot(np.matrix([[1,0,0,link1],
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
                        [0,0,1,len_lift],
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
    trans = np.dot(np.matrix([[1,0,0,link3],
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


    trans = np.dot(np.matrix([[1,0,0,link4],
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

    trans = np.dot(np.matrix([[1,0,0,link5],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3g[0],point3g[1],point3g[2],1]).transpose())
    point3g[0] = trans[(0,0)]
    point3g[1] = trans[(1,0)]
    point3g[2] = trans[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,len_lift],
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

    point1, point2, pointg, point2g, point3, point3_ext, point3g = forwardKinematics(horizmotor, theta1, theta4)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    #ax.plot([coord_home[0],pointg[0]],[coord_home[1],pointg[1]],[coord_home[2],pointg[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])
    ax.plot([point3[0],point3_ext[0]],[point3[1],point3_ext[1]],[point3[2],point3_ext[2]])
    ax.plot([point2g[0],point3g[0]],[point2g[1],point3g[1]],[point2g[2],point3g[2]])

    ax.plot([10,-10],[0,0],[0,0], color='red')
    ax.plot([0,0],[10,-10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[10,-10], color='green')

def VertMotor1(val = 0):
    global theta1
    theta1 = val
    ax.clear()
   
    point1, point2, pointg, point2g, point3, point3_ext, point3g = forwardKinematics(horizmotor, theta1, theta4)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    #ax.plot([coord_home[0],pointg[0]],[coord_home[1],pointg[1]],[coord_home[2],pointg[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])
    ax.plot([point3[0],point3_ext[0]],[point3[1],point3_ext[1]],[point3[2],point3_ext[2]])
    ax.plot([point2g[0],point3g[0]],[point2g[1],point3g[1]],[point2g[2],point3g[2]])

    ax.plot([10,-10],[0,0],[0,0], color='red')
    ax.plot([0,0],[10,-10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[10,-10], color='green')
    
def VertMotor2(val = 0):
    global theta4
    theta4 = val
    ax.clear()
    
    point1, point2, pointg, point2g, point3, point3_ext, point3g = forwardKinematics(horizmotor, theta1, theta4)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    #ax.plot([coord_home[0],pointg[0]],[coord_home[1],pointg[1]],[coord_home[2],pointg[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])
    ax.plot([point3[0],point3_ext[0]],[point3[1],point3_ext[1]],[point3[2],point3_ext[2]])
    ax.plot([point2g[0],point3g[0]],[point2g[1],point3g[1]],[point2g[2],point3g[2]])

    ax.plot([10,-10],[0,0],[0,0], color='red')
    ax.plot([0,0],[10,-10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[10,-10], color='green')

def main():

    axSlider = plt.axes([0.2, 0.1, 0.65, 0.03] )
    aySlider = plt.axes([0.2, 0.065, 0.65, 0.03] )
    azSlider = plt.axes([0.2, 0.03, 0.65, 0.03] )
    #ap1Slider = plt.axes([0.2, -0.01, 0.65, 0.03] )
    #ap2Slider = plt.axes([0.2, 0.15, 0.65, 0.03] )
    horiz = Slider(axSlider, 'horizplanemotor', -180.0, 180.0, valinit=0, valstep = 1)
    vert1 = Slider(aySlider, 'vertplanemotor1', -180.0, 180.0, valinit=0, valstep = 1)
    vert2 = Slider(azSlider, 'vertplanemotor2', -180.0, 180.0, valinit=0, valstep = 1)
    #vert_passive1 = Slider(ap1Slider, 'vertpassive1', -180.0, 180.0, valinit=0, valstep = 1)
    #vert_passive2 = Slider(ap2Slider, 'vertpassive2', -180.0, 180.0, valinit=0, valstep = 1)

    horiz.on_changed(HorizMotor)
    vert1.on_changed(VertMotor1)
    vert2.on_changed(VertMotor2)
    #vert_passive1.on_changed(VertPassive1)
    #vert_passive2.on_changed(VertPassive2)

    plt.show()

if __name__ == "__main__":
    main()
