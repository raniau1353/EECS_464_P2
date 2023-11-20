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
from matplotlib.widgets import Slider, Button
from scipy.optimize import fsolve
from itertools import product

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

vertpassive1 = 0
vertpassive2 = 0
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

points = []
drawing = False

# Create line objects for the arms
line_arm1, = ax.plot([], [], [], color='blue', label='Arm 1')
line_arm2, = ax.plot([], [], [], color='green', label='Arm 2')
line_arm3, = ax.plot([], [], [], color='red', label='Arm 3')
line_arm4, = ax.plot([], [], [], color='orange', label='Arm 4')
line_arm5, = ax.plot([], [], [], color='purple', label='Arm 5')
line_arm6, = ax.plot([], [], [], color='brown', label='Arm 6')


def forwardKinematics(ang1, ang2, ang3):
    
    #calculate theta3 (see diagram)
    BD_x = link5 + link4 * np.cos(-theta4 * np.pi/180) - link1 * np.cos(-theta1* np.pi/180)
    BD_y = link4 * np.sin(-theta4 * np.pi/180) - link1 * np.sin(-theta1* np.pi/180)
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
    # ax.clear()

    point1, point2, pointg, point2g, point3, point3_ext, point3g = forwardKinematics(horizmotor, theta1, theta4)

    line_arm1.set_data([coord_home[0], point1[0]], [coord_home[1], point1[1]])
    line_arm1.set_3d_properties([coord_home[2], point1[2]])

    line_arm2.set_data([point1[0], point2[0]], [point1[1], point2[1]])
    line_arm2.set_3d_properties([point1[2], point2[2]])

    line_arm3.set_data([point1[0], pointg[0], point2g[0]], [point1[1], pointg[1], point2g[1]])
    line_arm3.set_3d_properties([point1[2], pointg[2], point2g[2]])

    line_arm4.set_data([point2[0], point3[0]], [point2[1], point3[1]])
    line_arm4.set_3d_properties([point2[2], point3[2]])

    line_arm5.set_data([point3[0], point3_ext[0]], [point3[1], point3_ext[1]])
    line_arm5.set_3d_properties([point3[2], point3_ext[2]])

    line_arm6.set_data([point2g[0], point3g[0]], [point2g[1], point3g[1]])
    line_arm6.set_3d_properties([point2g[2], point3g[2]])

    # ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    # ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    # ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    # ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    # ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])
    # ax.plot([point3[0],point3_ext[0]],[point3[1],point3_ext[1]],[point3[2],point3_ext[2]])
    # ax.plot([point2g[0],point3g[0]],[point2g[1],point3g[1]],[point2g[2],point3g[2]])

    # ax.plot([10,-10],[0,0],[0,0], color='red')
    # ax.plot([0,0],[10,-10],[0,0], color='blue')
    # ax.plot([0,0],[0,0],[10,-10], color='green')

    if drawing:
        points.append(point3_ext)
        points_array = np.array(points).T
        ax.scatter(points_array[0], points_array[1], points_array[2], c='black', marker='o', s=2)

def VertMotor1(val = 0):
    global theta1
    theta1 = val
    # ax.clear()
   
    point1, point2, pointg, point2g, point3, point3_ext, point3g = forwardKinematics(horizmotor, theta1, theta4)

    line_arm1.set_data([coord_home[0], point1[0]], [coord_home[1], point1[1]])
    line_arm1.set_3d_properties([coord_home[2], point1[2]])

    line_arm2.set_data([point1[0], point2[0]], [point1[1], point2[1]])
    line_arm2.set_3d_properties([point1[2], point2[2]])

    line_arm3.set_data([point1[0], pointg[0], point2g[0]], [point1[1], pointg[1], point2g[1]])
    line_arm3.set_3d_properties([point1[2], pointg[2], point2g[2]])

    line_arm4.set_data([point2[0], point3[0]], [point2[1], point3[1]])
    line_arm4.set_3d_properties([point2[2], point3[2]])

    line_arm5.set_data([point3[0], point3_ext[0]], [point3[1], point3_ext[1]])
    line_arm5.set_3d_properties([point3[2], point3_ext[2]])

    line_arm6.set_data([point2g[0], point3g[0]], [point2g[1], point3g[1]])
    line_arm6.set_3d_properties([point2g[2], point3g[2]])

    # ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    # ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    # ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    # ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    # ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])
    # ax.plot([point3[0],point3_ext[0]],[point3[1],point3_ext[1]],[point3[2],point3_ext[2]])
    # ax.plot([point2g[0],point3g[0]],[point2g[1],point3g[1]],[point2g[2],point3g[2]])

    # ax.plot([10,-10],[0,0],[0,0], color='red')
    # ax.plot([0,0],[10,-10],[0,0], color='blue')
    # ax.plot([0,0],[0,0],[10,-10], color='green')

    if drawing:
        points.append(point3_ext)
        points_array = np.array(points).T
        ax.scatter(points_array[0], points_array[1], points_array[2], c='black', marker='o', s=2)
    
def VertMotor2(val = 0):
    global theta4
    theta4 = val
    # ax.clear()
    
    point1, point2, pointg, point2g, point3, point3_ext, point3g = forwardKinematics(horizmotor, theta1, theta4)

    line_arm1.set_data([coord_home[0], point1[0]], [coord_home[1], point1[1]])
    line_arm1.set_3d_properties([coord_home[2], point1[2]])

    line_arm2.set_data([point1[0], point2[0]], [point1[1], point2[1]])
    line_arm2.set_3d_properties([point1[2], point2[2]])

    line_arm3.set_data([point1[0], pointg[0], point2g[0]], [point1[1], pointg[1], point2g[1]])
    line_arm3.set_3d_properties([point1[2], pointg[2], point2g[2]])

    line_arm4.set_data([point2[0], point3[0]], [point2[1], point3[1]])
    line_arm4.set_3d_properties([point2[2], point3[2]])

    line_arm5.set_data([point3[0], point3_ext[0]], [point3[1], point3_ext[1]])
    line_arm5.set_3d_properties([point3[2], point3_ext[2]])

    line_arm6.set_data([point2g[0], point3g[0]], [point2g[1], point3g[1]])
    line_arm6.set_3d_properties([point2g[2], point3g[2]])

    # ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    # ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    # ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    # ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    # ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])
    # ax.plot([point3[0],point3_ext[0]],[point3[1],point3_ext[1]],[point3[2],point3_ext[2]])
    # ax.plot([point2g[0],point3g[0]],[point2g[1],point3g[1]],[point2g[2],point3g[2]])

    # ax.plot([10,-10],[0,0],[0,0], color='red')
    # ax.plot([0,0],[10,-10],[0,0], color='blue')
    # ax.plot([0,0],[0,0],[10,-10], color='green')

    if drawing:
        points.append(point3_ext)
        points_array = np.array(points).T
        ax.scatter(points_array[0], points_array[1], points_array[2], c='black', marker='o', s=2)

def Draw(event):
    global drawing
    drawing = True

def StopDraw(event):
    global drawing
    drawing = False

# plot cubic workspace
def plot_cubic_workspace():
    # Coordinates of the cube vertices
    cube_vertices = np.array(list(product([0, 33/10], repeat=3)))

    # Translate the cube
    translation_matrix = np.array([
        [1, 0, 0, -3.6],  # Translation along the x-axis
        [0, 1, 0, -1.65], # Translation along y-axis
        [0, 0, 1, 0], # Translation along z-axis
        [0, 0, 0, 1]
    ])
    cube_vertices = np.dot(cube_vertices, translation_matrix[:3, :3].T) + translation_matrix[:3, 3]

    # Define the edges of the cube (ex: vertex 0 -> 1)
    cube_edges = [
        (0, 1), (1, 3), (3, 2), (2, 0),
        (4, 5), (5, 7), (7, 6), (6, 4),
        (0, 4), (1, 5), (2, 6), (3, 7)
    ]

    # Plot the cube edges
    for edge in cube_edges:
        edge_coords = np.array([cube_vertices[edge[0]], cube_vertices[edge[1]]])
        ax.plot3D(edge_coords[:, 0], edge_coords[:, 1], edge_coords[:, 2], color='gray')

def main():

    axSlider = plt.axes([0.2, 0.1, 0.65, 0.03] )
    aySlider = plt.axes([0.2, 0.065, 0.65, 0.03] )
    azSlider = plt.axes([0.2, 0.03, 0.65, 0.03] )

    # Set initial plot dimensions
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_zlim([-10, 10])

    # Add axis labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plot_cubic_workspace()

    # code for drawing
    button_ax = plt.axes([0.3, 0.9, 0.1, 0.05])
    draw_button = Button(button_ax, 'Draw')
    reset_draw_button = Button(plt.axes([0.6, 0.9, 0.2, 0.05]), 'Stop Drawing')

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

    draw_button.on_clicked(Draw)
    reset_draw_button.on_clicked(StopDraw)

    plt.show()

if __name__ == "__main__":
    main()
