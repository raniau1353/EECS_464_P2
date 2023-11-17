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

# The home coordinates will be [0,0,0]
coord_home = [0,0,0]

len_coxa = 2
len_femur = 3
len_tibia = 3
len_ground = 4
len_3 = 4

joint1 = 0
horizmotor = 0
vertmotor1 = 0
joint2 = 0
joint3 = 0

point1 = [0,0,0]
point2 = [0,0,0]
point3 = [0,0,0]
pointg = [0,0,0]
point2g = [0,0,0]


coord_end = [5, 5,-5]

fig = plt.figure(figsize=(7,7))

ax = fig.add_subplot(111, projection='3d')

# Set initial plot dimensions
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([-10, 10])

axSlider = plt.axes([0.2, 0.1, 0.65, 0.03] )
aySlider = plt.axes([0.2, 0.065, 0.65, 0.03] )
azSlider = plt.axes([0.2, 0.03, 0.65, 0.03] )
horiz = Slider(axSlider, 'horizplanemotor', -180.0, 180.0, valinit=0, valstep = 1)
vert1 = Slider(aySlider, 'vertplanemotor1', -180.0, 180.0, valinit=0, valstep = 1)
#sliderZ = Slider(azSlider, 'Z', -180.0, 180.0, valinit=0, valstep = 1)

# Create line objects for the arms
line_arm1, = ax.plot([], [], [], color='blue', label='Arm 1')
line_arm2, = ax.plot([], [], [], color='green', label='Arm 2')
line_arm3, = ax.plot([], [], [], color='red', label='Arm 3')

# code for drawing
button_ax = plt.axes([0.3, 0.9, 0.1, 0.05])
draw_button = Button(button_ax, 'Draw')
reset_draw_button = Button(plt.axes([0.6, 0.9, 0.2, 0.05]), 'Stop Drawing')
points = []
drawing = False

def forwardKinematics(ang1, ang2):
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

    rot = np.dot(np.matrix([[math.cos(math.radians(ang2)),0,math.sin(math.radians(ang2)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang2)),0,math.cos(math.radians(ang2)),0],
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

    return point1, point2, pointg, point2g
    



def HorizMotor(val = 0):
    global horizmotor
    global points
    horizmotor = val
    # ax.clear()

    point1, point2, pointg, point2g = forwardKinematics(horizmotor,vertmotor1)

    line_arm1.set_data([coord_home[0], point1[0]], [coord_home[1], point1[1]])
    line_arm1.set_3d_properties([coord_home[2], point1[2]])

    line_arm2.set_data([point1[0], point2[0]], [point1[1], point2[1]])
    line_arm2.set_3d_properties([point1[2], point2[2]])

    line_arm3.set_data([point1[0], pointg[0], point2g[0]], [point1[1], pointg[1], point2g[1]])
    line_arm3.set_3d_properties([point1[2], pointg[2], point2g[2]])

    # ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    # #ax.plot([coord_home[0],pointg[0]],[coord_home[1],pointg[1]],[coord_home[2],pointg[2]])
    # ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    # ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    # ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    # #ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])

    # ax.plot([-10,10],[0,0],[0,0], color='red')
    # ax.plot([0,0],[-10,10],[0,0], color='blue')
    # ax.plot([0,0],[0,0],[-10,10], color='green')

    if drawing:
        points.append(point2g)
        points_array = np.array(points).T
        ax.scatter(points_array[0], points_array[1], points_array[2], c='black', marker='o')

def VertMotor1(val = 0):
    global vertmotor1
    global points
    vertmotor1 = val
    # ax.clear()

    point1, point2, pointg, point2g = forwardKinematics(horizmotor,vertmotor1)

    line_arm1.set_data([coord_home[0], point1[0]], [coord_home[1], point1[1]])
    line_arm1.set_3d_properties([coord_home[2], point1[2]])

    line_arm2.set_data([point1[0], point2[0]], [point1[1], point2[1]])
    line_arm2.set_3d_properties([point1[2], point2[2]])

    line_arm3.set_data([point1[0], pointg[0], point2g[0]], [point1[1], pointg[1], point2g[1]])
    line_arm3.set_3d_properties([point1[2], pointg[2], point2g[2]])

    # ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    # #ax.plot([coord_home[0],pointg[0]],[coord_home[1],pointg[1]],[coord_home[2],pointg[2]])
    # ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    # ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    # ax.plot([pointg[0],point2g[0]],[pointg[1],point2g[1]],[pointg[2],point2g[2]])
    # #ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])

    # ax.plot([-10,10],[0,0],[0,0], color='red')
    # ax.plot([0,0],[-10,10],[0,0], color='blue')
    # ax.plot([0,0],[0,0],[-10,10], color='green')

    # if points:
    if drawing:
        points.append(point2g)
        points_array = np.array(points).T
        ax.scatter(points_array[0], points_array[1], points_array[2], c='black', marker='o')
    # plot_points()
'''
def plotUpdateZ(val = 0):
    global joint3
    joint3 = val
    ax.clear()

    point1, point2, pointg = forwardKinematics(horizmotor,vertmotor1)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    #ax.plot([coord_home[0],pointg[0]],[coord_home[1],pointg[1]],[coord_home[2],pointg[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point1[0],pointg[0]],[point1[1],pointg[1]],[point1[2],pointg[2]])
    #ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')
'''

def draw(event):
    global drawing
    drawing = True

def stop_draw(event):
    global drawing
    drawing = False


horiz.on_changed(HorizMotor)
vert1.on_changed(VertMotor1)
#sliderZ.on_changed(plotUpdateZ)
draw_button.on_clicked(draw)
reset_draw_button.on_clicked(stop_draw)
plt.show()
