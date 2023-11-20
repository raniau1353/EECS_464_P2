import sys
import csv
import math
from FiveBar import FiveBar
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Kinematics:
    def __init__(self, linkage):
        self.linkage = linkage
        
    def inverse(self, END_EFFECTOR_X, END_EFFECTOR_Y, END_EFFECTOR_Z):
        # motor angles
        m1=m2=m3 = None
        
        # Rotation motor
        m3 = self.linkage.get_angle(END_EFFECTOR_Y, END_EFFECTOR_X, 0., 0.)
        
        # Linkage motors 
        m1, m2 = self.linkage.get_motor_angles(END_EFFECTOR_X/math.cos(math.radians(m3)), END_EFFECTOR_Z)
        
        print(f"m1: {math.degrees(m1)}\nm2: {math.degrees(m2)}\nm3: {math.degrees(m3)}")
        
        return m1, m2, m3
        
    def forward(self, MOTOR_ANGLE_1, MOTOR_ANGLE_2, MOTOR_ANGLE_3):
        r,ee_z = self.linkage.get_ee_coords(MOTOR_ANGLE_1, MOTOR_ANGLE_2)
        
        ee_x = math.cos(MOTOR_ANGLE_3)*r
        ee_y = math.sin(MOTOR_ANGLE_3)*r
        
        print(f"ee_x: {ee_x}\nee_y: {ee_y}\nee_z: {ee_z}")
        
        return ee_x, ee_y, ee_z
    
    def calibrate(self):
        points_list = []
        with open('calibration_data.csv') as cal_file:
            cal_angles = csv.reader(cal_file, delimiter=' ')
            for row in cal_angles:
                m1, m2, m3 = row
                x, y, z = self.forward(float(m1), float(m2), float(m3))
                points_list.append([x, y, z])
        points = np.array(points_list)
        
        # least square plane
        b = points[:, 2]
        A = np.c_[points[:, 0], points[:, 1], np.ones(len(points[:, 0]))]
        
        # plane coefficients Z = c0*X + c1*Y + c2
        C,_,_,_ = np.linalg.lstsq(A, b)
        
        return points, C


## DEBUG SCRIPT ##      
if __name__ == "__main__":
    linkage = FiveBar(45, 35, 30, 35, 40, 70, 20)
    model = Kinematics(linkage)
    
    raw, fit = model.calibrate()
    
    # plot raw data
    plt.figure()
    ax = plt.subplot(111, projection='3d')
    ax.scatter(raw, color='b')
    
    # plane
    X, Y = np.meshgrid(np.arange(ax.get_xlim[0], ax.get_xlim[1]), 
                       np.arange(ax.get_ylim[0], ax.get_ylim[1]))
    
    Z = X*fit[0] + Y*fit[1] + fit[2]
    
    
    # plot plane
    ax.plot_wireframe(X,Y,Z, color='k')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
