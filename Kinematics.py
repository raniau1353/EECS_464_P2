import sys
import csv
import math
from Linkage import FiveBar
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Kinematics:
    # w_x and w_y are front bottom left corner of workspace
    def __init__(self, linkage, w_x, w_y,):
        self.linkage = linkage
        self.paper_z = None
        self.paper_x, self.paper_y = np.meshgrid(np.linspace(w_x, w_x+3.3, num=10), 
                                                 np.linspace(w_y, w_y+3.3, num=10))
        
    # COORDINATE SYSTEM: 
    # origin <0, 0, 0> is j0
    # +x is towards workspace
    # +y is left
    # +z is up
    
    # m1, m2: 
    #   0 deg is +z
    #   90 deg is +x
    #   -90 deg is -x
    
    # m3:
    #   0 deg is +x
    #   90 deg is -y
    #   -90 deg is +y
    
    ###########################
    ### KINEMATIC FUNCTIONS ###
    ###########################
    
    # returns motor angles in radians
    def inverse(self, END_EFFECTOR_X, END_EFFECTOR_Y, END_EFFECTOR_Z):
        # motor angles
        m1=m2=m3 = None
        
        # Rotation motor
        m3 = self.linkage.get_angle(END_EFFECTOR_Y, END_EFFECTOR_X, 0., 0.)
        
        # Linkage motors 
        m1, m2 = self.linkage.get_motor_angles(END_EFFECTOR_X/math.cos(m3), END_EFFECTOR_Z)
        
        print(f"m1: {math.degrees(m1)}\nm2: {math.degrees(m2)}\nm3: {math.degrees(m3)}")
        
        return m1, m2, m3
    
    # returns end effector coordinates in linkage space
    def forward(self, MOTOR_ANGLE_1, MOTOR_ANGLE_2, MOTOR_ANGLE_3):
        # forward extension and z
        r,ee_z = self.linkage.get_ee_coords(MOTOR_ANGLE_1, MOTOR_ANGLE_2)
        
        # convert forward extention to <x,y>
        ee_x = math.cos(MOTOR_ANGLE_3)*r
        ee_y = math.sin(MOTOR_ANGLE_3)*r
        
        print(f"ee_x: {ee_x}\nee_y: {ee_y}\nee_z: {ee_z}")
        
        return ee_x, ee_y, ee_z
    
    #########################
    ### CONTROL FUNCTIONS ###
    #########################
    
    def calibrate(self):
        # read in list of calibration points
        points_list = []
        with open('calibration_data.csv') as cal_file:
            cal_angles = csv.reader(cal_file, delimiter=' ')
            for row in cal_angles:
                # angles read from file
                m1, m2, m3 = row
                
                # convert motor angles to ee point
                x, y, z = self.forward(math.radians(float(m1)), 
                                       math.radians(float(m2)), 
                                       math.radians(float(m3)))
                points_list.append([x, y, z])
        points = np.array(points_list)
        
        # least square plane
        b = points[:, 2]
        A = np.c_[points[:, 0], points[:, 1], np.ones(len(points[:, 0]))]
        
        # plane coefficients Z = c0*X + c1*Y + c2
        C,_,_,_ = np.linalg.lstsq(A, b, rcond=None)
        self.paper_z = self.paper_x*C[0] + self.paper_y*C[1] + C[2]
        
        return points

      # takes in start coordinate of square in paper frame and side length, draws a square
    def draw(self, p_x, p_y, length):
        # MOVE TO START
        ee_x, ee_y, ee_z = self.__paper_to_linkage(p_x, p_y)
        self.__move_linkage(ee_x, ee_y, ee_z)
        
        # DRAW BOTTOM LINE
        ee_x, ee_y, ee_z = self.__paper_to_linkage(p_x+length, p_y)
        self.__move_linkage(ee_x, ee_y, ee_z)
        
        # DRAW RIGHT LINE
        ee_x, ee_y, ee_z = self.__paper_to_linkage(p_x+length, p_y+length)
        self.__move_linkage(ee_x, ee_y, ee_z)
        
        # DRAW TOP LINE
        ee_x, ee_y, ee_z = self.__paper_to_linkage(p_x, p_y+length)
        self.__move_linkage(ee_x, ee_y, ee_z)
        
        # DRAW LEFT LINE
        ee_x, ee_y, ee_z = self.__paper_to_linkage(p_x, p_y)
        self.__move_linkage(ee_x, ee_y, ee_z)
        
    # convert point in paper reference frame to linkage reference frame
    def __paper_to_linkage(self, p_x, p_y):
        # TODO
        assert(False)
        
    # move linkage to designated end effector coordinates
    def __move_linkage(self, ee_x, ee_y, ee_z):
        m1, m2, m3 = self.inverse(ee_x, ee_y, ee_z)
        # TODO
        
        
## DEBUG SCRIPT ##      
if __name__ == "__main__":
    linkage = FiveBar(3, 6, 5, 5, 4, 5, 2.5)
    model = Kinematics(linkage, 15, -1.65)
    
    #model.inverse(4.45, 0, 3.21-2.5)
    
    
    raw = model.calibrate()
  
    # plot raw data
    plt.figure()
    ax = plt.subplot(111, projection='3d')
    ax.scatter(raw[:,0], raw[:,1], color='b')    
    
    # plot plane
    ax.plot_wireframe(model.paper_x,model.paper_y,model.paper_z, color='k')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
