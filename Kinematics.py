import csv
import math
from Linkage import FiveBar
from Paper import Paper
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Kinematics:
    # w_x and w_y are front bottom left corner of workspace
    def __init__(self, linkage):
        self.linkage = linkage
        self.C = None
        
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
        
        #print(f"m1: {math.degrees(m1)}\nm2: {math.degrees(m2)}\nm3: {math.degrees(m3)}")
        
        return m1, m2, m3
    
    # returns end effector coordinates in linkage space
    def forward(self, MOTOR_ANGLE_1, MOTOR_ANGLE_2, MOTOR_ANGLE_3):
        # forward extension and z
        r,ee_z = self.linkage.get_ee_coords(MOTOR_ANGLE_1, MOTOR_ANGLE_2)
        
        # convert forward extention to <x,y>
        ee_x = math.cos(MOTOR_ANGLE_3)*r
        ee_y = math.sin(MOTOR_ANGLE_3)*r
        
        #print(f"ee_x: {ee_x}\nee_y: {ee_y}\nee_z: {ee_z}")
        
        return ee_x, ee_y, ee_z
    
    #########################
    ### CONTROL FUNCTIONS ###
    #########################
    
    def cal2(self):
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

        x = points[1, :] - points[0, :]
        y = points[2, :] - points[0, :]
        z = np.cross(x, y)
        
        r_m = np.array([[x], [y], [z]])
        t_m = np.array(points[0, :])
        M = np.append(r_m, t_m, axis=1)
        M = np.append(M, [0, 0, 0, 1], axis=0)
        print("KINEMAICS: M")
        print(M)
        return M
        
        
    
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
        self.C,_,_,_ = np.linalg.lstsq(A, b, rcond=None)
        
        return points

      # takes in start coordinate of square in paper frame and side length, draws a square
    def draw(self, p_x, p_y, scale):
        # MOVE TO START
        ee_x, ee_y, ee_z = self.__paper_to_linkage(p_x-scale, p_y-scale)
        self.__move_linkage(ee_x, ee_y, ee_z)
        
        # DRAW BOTTOM LINE
        ee_x, ee_y, ee_z = self.__paper_to_linkage(p_x+scale, p_y-scale)
        self.__move_linkage(ee_x, ee_y, ee_z)
        
        # DRAW RIGHT LINE
        ee_x, ee_y, ee_z = self.__paper_to_linkage(p_x+scale, p_y+scale)
        self.__move_linkage(ee_x, ee_y, ee_z)
        
        # DRAW TOP LINE
        ee_x, ee_y, ee_z = self.__paper_to_linkage(p_x-scale, p_y+scale)
        self.__move_linkage(ee_x, ee_y, ee_z)
        
        # DRAW LEFT LINE
        ee_x, ee_y, ee_z = self.__paper_to_linkage(p_x-scale, p_y-scale)
        self.__move_linkage(ee_x, ee_y, ee_z)
                
    # move linkage to designated end effector coordinates
    def __move_linkage(self, ee_x, ee_y, ee_z):
        m1, m2, m3 = self.inverse(ee_x, ee_y, ee_z)
        # TODO
        
        
## DEBUG SCRIPT ##      
if __name__ == "__main__":
    linkage = FiveBar(3, 6, 5, 5, 4, 5, 2.5)
    model = Kinematics(linkage)    
    
    raw = model.calibrate()
    M = model.cal2()
    
    paper = Paper(4, -1.65, 0, [1, 0, 1])
    
    v = np.array([1,1,0,1]).reshape((4,1))
    
    E1 = np.matmul(paper.M, v)
    
    E2 = np.matmul(M, v)
    E2[2] = E2[0]*model.C[0] + E2[1]*model.C[1] + model.C[2]
    
    # plot raw data
    plt.figure()
    ax = plt.subplot(111, projection='3d')
    ax.scatter(raw[:,0], raw[:,1], raw[:,2], color='b')    
    

    # plot plane
    paper_x, paper_y = np.meshgrid(np.linspace(4, 7.3, num=10), np.linspace(-1.65, 1.65, num=10))
    
    paper_z = paper_x*model.C[0] + paper_y*model.C[1] + model.C[2]
    ax.plot_wireframe(paper_x,paper_y,paper_z, color='k')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    
    # plot transformed points
    ax.scatter(E1[0], E1[1], E1[2], color='r')
    ax.scatter(E2[0], E2[1], E2[2], color='g')
    
    
    plt.show()
