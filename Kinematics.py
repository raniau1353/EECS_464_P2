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
      
    def quick_cal(self):
        # read in list of calibration points
        points_list = []
        with open('quick_calibration_data.csv') as cal_file:
            cal_angles = csv.reader(cal_file)
            next(cal_angles)
            for row in cal_angles:
                
                # angles read from file
                #print(m1)
                m1 = row[1]
                m2 = row[2]
                m3 = row[3]
                # _, m1, m2, m3 = row
                
                # convert motor angles to ee point
                x, y, z = self.forward(math.radians(float(m1)/100), 
                                       math.radians(float(m2)/100), 
                                       math.radians(float(m3)/100))
                points_list.append([x, y, z])
        
        points = np.array(points_list)

        x = points[1, :] - points[0, :]
        x = x/np.linalg.norm(x)
        y = points[2, :] - points[0, :]
        y = y/np.linalg.norm(y)
        z = np.cross(x, y)
        z = z/np.linalg.norm(z)
        
        r_m = np.array([x, y, z])
        t_m = np.transpose(np.array([points[0, :]])) 
        M = np.append(r_m, t_m, axis=1)
        M = np.append(M, np.array([[0, 0, 0, 1]]), axis=0)
        print("QUICK: M")
        print(M)
        return M        
    
    def calibrate(self):
        # read in list of calibration points
        points_list = []
        with open('quick_calibration_data.csv') as cal_file:
            cal_angles = csv.reader(cal_file)
            next(cal_angles)
            for row in cal_angles:
                # angles read from file
                m1 = row[1]
                m2 = row[2]
                m3 = row[3]
                
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
        


## DEBUG SCRIPT ##      
if __name__ == "__main__":
    linkage = FiveBar(30, 60, 50, 50, 40, 55, 28.5)
    model = Kinematics(linkage)    
    
    raw = model.calibrate()
    
    M = model.quick_cal()
    
    
    # paper corner: 
    p_x = raw[0,0]
    p_y = raw[0,1]
    p_z = raw[0,2]
    
    paper = Paper(p_x, p_y, p_z, [1, 0, 1])
    
    v = np.transpose(np.array([7.5,17.5,0,1]))
    E = np.matmul(paper.M, v)
    E_quick = np.matmul(M, v)
        
    # plot raw data
    plt.figure()
    ax = plt.subplot(111, projection='3d')
    ax.scatter(raw[:,0], raw[:,1], raw[:,2], color='b')    

    # plot plane
    paper_x, paper_y = np.meshgrid(np.linspace(p_x, p_x+33, num=10), np.linspace(p_y, p_y+33, num=10))
    
    paper_z = paper_x*model.C[0] + paper_y*model.C[1] + model.C[2]
    ax.plot_wireframe(paper_x,paper_y,paper_z, color='k')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    
    # plot transformed points
    ax.scatter(E[0], E[1], E[2], color='r')
    E[2] = E[0]*model.C[0] + E[1]*model.C[1] + model.C[2]
    ax.scatter(E[0], E[1], E[2], color='g')
    ax.scatter(E_quick[0], E_quick[1], E_quick[2], color='y')
    
    plt.show()
