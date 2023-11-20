import Kinematics
import FiveBar
import numpy as np



if __name__ == "__main__": 
    linkage = FiveBar(45, 35, 30, 35, 40, 70, 20)
    model = Kinematics(linkage)
    cal_points = np.array()
    
    # loop through calibration points
    while ():
        # get motor angles from modules
        # TODO
        
        m1, m2, m3 = None
        ee_x, ee_y, ee_z = model.forward(m1, m2, m3)
        cal_points.append([ee_x, ee_y, ee_z])
        
    # least square plane
    b = cal_points[:, 2]
    A = np.c_[cal_points[:, 0], cal_points[:, 1], np.ones(cal_points.shape(0))]
    
    # plane coefficients c3 = c0*X + c1*Y + c2*Z
    C = np.linalg.lstsq(A, b)
    
    X = np.linspace(-10, 10)
    Y = np.linspace(-10, 10)
    Z = (C[3] - C[0]*X - C[1]*Y) / C[2]
    
    