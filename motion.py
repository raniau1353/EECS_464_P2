from Linkage import FiveBar
from Paper import Paper
from Kinematics import Kinematics
import csv
import math
import numpy as np

#                 l1  l2  l3  l4  lg  le  lift
linkage = FiveBar(30, 60, 50, 50, 40, 55, 28.5) # cm
model = Kinematics(linkage)

M = model.quick_cal()


# Define coords
# paper: 21.5cm x 27.9cm
p0 = [7.5, 7.5]
p1 = [7.5, 17.5]
p2 = [17.5, 17.5]
p3 = [17.5, 7.5]


# CSV creation 
with open("output.csv", 'w') as csv_out:
    output = csv.writer(csv_out)
    output.writerow(['t', "'Nx2F'", "'Nx2E'", "'Nx2D'"])

    # point 1 - <0, 0>
    v = np.array([p0[0],p0[1],0,1]).reshape((4,1))
    E_quick = np.matmul(M, v)
    m1, m2, m3 = model.inverse(E_quick[0], E_quick[1], E_quick[2])
    output.writerow(['0', math.degrees(m1)*100, math.degrees(m2)*100, math.degrees(m3)*100])

    # point 2 - <0, 1>
    v = np.array([p1[0],p1[1],0,1]).reshape((4,1))
    E_quick = np.matmul(M, v)
    m1, m2, m3 = model.inverse(E_quick[0], E_quick[1], E_quick[2])
    output.writerow(['1', math.degrees(m1)*100, math.degrees(m2)*100, math.degrees(m3)*100])

    # point 3 - <1, 1>
    v = np.array([p2[0],p2[1],0,1]).reshape((4,1))
    E_quick = np.matmul(M, v)
    m1, m2, m3 = model.inverse(E_quick[0], E_quick[1], E_quick[2])
    output.writerow(['2', math.degrees(m1)*100, math.degrees(m2)*100, math.degrees(m3)*100])

    # point 4 - <1, 0>
    v = np.array([p3[0],p3[1],0,1]).reshape((4,1))
    E_quick = np.matmul(M, v)
    m1, m2, m3 = model.inverse(E_quick[0], E_quick[1], E_quick[2])
    output.writerow(['3', math.degrees(m1)*100, math.degrees(m2)*100, math.degrees(m3)*100])

    # point 5 - <0, 0>
    v = np.array([p0[0],p0[1],0,1]).reshape((4,1))
    E_quick = np.matmul(M, v)
    m1, m2, m3 = model.inverse(E_quick[0], E_quick[1], E_quick[2])
    output.writerow(['4', math.degrees(m1)*100, math.degrees(m2)*100, math.degrees(m3)*100])
