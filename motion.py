from Linkage import FiveBar
from Paper import Paper
from Kinematics import Kinematics
import csv
import math
import numpy as np
import sys

from joy.decl import *
from joy import JoyApp
from joy.plans import Plan, SheetPlan
from joy.misc import loadCSV

import ckbot.logical as L


def test_kinematics():
    #                 l1  l2  l3  l4  lg  le  lift
    linkage = FiveBar(30, 60, 50, 50, 40, 51.5, 28.5) # cm
    model = Kinematics(linkage)

    c = L.Cluster(count = 3)

    # m1: Nx2F
    # m2: Nx2D
    # m3: Nx2E

    m1, m2, m3 = model.inverse(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    m1 = math.degrees(m1)
    m2 = math.degrees(m2)
    m3 = math.degrees(m3)
    print("INVERSE OUTPUT: ")
    print(f"m1: {m1}\nm2: {m2}\nm3: {m3}")
    
    m1 -= 45
    m1 *= -1
    m2 *= -1
    print("CORRECTED:")
    print(f"m1: {m1}\nm2: {m2}\nm3: {m3}")

    
    c.at.Nx2F.set_pos(math.ceil(m1)*100)
    c.at.Nx2D.set_pos(math.ceil(m2)*100)
    c.at.Nx2E.set_pos(math.ceil(m3)*100)

    


def main():
    #                 l1  l2  l3  l4  lg  le  lift
    linkage = FiveBar(30, 60, 50, 50, 40, 55, 28.5) # cm
    model = Kinematics(linkage)



    M = model.quick_cal()
    
    
    # Define coordst
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


if __name__ == "__main__":
    test_kinematics()