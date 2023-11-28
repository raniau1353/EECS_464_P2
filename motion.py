from Linkage import FiveBar
from Paper import Paper
from Kinematics import Kinematics
import csv
import math
import numpy as np
import sys
from time import sleep
from os.path import exists

from joy.decl import *
from joy import JoyApp
from joy.plans import Plan, SheetPlan
from joy.misc import loadCSV

import ckbot.logical as L

# global variable to specify how many steps to take between each point in the square
STEPS = 5

def test_kinematics(m1, m2, m3):
    print('testing kinematics')
    #                 l1  l2  l3  l4  lg  le  lift
    linkage = FiveBar(30, 60, 50, 50, 40, 51.5, 28.5) # cm
    model = Kinematics(linkage)

    c = L.Cluster(count = 3)

    # m1: Nx2F
    # m2: Nx2D_
    # m3: Nx2E

    m1 = float(m1)
    m2 = float(m2)
    m3 = float(m3)
    # m1, m2, m3 = model.inverse(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    print("INVERSE")
    print(f"m1: {math.degrees(m1)}\nm2: {math.degrees(m2)}\nm3: {math.degrees(m3)}")
    
    m1_f = ((math.degrees(m1) - 45)*(-1))*100
    m2_f = (math.degrees(m2)*(-1))*100
    m3_f = math.degrees(m3)*100
    print("CORRECTED:")
    print(f"m1: {m1_f}\nm2: {m2_f}\nm3: {m3_f}")

    m1_i = c.at.Nx2F.get_pos()
    m2_i = c.at.Nx2D.get_pos()
    m3_i = c.at.Nx2E.get_pos()

    m1_ls = np.linspace(m1_i, m1_f, num=25)
    m2_ls = np.linspace(m2_i, m2_f, num=25)
    m3_ls = np.linspace(m3_i, m3_f, num=25)
    
    print(f"m1_ls: {m1_ls}")
    print(f"m2_ls: {m2_ls}")
    print(f"m3_ls: {m3_ls}")

    for i in range(len(m1_ls)):
        c.at.Nx2F.set_pos(m1_ls[i])
        c.at.Nx2D.set_pos(m2_ls[i])
        c.at.Nx2E.set_pos(m3_ls[i])
        sleep(0.1)
    
def create_csv(filename):
    # create a csv file to write pose positions to 
    with open(filename, mode='w', newline='') as csv_file:
        print(f"CSV file '{filename}' created successfully.")

def pose(filename):
    # get current motor positions
    # m1 = c.at.Nx2F.get_pos() UNCOMMENT
    # m2 = c.at.Nx2D.get_pos() UNCOMMENT
    # m3 = c.at.Nx2E.get_pos() UNCOMMENT

    m1, m2, m3 = 10, 11, 12 # COMMENT
    current_pos = [m1, m2, m3]
    
    # check if csv file exists
    if exists(filename):
        # write each motor position as a row to the current csv file
        with open(filename, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(current_pos)
            print(f"Wrote positions {current_pos} to '{filename}' successfully.")
    else:
        print(f"Error: The CSV file '{filename}' does not exist. Run 'start <filename>' from the command line to create a new csv file")


def run(filename):
    # check if csv file exists
    if exists(filename):
        # loop through each row of the csv file and command the motors to that position
        with open(filename, mode='r') as csv_file:
            csv_reader = csv.reader(csv_file)
            first_row = next(csv_reader, None)

            # go to starting position
            m1_i, m2_i, m3_i = map(float, first_row)
            print(f"Moving to starting position {first_row}")
            # test_kinematics(m1_i, m2_i, m3_i) UNCOMMENT

            for row in csv_reader:
                # ex: ['10', '11', '12']
                print(f"Moving to position {row}")
                m1_curr, m2_curr, m3_curr = map(float, row)

                m1_ls = np.linspace(m1_i, m1_curr, num=STEPS)
                m2_ls = np.linspace(m2_i, m2_curr, num=STEPS)
                m3_ls = np.linspace(m3_i, m3_curr, num=STEPS)

                # move arm
                for i in range(len(m1_ls)):
                    print(f"Moving to {m1_ls[i]}, {m2_ls[i]}, {m3_ls[i]}")
                    # c.at.Nx2F.set_pos(m1_ls[i]) UNCOMMENT
                    # c.at.Nx2D.set_pos(m2_ls[i]) UNCOMMENT
                    # c.at.Nx2E.set_pos(m3_ls[i]) UNCOMMENT
                    # sleep(0.1) UNCOMMENT
                
                m1_i, m2_i, m3_i = m1_curr, m2_curr, m3_curr

    else:
        print(f"Error: The CSV file '{filename}' does not exist. Run 'start <filename>' from the command line to create a new csv file")

def cal_3p():
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

def main():
    if len(sys.argv) > 1:
        # ex: motion.py 10 16 0
        if len(sys.argv) == 4:
            test_kinematics(sys.argv[1], sys.argv[2], sys.argv[3])
        elif len(sys.argv) == 3:
            # ex: motion.py start square.csv
            if sys.argv[1] == 'start':
                create_csv(sys.argv[2])
            # ex: motion.py pose square.csv
            elif sys.argv[1] == 'pose':
                pose(sys.argv[2])
            # ex: motion.py run square.csv
            elif sys.argv[1] == 'run':
                run(sys.argv[2])
        else:
            print('Please enter a valid command line argument (args < 5)')
            return

if __name__ == "__main__":
    main()
