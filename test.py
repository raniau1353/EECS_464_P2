import sys
import csv
from os.path import exists
import numpy as np

# global variable to specify how many steps to take between each point in the square
STEPS = 5

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

def main():
    if len(sys.argv) > 1:
        # ex: motion.py 10 16 0
        if len(sys.argv) == 4:
            # test_kinematics()
            print('test kinematics()')
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