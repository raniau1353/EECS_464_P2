import numpy as np
import math

class Paper:
    # ORIGIN - front bottom left
    def __init__(self, x, y, z, norm):
        translation_mtx = np.array([x, y, z]).reshape((3,1))
        rotation_mtx = np.ones(3)
        # X
        if (norm == [1, 0, 0]):
            # rotate 90 about y
            th1 = math.radians(90)
            rotation_mtx = np.array([[math.cos(th1), 0, -math.sin(th1)],
                                    [0, 1, 0],
                                    [math.sin(th1), 0, math.cos(th1)]])
        # Y
        elif (norm == [0, 1, 0]):
            # rotate 90 about x
            th1 = math.radians(90)
            rotation_mtx = np.array([[1, 0, 0],
                                    [0, math.cos(th1), math.sin(th1)],
                                    [0, -math.sin(th1), math.cos(th1)]])
        # Z
        elif (norm == [0, 0, 1]):
            # no rotation
            rotation_mtx = np.array([[1, 0, 0],
                                    [0, 1, 0],
                                    [0, 0, 1]])
        # XY
        elif (norm == [1, 1, 0]):
            # rotate 90 about x
            th1 = math.radians(90)
            r1 = np.array([[1, 0, 0],
                          [0, math.cos(th1), math.sin(th1)],
                          [0, -math.sin(th1), math.cos(th1)]])
            # rotate 45 about z
            th2 = math.radians(45)
            r2 = np.array([[math.cos(th2), -math.sin(th2), 0],
                          [math.sin(th2), math.cos(th2), 0],
                          [0, 0, 1]])
            rotation_mtx = r1*r2
            
        # XZ
        elif (norm == [1, 0, 1]):
            # rotate 45 about y
            th1 = math.radians(45)
            rotation_mtx = np.array([[math.cos(th1), 0, -math.sin(th1)],
                                    [0, 1, 0],
                                    [math.sin(th1), 0, math.cos(th1)]])
        # YZ
        elif (norm == [0, 1, 1]):
            # rotate 45 about x
            th1 = math.radians(45)
            rotation_mtx = np.array([[1, 0, 0],
                                    [0, math.cos(th1), math.sin(th1)],
                                    [0, -math.sin(th1), math.cos(th1)]])
        # XYZ
        else:
            # rotate 45 about x
            th1 = math.radians(45)
            r1 = np.array([[1, 0, 0],
                          [0, math.cos(th1), math.sin(th1)],
                          [0, -math.sin(th1), math.cos(th1)]])
            # rotate 45 about z
            th2 = math.radians(45)
            r2 = np.array([[math.cos(th2), -math.sin(th2), 0],
                          [math.sin(th2), math.cos(th2), 0],
                          [0, 0, 1]])
            
            rotation_mtx = r1*r2
        
        self.M = np.append(rotation_mtx, translation_mtx, axis=1)
        self.M = np.append(self.M, [[0, 0, 0, 1]], axis=0)
        print(self.M)
        