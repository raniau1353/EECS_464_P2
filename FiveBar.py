# five bar linkage class, operates in 2d space
import sys
import math

#           //\\
#     l3   //  \\ l2
#         //    \\
#        //     //\
#        ||    // \\
#     l4 ||   //   \\ le
#        \\  // l1  \\
#      lg \\//


class FiveBar:
    # initialize links
    def __init__(self, l1, l2, l3, l4, lg, le, theta):
        # links
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        self.lg = lg 
        self.le = le
        
        self.theta = theta
        self.m1 = self.m2 = None     
        
    # returns all intersections of two circles
    def get_intersections(self, x0, z0, r0, x1, z1, r1):
        # circle 1: (x0, z0), radius r0
        # circle 2: (x1, z1), radius r1

        d=math.sqrt((x1-x0)**2 + (z1-z0)**2)
        
        # non intersecting
        if d > r0 + r1 :
            sys.exit("ERROR: outside movable range")
        # One circle within other
        if d < abs(r0-r1):
            sys.exit("ERROR: outside moveable range")
        # coincident circles
        if d == 0 and r0 == r1:
            sys.exit("ERROR: outside moveable range")
        else:
            a=(r0**2-r1**2+d**2)/(2*d)
            h=math.sqrt(r0**2-a**2)
            x2=x0+a*(x1-x0)/d   
            z2=z0+a*(z1-z0)/d   
            x3=x2+h*(z1-z0)/d     
            z3=z2-h*(x1-x0)/d 
            x4=x2-h*(z1-z0)/d
            z4=z2+h*(x1-x0)/d
            
            return (x3, z3, x4, z4)
        
    # returns angle between two points in radians
    def get_angle(self, x0, z0, x1, z1):
        x = x1 - x0
        z = z1 - z0
        
        if (z == 0 and x > 0):
            return math.radians(90)
        elif (z == 0 and x < 0):
            return math.radians(-90)
        else:
            return math.atan(x/z)
    
    # returns m1, m2 in radians
    def get_motor_angles(self, e0x, e0z):
        j0x = j0z = 0.0 # M1
        j4x, j4z = 0 - math.cos(math.radians(self.theta))*self.lg, math.sin(math.radians(self.theta))*self.lg # M2
        
        # Motor 1 angle
        j1x = j1z = None
        jx0, jz0, jx1, jz1 = self.get_intersections(j0x, j0z, self.l1, e0x, e0z, self.le)
        if (jz0 > jz1):
            j1x = jx0
            j1z = jz0
        else:
            j1x = jx1
            j1z = jz1        
        m1 = self.get_angle(0., 0., j1x, j1z)
        
        # Motor 2 angle
        theta_2 = self.get_angle(j1z, j1x, e0z, e0x)
        j2x = j1x - math.cos(theta_2)*self.l2
        j2z = j1z - math.sin(theta_2)*self.l2    
        j3x, j3z = None, None
        jx0, jz0, jx1, jz1 = self.get_intersections(j2x, j2z, self.l3, j4x, j4z, self.l4)
        if (jx0 < jx1):
            j3x = jx0
            j3z = jz0
        else:
            j3x = jx1
            j3z = jz1
        
        m2 = self.get_angle(j3x, j3z, j4x, j4z)
        
        return m1, m2
    
    def get_ee_coords(self, m1, m2):
        j1x = math.sin(m1)*self.l1
        j1z = math.cos(m1)*self.l1
        
        j4x = 0 - math.cos(math.radians(self.theta))*self.lg
        j4z = math.sin(math.radians(self.theta))*self.lg
        
        j3x = j4x + math.sin(m2)*self.l4
        j3z = j4z + math.cos(m2)*self.l4
        
        j2x = j2z = None
        jx0, jz0, jx1, jz1 = self.get_intersections(j1x, j1z, self.l2, j3x, j3z, self.l3)
        if (jz0 > jz1):
            j2x = jx0
            j2z = jz0
        else:
            j2x = jx1
            j2z = jz1
        
        theta_2 = self.get_angle(j1z, j1x, j2z, j2x)*(-1)
        ee_z = j1z - math.sin(theta_2)*self.le
        ee_x = j1x + math.cos(theta_2)*self.le
        return ee_x, ee_z
    