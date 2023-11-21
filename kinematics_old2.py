import sys
import math
from linkage import FiveBar


class Kinematics:
    def __init__(self, linkage):
        self.linkage = linkage
        
    def inverse(self, END_EFFECTOR_X, END_EFFECTOR_Y, END_EFFECTOR_Z):
        # motor angles
        m1=m2=m3 = None
        
        # Rotation motor
        m3 = self.linkage.get_angle(END_EFFECTOR_Y, END_EFFECTOR_X, 0., 0.)
        
        # Linkage motors 
        m1, m2 = self.linkage.get_motor_angles(END_EFFECTOR_X/math.cos((m3)), END_EFFECTOR_Z)
        
        print(f"m1: {math.degrees(m1)}\nm2: {math.degrees(m2)}\nm3: {math.degrees(m3)}")
        
        return m1, m2, m3
        
    def forward(self, MOTOR_ANGLE_1, MOTOR_ANGLE_2, MOTOR_ANGLE_3):
        r,ee_z,theta_2, theta_3 = self.linkage.get_ee_coords(MOTOR_ANGLE_1, MOTOR_ANGLE_2)
        
        ee_x = math.cos(MOTOR_ANGLE_3)*r
        ee_y = math.sin(MOTOR_ANGLE_3)*r
        
        print(f"ee_x: {ee_x}\nee_y: {ee_y}\nee_z: {ee_z}")
        
        return ee_x, ee_y, ee_z, theta_2, theta_3
        
if __name__ == "__main__":
    if (len(sys.argv) != 4):
        print("Incorrect input, use: <X> <Y> <Z>")
        print(f"arg count: {len(sys.argv)}")
        sys.exit()
        
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    
    linkage = FiveBar(45, 35, 30, 35, 40, 70, 20)
    kinematics = Kinematics(linkage)
    m1, m2, m3 = kinematics.inverse(x, y, z)
    ee_x, ee_y, ee_z, theta_2, theta_3 = kinematics.forward(m1, m2, m3)
