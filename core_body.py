from core_ik import *
from config import *
import time

class BODY:
    def __init__(self):
        # Initialize Variable
        self.LEGS = [0, 0, 0, 0, 0, 0]
        self.parameter = []

        # Create Joint for each Leg
        self.LEGS[R_FRONT] = LEG(JOINT(1, SERVO_TYPE_AX12, 175, FALSE),
                                 JOINT(2, SERVO_TYPE_MX28, 200, TRUE),
                                 JOINT(3, SERVO_TYPE_MX28, 285, FALSE))
        self.LEGS[R_MIDDLE] = LEG(JOINT(4, SERVO_TYPE_AX12, 150, FALSE),
                                  JOINT(5, SERVO_TYPE_MX28, 200, TRUE),
                                  JOINT(6, SERVO_TYPE_MX28, 285, FALSE))
        self.LEGS[R_BACK] = LEG(JOINT(7, SERVO_TYPE_AX12, 125, FALSE),
                                JOINT(8, SERVO_TYPE_MX28, 200, TRUE),
                                JOINT(9, SERVO_TYPE_MX28, 285, FALSE))
        self.LEGS[L_FRONT] = LEG(JOINT(10, SERVO_TYPE_AX12, 125, TRUE),
                                 JOINT(11, SERVO_TYPE_MX28, 160, FALSE),
                                 JOINT(12, SERVO_TYPE_MX28, 75, TRUE))
        self.LEGS[L_MIDDLE] = LEG(JOINT(13, SERVO_TYPE_AX12, 150, TRUE),
                                  JOINT(14, SERVO_TYPE_MX28, 140, FALSE),
                                  JOINT(15, SERVO_TYPE_MX28, 75, TRUE))
        self.LEGS[L_BACK] = LEG(JOINT(16, SERVO_TYPE_AX12, 175, TRUE),
                                JOINT(17, SERVO_TYPE_MX28, 160, FALSE),
                                JOINT(18, SERVO_TYPE_MX28, 75, TRUE))

    def getParameter(self):
        for leg in self.LEGS:
            leg.getParameter(self.parameter)
        print(self.parameter)

    def getServoParameter(self, instruction):
        servo_parameter = []
        servo_parameter.append(0xFE)
        servo_parameter.append(0xFE)
        servo_parameter.append(0xFE)
        servo_parameter.append(94) # [Parameter Lenght (Parameter in joint) + 1 = 5 ] * [Total Joint] + [Offset First Parameter Data]
        servo_parameter.append(instruction)
        servo_parameter.append(30)
        servo_parameter.append(4) # [Parameter Lenght (Parameter in joint)  = 4 ]
        for leg in self.LEGS:
            leg.getParameter(servo_parameter)
        checksum = 0
        for s in servo_parameter :
            checksum += s;
        servo_parameter.append(255 - ((checksum) % 256))
        print(servo_parameter)

    def setPosition(self, x, y, z, position_setting):
        if (position_setting == ALL_LEGS):
            for leg in self.LEGS:
                leg.setPosition(x, y, z)
        else :
            self.LEGS[position_setting].setPosition(x, y, z)


class LEG:
    def __init__(self, joint_coxa, joint_tibia, joint_femur):
        # Initialize Variable
        self.parameter = []
        self.JOINTS = []

        #Initialize Joints
        self.JOINTS.append(joint_coxa)
        self.JOINTS.append(joint_tibia)
        self.JOINTS.append(joint_femur)

    def getParameter(self, parameter):
        for joint in self.JOINTS:
            joint.get_parameter(parameter)
        return self.parameter

    def setPosition(self, x, y, z):
        degrees = calc_ik(x, y, z)
        i = 0
        for joint in self.JOINTS:
            joint.set_degree(degrees[i])
            i = i + 1


class JOINT:
    def __init__(self, servo_id, servo_type, offset_degree, inversed):
        self.parameter = []
        self.servo_id = servo_id
        self.servo_type = servo_type
        self.offset_degree = offset_degree
        self.inversed = inversed

    def get_parameter(self, parameter):
        for p in self.parameter :
            parameter.append(p)

    def set_degree2(self, degree):
        # Average Funciton Process Time : 3.079698359 ns
        start_time = time.clock()
        if self.servo_type == SERVO_TYPE_AX12:
            resolution = AX12_RESOLUTION
        else:
            resolution = MX28_RESOLUTION

        d = int(calculate_special_degree(degree, self.offset_degree, self.inversed))
        a = int( (d / 300.0) * resolution)
        H = (a / 256)
        L = (a - (H * 256))
        self.parameter = [self.servo_id, H, L, 0x88, 0x01]
        difftime = (time.clock() - start_time) * 1000 * 1000
        print(difftime, " nano second (calculation)")

    def set_degree(self, degree):
        # Average Funciton Process Time : 1.796490709 ns
        self.set_degree2(degree)
        a = time.clock()
        if self.servo_type == SERVO_TYPE_AX12:
            H = degree_translation(degree, TRANS_HIGH_AX12)
            L = degree_translation(degree, TRANS_LOW_AX12)
        else:
            H = degree_translation(degree, TRANS_HIGH_MX28)
            L = degree_translation(degree, TRANS_LOW_MX28)
        self.parameter = [self.servo_id, H, L, 0x88, 0x01]
        b = (time.clock() - a)*1000*1000
        print(b," nano second (from table)")

def calculate_special_degree(original_degree, offset_degree, inversed):
    if inversed == TRUE:
        return (original_degree - offset_degree) * (-1)
    else:
        return original_degree + offset_degree


def degree_translation(degree, type) :
    return DEGREE_TRANSLATION_TABLE[degree][type]
