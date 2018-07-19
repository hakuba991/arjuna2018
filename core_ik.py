import math

from config import *

# Pre Calculated Define
LINK1_SQ_MIN_LINK2_SQ = (LINK_1 ** 2) - (LINK_2 ** 2)
LINK1_LINK2_2 = (2 * LINK_1 * LINK_2)
# input : position x, position y, position z
# output : Array of degree for each joint 
def calc_ik(x, y, z):
    F = ((x * x) + (y * y) + (z * z) - LINK1_SQ_MIN_LINK2_SQ)
    E = LINK1_LINK2_2
    A = F / E
    B = math.sqrt(math.fabs(1 - (A * A)))
    tetha3 = math.atan2(-B, A)
    buff_1 = (LINK_2 * math.cos(tetha3)) + LINK_1
    buff_2 = math.sqrt((x * x) + (y * y))
    buff_3 = LINK_2 * math.sin(tetha3)
    C = (z * buff_1) - (buff_2 * buff_3)
    D = (buff_1 * buff_2) + (z * buff_3)
    return [rad_to_degree(math.atan2(x, y)), rad_to_degree(math.atan2(math.fabs(C), D)), rad_to_degree(tetha3)]


# input : radian value
# output : degree
def rad_to_degree(radian):
    return int(radian * RAD_DEG)