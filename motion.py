from config import *
from core_ik import *
import core_body
import math
import time

# Constant
B = 7
C = 3
D = 6

# Sinus Patern for create movement 
# Input :
#
#	B =
#	C =
#	D =
#	alpha =
#	phasa =
#	direksi =
# 	A =


def calc_sinus_pattern(amplitude, alpha, phasa, direction, loop_degree):
    x = 0
    y = 0
    z = -D

    f_table_a = get_motion_calc_table(loop_degree, MOTION_TABLE_SIN_LOOP_MIN90) # math.sin((LoopingDegree - 90) * 0.0174532925)
    f_table_b = get_motion_calc_table(alpha, MOTION_TABLE_COS_90MINALPHA) #math.cos((90 - Alpha) * 0.0174532925)
    f_table_c = get_motion_calc_table(loop_degree, MOTION_TABLE_SIN_LOOP) #math.sin(LoopingDegree * 0.0174532925)
    f_table_d = get_motion_calc_table(alpha, MOTION_TABLE_SIN_90MINALPHA) #math.sin((90 - Alpha) * 0.0174532925)

    if phasa == 0:
        temp_a = (-0.5)
        if (loop_degree >= 90) and (loop_degree < 270):
            z = ((C * f_table_a) - D)
    else :
        temp_a = 0.5
        if not((loop_degree >= 90) and (loop_degree < 270)):
            z = ((C * f_table_a) - D)

    if direction == DIRECTION_BACKWARD :
        amplitude = amplitude * (-1)

    x = temp_a * amplitude * f_table_b * f_table_c
    y = (temp_a * amplitude * f_table_d * f_table_c) + B
    return [x, y, z]


def get_motion_calc_table(degree_or_alpha, calculation_type):
    return MOTION_CALCULATION_TABLE[degree_or_alpha][calculation_type]
