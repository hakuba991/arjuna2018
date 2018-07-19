import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.UART as UART
import serial
import smbus
import time
import math
import decimal
from threading import Thread
import os
import sys
import Adafruit_BBIO.ADC as ADC
import random
import signal
import glob

# //----------inisilisasi ultrasonik--------------//
UART.setup("UART1")
global i
i = 0
global var
var = {}
global string
global w
ser = serial.Serial(port="/dev/ttyO1", baudrate=57600)
ser.close()
ser.open()
if ser.isOpen():
    print
    "Serial is open!"
# //---------inisialisasi serial servo-----------//
bus = smbus.SMBus(0)
address = 0x1e  ##kaki
SERIALPORT = "/dev/ttyUSB0"
BAUDRATE = 57142.9

INST_WRITE = 3
AX_WRITE = 131
try:
    ax = serial.Serial(SERIALPORT, BAUDRATE)
except serial.SerialException:
    ports = glob.glob('/dev/ttyUSB*')
    SERIALPORT = ports[0]
    print
    SERIALPORT
    ax = serial.Serial(SERIALPORT, BAUDRATE)
# rf = serial.Serial(SERIALPORT,BAUDRATE)
parameter = range(400)
transfer = range(400)
parameter1 = range(6)
transfer1 = range(12)

clpwm = 0
crpwm = 0

# //--------------inisialisasi program atas------------//
b = 15
a = 1
susurkiri = 0
susurkanan = 0
countbelok = 0
rpwm = 0
lpwm = 0
Var_waktu = 0.0
MV = 0.0
rate = 0.0
D = 0.0
It = 0.0
P = 0.0
kd2 = 0.0
kd1 = 0.0
ki2 = 0.0
# ki1=0.0
ki1 = 0.0
kp2 = 0.35
kpspecia1 = 5
kp1 = 0.35
last_error = 0
error = 0.0
b1 = 5
b2 = 5
b3 = 5
b4 = 5
b5 = 5
b6 = 5
b8 = 5
# ================================================
batasa = 50000
bataso = 1500
countnilaiy = 0
has677 = 0
belumkeluarhome = 1
statebelka1 = 1
statebelki1 = 1
batasproxidepan = 0.70
batasproxidepan2 = 0.70
batasproxikade = 0.60
batasproxikide = 0.70
batasproxibeki = 0.46
batasproxibeka = 0.65
has8 = 0
balikinfra = 1
nyasa = 0
counci = 0
coun12 = 0
counm = 0
tambahgar = 0
garisruang = 0
pembandingdepan = 0
eee = 0
balikultradepandekat2 = 0
ddd = 1
counttt = 0
garpul = 0
skipperataan = 0
countgarismisi = 0
jalanmisi = 0
asal = 0
countgaris = 0
cekcek = 0
kondisicek = 0

api = 0
susur = 0
countingapi = 0
misiapi = 0
apiruang = 0
lorong = 0
ruang = 0
garis = 0
state2 = 0
state1 = 0
ap = 1
countapi2 = 0
garisnyasar33 = 0
garisnyasar22 = 0
garisnyasar11 = 0
konnyasar2 = 1
keluarnyasar = 0
garisnyasar3 = 0
garisnyasar2 = 0
garisnyasar1 = 0
konnyasar1 = 1
pulangkhusus = 0
arahpul = 0  # mule
nyasar = 0
arahahir = 0
arahawal = 0
detect_garis = 0
arah = 0
cono = 0
flame2 = 0
flame1 = 0
flame3 = 0
countning = 0
bear2 = 0
bear1 = 0
konq = 0
loopmuterkiri = 0
susurkiri = 0
majucon = 0
done = 0
countpu = 0
sound = 0
start = 0
mulai = 0
misiapi = 0
konk = 0
loopmuterkanan = 0
loopmuterkiri = 0
loopmuterkanan = 0
susurkiri = 0
susurkanan = 0
countbelok = 0
ir_pulang = 0
countingsound = 0
garpul = 0
garisbalik2 = 0
garisbalik1 = 0
susurkanan = 0
susurkiri = 0
state55 = 0
state4 = 0
state3 = 0
konmajju = 0
kontinga = 0
has45 = 0
has35 = 0
countingki = 0
countingka = 0
siaga2 = 0
siaga1 = 0
has80 = 0
has60 = 0
has50 = 0
has40 = 0
has30 = 0
has20 = 0
has10 = 0
countnilai = 0
kontinggg = 0
kontinggzz = 0
has100 = 0
has600 = 0
has866 = 0
ir_khususruang1 = 0
pembandingdepanruang1 = 0
kontingirnya = 0
statebeka = 0
statebeki = 0
state1 = 0
state2 = 0
statekade = 0
garisbalik2hitung = 0
garisbalik55hitung = 0
rotation = 0

# ==============Thermal Sensor==================
busth = smbus.SMBus(2)
addressth = 0x68
addressth2 = 0x6a
addressth3 = 0x69
pixel1 = range(9)
pixel2 = range(9)
pixel3 = range(9)
# ==============kompas=============================
buscm = smbus.SMBus(2)
addresscmps = 0x60


# //------------fungsi untuk tunning------------//
def packet_kirim(perintah, parameterlength, n):
    global SERIALPORT
    global BAUDRATE
    global ax
    transfer[0] = 0xFF
    transfer[1] = 0xFF
    transfer[2] = 0XFE
    transfer[3] = ((parameterlength + 1) * n + 4)
    transfer[4] = perintah
    transfer[5] = 30
    transfer[6] = parameterlength
    for i in range(1, 91):
        transfer[i + 6] = parameter[i]
    check = 0
    for o in range(2, 97):
        check += (transfer[o])
    transfer[97] = 255 - ((check) % 256)
    try:
        ax.write(chr(transfer[0]))
    except serial.SerialException:
        ports = glob.glob('/dev/ttyUSB*')
        while (len(ports) == 0):
            ports = glob.glob('/dev/ttyUSB*')
        SERIALPORT = ports[0]
        print
        SERIALPORT
        ax = serial.Serial(SERIALPORT, BAUDRATE)
        ax.write(chr(transfer[0]))
    for p in range(1, 98):
        ax.write(chr(transfer[p]))


l1 = 7.5
l2 = 6.2


def invers_kinematik1(x, y, z):
    global tetha11
    global tetha21
    global tetha31
    F = ((x * x) + (y * y) + (z * z) - (l1 ** 2) - (l2 ** 2))
    E = (2 * l1 * l2)
    A = F / E
    B = math.sqrt(math.fabs(1 - (A * A)))
    tetha31 = math.atan2(-B, A)
    buff_1 = (l2 * math.cos(tetha31)) + l1
    buff_2 = math.sqrt((x * x) + (y * y))
    buff_3 = l2 * math.sin(tetha31)
    C = (z * buff_1) - (buff_2 * buff_3)
    D = (buff_1 * buff_2) + (z * buff_3)
    tetha21 = math.atan2(math.fabs(C), D)
    tetha11 = math.atan2(x, y)


def invers_kinematik2(x, y, z):
    global tetha12
    global tetha22
    global tetha32
    F = ((x * x) + (y * y) + (z * z) - (l1 ** 2) - (l2 ** 2))
    E = (2 * l1 * l2)
    A = F / E
    B = math.sqrt(math.fabs(1 - (A * A)))
    tetha32 = math.atan2(-B, A)
    buff_1 = (l2 * math.cos(tetha32)) + l1
    buff_2 = math.sqrt((x * x) + (y * y))
    buff_3 = l2 * math.sin(tetha32)
    C = (z * buff_1) - (buff_2 * buff_3)
    D = (buff_1 * buff_2) + (z * buff_3)
    tetha22 = math.atan2(math.fabs(C), D)
    tetha12 = math.atan2(x, y)


def invers_kinematik3(x, y, z):
    global tetha13
    global tetha23
    global tetha33
    F = ((x * x) + (y * y) + (z * z) - (l1 ** 2) - (l2 ** 2))
    E = (2 * l1 * l2)
    A = F / E
    B = math.sqrt(math.fabs(1 - (A * A)))
    tetha33 = math.atan2(-B, A)
    buff_1 = (l2 * math.cos(tetha33)) + l1
    buff_2 = math.sqrt((x * x) + (y * y))
    buff_3 = l2 * math.sin(tetha33)
    C = (z * buff_1) - (buff_2 * buff_3)
    D = (buff_1 * buff_2) + (z * buff_3)
    tetha23 = math.atan2(math.fabs(C), D)
    tetha13 = math.atan2(x, y)


def invers_kinematik4(x, y, z):
    global tetha14
    global tetha24
    global tetha34
    F = ((x * x) + (y * y) + (z * z) - (l1 ** 2) - (l2 ** 2))
    E = (2 * l1 * l2)
    A = F / E
    B = math.sqrt(math.fabs(1 - (A * A)))
    tetha34 = math.atan2(-B, A)
    buff_1 = (l2 * math.cos(tetha34)) + l1
    buff_2 = math.sqrt((x * x) + (y * y))
    buff_3 = l2 * math.sin(tetha34)
    C = (z * buff_1) - (buff_2 * buff_3)
    D = (buff_1 * buff_2) + (z * buff_3)
    tetha24 = math.atan2(math.fabs(C), D)
    tetha14 = math.atan2(x, y)


def invers_kinematik5(x, y, z):
    global tetha15
    global tetha25
    global tetha35
    F = ((x * x) + (y * y) + (z * z) - (l1 ** 2) - (l2 ** 2))
    E = (2 * l1 * l2)
    A = F / E
    B = math.sqrt(math.fabs(1 - (A * A)))
    tetha35 = math.atan2(-B, A)
    buff_1 = (l2 * math.cos(tetha35)) + l1
    buff_2 = math.sqrt((x * x) + (y * y))
    buff_3 = l2 * math.sin(tetha35)
    C = (z * buff_1) - (buff_2 * buff_3)
    D = (buff_1 * buff_2) + (z * buff_3)
    tetha25 = math.atan2(math.fabs(C), D)
    tetha15 = math.atan2(x, y)


def invers_kinematik6(x, y, z):
    global tetha16
    global tetha26
    global tetha36
    F = ((x * x) + (y * y) + (z * z) - (l1 ** 2) - (l2 ** 2))
    E = (2 * l1 * l2)
    A = F / E
    B = math.sqrt(math.fabs(1 - (A * A)))
    tetha36 = math.atan2(-B, A)
    buff_1 = (l2 * math.cos(tetha36)) + l1
    buff_2 = math.sqrt((x * x) + (y * y))
    buff_3 = l2 * math.sin(tetha36)
    C = (z * buff_1) - (buff_2 * buff_3)
    D = (buff_1 * buff_2) + (z * buff_3)
    tetha26 = math.atan2(math.fabs(C), D)
    tetha16 = math.atan2(x, y)


def rad_to_deg1():
    global degree_11
    global degree_21
    global degree_31
    satu1 = tetha11 * 57.2957795
    dua1 = tetha21 * 57.2957795
    tiga1 = tetha31 * 57.2957795
    degree_11 = int(satu1)
    degree_21 = int(dua1)
    degree_31 = int(tiga1)


def rad_to_deg2():
    global degree_12
    global degree_22
    global degree_32
    satu2 = tetha12 * 57.2957795
    dua2 = tetha22 * 57.2957795
    tiga2 = tetha32 * 57.2957795
    degree_12 = int(satu2)
    degree_22 = int(dua2)
    degree_32 = int(tiga2)


def rad_to_deg3():
    global degree_13
    global degree_23
    global degree_33
    satu3 = tetha13 * 57.2957795
    dua3 = tetha23 * 57.2957795
    tiga3 = tetha33 * 57.2957795
    degree_13 = int(satu3)
    degree_23 = int(dua3)
    degree_33 = int(tiga3)


def rad_to_deg4():
    global degree_14
    global degree_24
    global degree_34
    satu4 = tetha14 * 57.2957795
    dua4 = tetha24 * 57.2957795
    tiga4 = tetha34 * 57.2957795
    degree_14 = int(satu4)
    degree_24 = int(dua4)
    degree_34 = int(tiga4)


def rad_to_deg5():
    global degree_15
    global degree_25
    global degree_35
    satu5 = tetha15 * 57.2957795
    dua5 = tetha25 * 57.2957795
    tiga5 = tetha35 * 57.2957795
    degree_15 = int(satu5)
    degree_25 = int(dua5)
    degree_35 = int(tiga5)


def rad_to_deg6():
    global degree_16
    global degree_26
    global degree_36
    satu6 = tetha16 * 57.2957795
    dua6 = tetha26 * 57.2957795
    tiga6 = tetha36 * 57.2957795
    degree_16 = int(satu6)
    degree_26 = int(dua6)
    degree_36 = int(tiga6)


def kanan1():
    global outdeg11
    global outdeg21
    global outdeg31
    sp_deg1 = 175
    sp_deg2 = 200
    sp_deg3 = 285
    outdeg11 = (degree_11 + sp_deg1)
    outdeg21 = (degree_21 - sp_deg2) * (-1)
    outdeg31 = (degree_31 + sp_deg3)


def kanan2():
    global outdeg12
    global outdeg22
    global outdeg32
    sp_deg1 = 150
    sp_deg2 = 200
    sp_deg3 = 285
    outdeg12 = (degree_12 + sp_deg1)
    outdeg22 = (degree_22 - sp_deg2) * (-1)
    outdeg32 = (degree_32 + sp_deg3)


def kanan3():
    global outdeg13
    global outdeg23
    global outdeg33
    sp_deg1 = 125
    sp_deg2 = 200
    sp_deg3 = 285
    outdeg13 = (degree_13 + sp_deg1)
    outdeg23 = (degree_23 - sp_deg2) * (-1)
    outdeg33 = (degree_33 + sp_deg3)


def kiri1():
    global outdeg14
    global outdeg24
    global outdeg34
    sp_deg4 = 125
    sp_deg5 = 160
    sp_deg6 = 75
    outdeg14 = (degree_14 - sp_deg4) * (-1)
    outdeg24 = (degree_24 + sp_deg5)
    outdeg34 = (degree_34 - sp_deg6) * (-1)


def kiri2():
    global outdeg15
    global outdeg25
    global outdeg35
    sp_deg4 = 150
    sp_deg5 = 140
    sp_deg6 = 75
    outdeg15 = (degree_15 - sp_deg4) * (-1)
    outdeg25 = (degree_25 + sp_deg5)
    outdeg35 = (degree_35 - sp_deg6) * (-1)


def kiri3():
    global outdeg16
    global outdeg26
    global outdeg36
    sp_deg4 = 175
    sp_deg5 = 160
    sp_deg6 = 75
    outdeg16 = (degree_16 - sp_deg4) * (-1)
    outdeg26 = (degree_26 + sp_deg5)
    outdeg36 = (degree_36 - sp_deg6) * (-1)


var_z1 = 0
var_y1 = 0
var_x1 = 0
z = 0
y = 0
x = 0
var_z3 = 0
var_y3 = 0
var_x3 = 0
var_z4 = 0
var_y4 = 0
var_x4 = 0
var_z5 = 0

var_y5 = 0
var_x5 = 0
var_z6 = 0
var_y6 = 0
var_x6 = 0


def sinus_pattern1(A, B, C, D, alpa, phasa, direksi):
    global var_x1
    global var_y1
    global var_z1
    global sudut
    if direksi == 0:
        var_y1 = ((-0.5) * (A) * math.sin((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)) + B
        if phasa == 0:
            var_x1 = (-0.5) * (A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z1 = -D
            elif (sudut >= 90) and (sudut < 270):
                var_z1 = ((C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 270) and (sudut < 360):
                var_z1 = -D

        elif phasa == 1:
            var_x1 = (0.5) * (A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z1 = ((-C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 90) and (sudut < 270):
                var_z1 = -D
            elif (sudut >= 270) and (sudut < 360):
                var_z1 = (((-C) * math.sin((sudut - 90) * 0.0174532925)) - D)
    elif direksi == 1:
        var_y1 = ((-0.5) * (-A) * math.sin((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)) + B
        if phasa == 0:
            var_x1 = (-0.5) * (-A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z1 = -D
            elif (sudut >= 90) and (sudut < 270):
                var_z1 = ((C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 270) and (sudut < 360):
                var_z1 = -D

        elif phasa == 1:
            var_x1 = (0.5) * (-A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925);
            if (sudut >= 0) and (sudut < 90):
                var_z1 = ((-C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 90) and (sudut < 270):
                var_z1 = -D
            elif (sudut >= 270) and (sudut < 360):
                var_z1 = (((-C) * math.sin((sudut - 90) * 0.0174532925)) - D)


def sinus_pattern2(A, B, C, D, alpa, phasa, direksi):
    global x
    global y
    global z
    global sudut
    if direksi == 0:
        var_y2 = ((-0.5) * (A) * math.sin((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)) + B
        if phasa == 0:
            var_x2 = (-0.5) * (A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z2 = -D
            elif (sudut >= 90) and (sudut < 270):
                var_z2 = ((C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 270) and (sudut < 360):
                var_z2 = -D

        elif phasa == 1:
            var_x2 = (0.5) * (A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z2 = ((-C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 90) and (sudut < 270):
                var_z2 = -D
            elif (sudut >= 270) and (sudut < 360):
                var_z2 = (((-C) * math.sin((sudut - 90) * 0.0174532925)) - D)

    elif direksi == 1:
        var_y2 = ((-0.5) * (-A) * math.sin((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)) + B

        if phasa == 0:
            var_x2 = (-0.5) * (-A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925);

            if (sudut >= 0) and (sudut < 90):
                var_z2 = -D
            elif (sudut >= 90) and (sudut < 270):
                var_z2 = ((C * math.sin((sudut - 90) * 0.0174532925)) - D)

            elif (sudut >= 270) and (sudut < 360):
                var_z2 = -D

        elif phasa == 1:
            var_x2 = (0.5) * (-A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925);
            if (sudut >= 0) and (sudut < 90):
                var_z2 = ((-C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 90) and (sudut < 270):
                var_z2 = -D
            elif (sudut >= 270) and (sudut < 360):
                var_z2 = (((-C) * math.sin((sudut - 90) * 0.0174532925)) - D)


def sinus_pattern3(A, B, C, D, alpa, phasa, direksi):
    global var_x3
    global var_y3
    global var_z3
    global sudut
    if direksi == 0:
        var_y3 = ((-0.5) * (A) * math.sin((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)) + B
        if phasa == 0:
            var_x3 = (-0.5) * (A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z3 = -D
            elif (sudut >= 90) and (sudut < 270):
                var_z3 = ((C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 270) and (sudut < 360):
                var_z3 = -D

        elif phasa == 1:
            var_x3 = (0.5) * (A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z3 = ((-C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 90) and (sudut < 270):
                var_z3 = -D
            elif (sudut >= 270) and (sudut < 360):
                var_z3 = (((-C) * math.sin((sudut - 90) * 0.0174532925)) - D)

    elif direksi == 1:
        var_y3 = ((-0.5) * (-A) * math.sin((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)) + B
        if phasa == 0:
            var_x3 = (-0.5) * (-A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z3 = -D
            elif (sudut >= 90) and (sudut < 270):
                var_z3 = ((C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 270) and (sudut < 360):
                var_z3 = -D

        elif phasa == 1:
            var_x3 = (0.5) * (-A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925);
            if (sudut >= 0) and (sudut < 90):
                var_z3 = ((-C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 90) and (sudut < 270):
                var_z3 = -D
            elif (sudut >= 270) and (sudut < 360):
                var_z3 = (((-C) * math.sin((sudut - 90) * 0.0174532925)) - D)


def sinus_pattern4(A, B, C, D, alpa, phasa, direksi):
    global var_x4
    global var_y4
    global var_z4
    global sudut
    if direksi == 0:
        var_y4 = ((-0.5) * (A) * math.sin((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)) + B
        if phasa == 0:
            var_x4 = (-0.5) * (A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z4 = -D
            elif (sudut >= 90) and (sudut < 270):
                var_z4 = ((C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 270) and (sudut < 360):
                var_z4 = -D
        elif phasa == 1:
            var_x4 = (0.5) * (A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z4 = ((-C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 90) and (sudut < 270):
                var_z4 = -D
            elif (sudut >= 270) and (sudut < 360):
                var_z4 = (((-C) * math.sin((sudut - 90) * 0.0174532925)) - D)

    elif direksi == 1:
        var_y4 = ((-0.5) * (-A) * math.sin((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)) + B
        if phasa == 0:
            var_x4 = (-0.5) * (-A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z4 = -D
            elif (sudut >= 90) and (sudut < 270):
                var_z4 = ((C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 270) and (sudut < 360):
                var_z4 = -D
        elif phasa == 1:
            var_x4 = (0.5) * (-A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925);
            if (sudut >= 0) and (sudut < 90):
                var_z4 = ((-C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 90) and (sudut < 270):
                var_z4 = -D
            elif (sudut >= 270) and (sudut < 360):
                var_z4 = (((-C) * math.sin((sudut - 90) * 0.0174532925)) - D)


def sinus_pattern5(A, B, C, D, alpa, phasa, direksi):
    global var_x5
    global var_y5
    global var_z5
    global sudut
    if direksi == 0:
        var_y5 = ((-0.5) * (A) * math.sin((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)) + B
        if phasa == 0:
            var_x5 = (-0.5) * (A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z5 = -D
            elif (sudut >= 90) and (sudut < 270):
                var_z5 = ((C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 270) and (sudut < 360):
                var_z5 = -D

        elif phasa == 1:
            var_x5 = (0.5) * (A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z5 = ((-C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 90) and (sudut < 270):
                var_z5 = -D
            elif (sudut >= 270) and (sudut < 360):
                var_z5 = (((-C) * math.sin((sudut - 90) * 0.0174532925)) - D)
    elif direksi == 1:
        var_y5 = ((-0.5) * (-A) * math.sin((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)) + B
        if phasa == 0:
            var_x5 = (-0.5) * (-A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z5 = -D
            elif (sudut >= 90) and (sudut < 270):
                var_z5 = ((C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 270) and (sudut < 360):
                var_z5 = -D
        elif phasa == 1:
            var_x5 = (0.5) * (-A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925);
            if (sudut >= 0) and (sudut < 90):
                var_z5 = ((-C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 90) and (sudut < 270):
                var_z5 = -D
            elif (sudut >= 270) and (sudut < 360):
                var_z5 = (((-C) * math.sin((sudut - 90) * 0.0174532925)) - D)


def sinus_pattern6(A, B, C, D, alpa, phasa, direksi):
    global var_x6
    global var_y6
    global var_z6
    global sudut
    if direksi == 0:
        var_y6 = ((-0.5) * (A) * math.sin((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)) + B
        if phasa == 0:
            var_x6 = (-0.5) * (A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z6 = -D
            elif (sudut >= 90) and (sudut < 270):
                var_z6 = ((C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 270) and (sudut < 360):
                var_z6 = -D
        elif phasa == 1:
            var_x6 = (0.5) * (A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z6 = ((-C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 90) and (sudut < 270):
                var_z6 = -D
            elif (sudut >= 270) and (sudut < 360):
                var_z6 = (((-C) * math.sin((sudut - 90) * 0.0174532925)) - D)
    elif direksi == 1:
        var_y6 = ((-0.5) * (-A) * math.sin((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)) + B
        if phasa == 0:
            var_x6 = (-0.5) * (-A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925)
            if (sudut >= 0) and (sudut < 90):
                var_z6 = -D
            elif (sudut >= 90) and (sudut < 270):
                var_z6 = ((C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 270) and (sudut < 360):
                var_z6 = -D
        elif phasa == 1:
            var_x6 = (0.5) * (-A) * math.cos((90 - alpa) * 0.0174532925) * math.sin(sudut * 0.0174532925);
            if (sudut >= 0) and (sudut < 90):
                var_z6 = ((-C * math.sin((sudut - 90) * 0.0174532925)) - D)
            elif (sudut >= 90) and (sudut < 270):
                var_z6 = -D
            elif (sudut >= 270) and (sudut < 360):
                var_z6 = (((-C) * math.sin((sudut - 90) * 0.0174532925)) - D)


def translate1(z1):
    global H1
    global L1
    global deci1
    global desi1
    desi1 = (z1 / 300.0) * 1023
    deci1 = int(desi1)
    H1 = (deci1 / 256)
    L1 = (deci1 - (H1 * 256))


def translate2(z2):
    global H2
    global L2
    global deci2
    global desi2
    desi2 = (z2 / 300.0) * 3420
    deci2 = int(desi2)
    H2 = (deci2 / 256)
    L2 = (deci2 - (H2 * 256))


def translate3(z3):
    global H3
    global L3
    global deci3
    global desi3
    desi3 = (z3 / 300.0) * 3420
    deci3 = int(desi3)
    H3 = (deci3 / 256)
    L3 = (deci3 - (H3 * 256))


def translate4(z4):
    global H4
    global L4
    global deci4
    global desi4
    desi4 = (z4 / 300.0) * 1023
    deci4 = int(desi4)
    H4 = (deci4 / 256)
    L4 = (deci4 - (H4 * 256))


def translate5(z5):
    global H5
    global L5
    global deci5
    global desi5
    desi5 = (z5 / 300.0) * 3420
    deci5 = int(desi5)
    H5 = (deci5 / 256)
    L5 = (deci5 - (H5 * 256))


def translate6(z6):
    global H6
    global L6
    global deci6
    global desi6
    desi6 = (z6 / 300.0) * 3420
    deci6 = int(desi6)
    H6 = (deci6 / 256)
    L6 = (deci6 - (H6 * 256))


def translate7(z7):
    global H7
    global L7
    global deci7
    global desi7
    desi7 = (z7 / 300.0) * 1023
    deci7 = int(desi7)
    H7 = (deci7 / 256)
    L7 = (deci7 - (H7 * 256))


def translate8(z8):
    global H8
    global L8
    global deci8
    global desi8
    desi8 = (z8 / 300.0) * 3420
    deci8 = int(desi8)
    H8 = (deci8 / 256)
    L8 = (deci8 - (H8 * 256))


def translate9(z9):
    global H9
    global L9
    global deci9
    global desi9
    desi9 = (z9 / 300.0) * 3420
    deci9 = int(desi9)
    H9 = (deci9 / 256)
    L9 = (deci9 - (H9 * 256))


def translatea(za):
    global Ha
    global La
    global decia
    global desia
    desia = (za / 300.0) * 1023
    decia = int(desia)
    Ha = (decia / 256)
    La = (decia - (Ha * 256))


def translateb(zb):
    global Hb
    global Lb
    global decib
    global desib
    desib = (zb / 300.0) * 3420
    decib = int(desib)
    Hb = (decib / 256)
    Lb = (decib - (Hb * 256))


def translatec(zc):
    global Hc
    global Lc
    global decic
    global desic
    desic = (zc / 300.0) * 3420
    decic = int(desic)
    Hc = (decic / 256)
    Lc = (decic - (Hc * 256))


def translated(zd):
    global Hd
    global Ld
    global decid
    global desid
    desid = (zd / 300.0) * 1023
    decid = int(desid)
    Hd = (decid / 256)
    Ld = (decid - (Hd * 256))


def translatee(ze):
    global He
    global Le
    global decie
    global desie
    desie = (ze / 300.0) * 3420
    decie = int(desie)
    He = (decie / 256)
    Le = (decie - (He * 256))


def translatef(zf):
    global Hf
    global Lf
    global decif
    global desif
    desif = (zf / 300.0) * 3420
    decif = int(desif)
    Hf = (decif / 256)
    Lf = (decif - (Hf * 256))


def translateg(zg):
    global Hg
    global Lg
    global decig
    global desig
    desig = (zg / 300.0) * 1023
    decig = int(desig)
    Hg = (decig / 256)
    Lg = (decig - (Hg * 256))


def translateh(zh):
    global Hh
    global Lh
    global decih
    global desih
    desih = (zh / 300.0) * 3420
    decih = int(desih)
    Hh = (decih / 256)
    Lh = (decih - (Hh * 256))


def translatei(zi):
    global Hi
    global Li
    global decii
    global desii
    desii = (zi / 300.0) * 3420
    decii = int(desii)
    Hi = (decii / 256)
    Li = (decii - (Hi * 256))


def kanan_1(A, alpa, direksi):
    sinus_pattern1(A, 7, 3, 6, alpa, 0, direksi)  # b5c3
    invers_kinematik1(var_x1, var_y1, var_z1)
    rad_to_deg1()
    kanan1()
    translate1(outdeg11)
    translate2(outdeg21 - 5)  # -3
    translate3(outdeg31)  # -4 +3

    #	print "sudut kanan1: ", outdeg11, ", ", outdeg21, ", ", outdeg31

    parameter[1] = 1
    parameter[2] = L1
    parameter[3] = H1
    parameter[4] = 0x88
    parameter[5] = 0x01

    parameter[6] = 2
    parameter[7] = L2
    parameter[8] = H2
    parameter[9] = 0x88
    parameter[10] = 0x01

    parameter[11] = 3
    parameter[12] = L3
    parameter[13] = H3
    parameter[14] = 0x88
    parameter[15] = 0x01


def kanan_2(A, alpa, direksi):
    sinus_pattern2(A, 7, 3, 6, alpa, 1, direksi)  # b5.5c3d3.25
    invers_kinematik2(x, y, z)
    rad_to_deg2()
    kanan2()
    translate4(outdeg12)
    translate5(outdeg22)  # +6
    translate6(outdeg32)  # -1

    #	print "sudut kanan2: ", outdeg12, ", ", outdeg22, ", ", outdeg32

    parameter[16] = 4
    parameter[17] = L4
    parameter[18] = H4
    parameter[19] = 0x88
    parameter[20] = 0x01

    parameter[21] = 5
    parameter[22] = L5
    parameter[23] = H5
    parameter[24] = 0x88
    parameter[25] = 0x01

    parameter[26] = 6
    parameter[27] = L6
    parameter[28] = H6
    parameter[29] = 0x88
    parameter[30] = 0x01


def kanan_3(A, alpa, direksi):
    sinus_pattern3(A, 7, 3, 6, alpa, 0, direksi)  # D2.5#c3
    invers_kinematik3(var_x3, var_y3, var_z3)
    rad_to_deg3()
    kanan3()
    translate7(outdeg13)
    translate8(outdeg23)  # +1
    translate9(outdeg33)

    #	print "sudut kanan3: ", outdeg13, ", ", outdeg23, ", ", outdeg33

    parameter[31] = 7
    parameter[32] = L7
    parameter[33] = H7
    parameter[34] = 0x88
    parameter[35] = 0x01

    parameter[36] = 8
    parameter[37] = L8
    parameter[38] = H8
    parameter[39] = 0x88
    parameter[40] = 0x01

    parameter[41] = 9
    parameter[42] = L9
    parameter[43] = H9
    parameter[44] = 0x88
    parameter[45] = 0x01


def kiri_1(A, alpa, direksi):
    sinus_pattern4(A, 7, 3, 6, alpa, 1, direksi)  # d3.35 c2.5#b5c3d3.25
    invers_kinematik4(var_x4, var_y4, var_z4)
    rad_to_deg4()
    kiri1()
    translatea(outdeg14)  # +2
    translateb(outdeg24 + 5)  # -2
    translatec(outdeg34)  # -2

    parameter[46] = 10
    parameter[47] = La
    parameter[48] = Ha
    parameter[49] = 0x88
    parameter[50] = 0x01

    parameter[51] = 20
    parameter[52] = Lb
    parameter[53] = Hb
    parameter[54] = 0x88
    parameter[55] = 0x01

    parameter[56] = 12
    parameter[57] = Lc
    parameter[58] = Hc
    parameter[59] = 0x88
    parameter[60] = 0x01


def kiri_2(A, alpa, direksi):
    sinus_pattern5(A, 7, 3, 6, alpa, 0, direksi)  # d3.4 c2.5#b5c3d3.25
    invers_kinematik5(var_x5, var_y5, var_z5)
    rad_to_deg5()
    kiri2()
    translated(outdeg15)
    translatee(outdeg25)  # -6
    translatef(outdeg35)  # -1

    #       print "sudut kiri2: ", outdeg15, ", ", outdeg25, ", ", outdeg35

    parameter[61] = 13
    parameter[62] = Ld
    parameter[63] = Hd
    parameter[64] = 0x88
    parameter[65] = 0x01

    parameter[66] = 14
    parameter[67] = Le
    parameter[68] = He
    parameter[69] = 0x88
    parameter[70] = 0x01

    parameter[71] = 15
    parameter[72] = Lf
    parameter[73] = Hf
    parameter[74] = 0x88
    parameter[75] = 0x01


def kiri_3(A, alpa, direksi):
    sinus_pattern6(A, 7, 3, 6, alpa, 1, direksi)  # c3
    invers_kinematik6(var_x6, var_y6, var_z6)
    rad_to_deg6()
    kiri3()
    translateg(outdeg16)
    translateh(outdeg26)  # -2
    translatei(outdeg36)

    parameter[76] = 19
    parameter[77] = Lg
    parameter[78] = Hg
    parameter[79] = 0x88
    parameter[80] = 0x01

    parameter[81] = 17
    parameter[82] = Lh
    parameter[83] = Hh
    parameter[84] = 0x88
    parameter[85] = 0x01

    parameter[86] = 18
    parameter[87] = Li
    parameter[88] = Hi
    parameter[89] = 0x88
    parameter[90] = 0x01


def Kanan_depan(x, y, z):
    invers_kinematik1(x, y, -z)
    rad_to_deg1()
    kanan1()
    translate1(outdeg11)
    translate2(outdeg21)
    translate3(outdeg31)
    # print "sudut kanan1: ", outdeg11, ", ", outdeg21, ", ", outdeg31

    parameter[1] = 1
    parameter[2] = L1
    parameter[3] = H1
    parameter[4] = 0x00
    parameter[5] = 0x02

    parameter[6] = 2
    parameter[7] = L2
    parameter[8] = H2
    parameter[9] = 0x00
    parameter[10] = 0x02

    parameter[11] = 3  #
    parameter[12] = L3  #
    parameter[13] = H3
    parameter[14] = 0x00
    parameter[15] = 0x02


def Kanan_tengah(x, y, z):
    invers_kinematik2(x, y, -z)
    rad_to_deg2()
    kanan2()
    translate4(outdeg12)
    translate5(outdeg22)
    translate6(outdeg32)

    parameter[16] = 4
    parameter[17] = L4
    parameter[18] = H4
    parameter[19] = 0x00
    parameter[20] = 0x02

    parameter[21] = 5
    parameter[22] = L5
    parameter[23] = H5
    parameter[24] = 0x00
    parameter[25] = 0x02

    parameter[26] = 6
    parameter[27] = L6
    parameter[28] = H6
    parameter[29] = 0x00
    parameter[30] = 0x02


def Kanan_belakang(x, y, z):
    invers_kinematik3(x, y, -z)
    rad_to_deg3()
    kanan3()
    translate7(outdeg13)
    translate8(outdeg23)
    translate9(outdeg33)

    parameter[31] = 7
    parameter[32] = L7
    parameter[33] = H7
    parameter[34] = 0x00
    parameter[35] = 0x02

    parameter[36] = 8
    parameter[37] = L8
    parameter[38] = H8
    parameter[39] = 0x00
    parameter[40] = 0x02

    parameter[41] = 9
    parameter[42] = L9
    parameter[43] = H9
    parameter[44] = 0x00
    parameter[45] = 0x02


def Kiri_depan(x, y, z):
    invers_kinematik4(x, y, -z)
    rad_to_deg4()
    kiri1()
    translatea(outdeg14)
    translateb(outdeg24)
    translatec(outdeg34)

    parameter[46] = 10
    parameter[47] = La
    parameter[48] = Ha
    parameter[49] = 0x00
    parameter[50] = 0x02

    parameter[51] = 20
    parameter[52] = Lb
    parameter[53] = Hb
    parameter[54] = 0x00
    parameter[55] = 0x02

    parameter[56] = 12
    parameter[57] = Lc
    parameter[58] = Hc
    parameter[59] = 0x00
    parameter[60] = 0x02


def Kiri_tengah(x, y, z):
    invers_kinematik5(x, y, -z)
    rad_to_deg5()
    kiri2()
    translated(outdeg15)
    translatee(outdeg25)
    translatef(outdeg35)

    parameter[61] = 13
    parameter[62] = Ld
    parameter[63] = Hd
    parameter[64] = 0x00
    parameter[65] = 0x02

    parameter[66] = 14
    parameter[67] = Le
    parameter[68] = He
    parameter[69] = 0x00
    parameter[70] = 0x02

    parameter[71] = 15
    parameter[72] = Lf
    parameter[73] = Hf
    parameter[74] = 0x00
    parameter[75] = 0x02


def Kiri_belakang(x, y, z):
    invers_kinematik6(x, y, -z)
    rad_to_deg6()
    kiri3()
    translateg(outdeg16)
    translateh(outdeg26)
    translatei(outdeg36)

    parameter[76] = 19
    parameter[77] = Lg
    parameter[78] = Hg
    parameter[79] = 0x00
    parameter[80] = 0x02

    parameter[81] = 17
    parameter[82] = Lh
    parameter[83] = Hh
    parameter[84] = 0x00
    parameter[85] = 0x02

    parameter[86] = 18
    parameter[87] = Li
    parameter[88] = Hi
    parameter[89] = 0x00
    parameter[90] = 0x02


def kepiting_kanan_1(A, alpa, direksi):
    sinus_pattern1(A, 7, 3, 6, alpa, 0, direksi)
    invers_kinematik1(var_x1, var_y1, var_z1)
    rad_to_deg1()
    kanan1()
    translate1(outdeg11)
    translate2(outdeg21)
    translate3(outdeg31)

    parameter[1] = 1
    parameter[2] = L1
    parameter[3] = H1
    parameter[4] = 0x88
    parameter[5] = 0x01

    parameter[6] = 2
    parameter[7] = L2
    parameter[8] = H2
    parameter[9] = 0x88
    parameter[10] = 0x01

    parameter[11] = 3
    parameter[12] = L3
    parameter[13] = H3
    parameter[14] = 0x88
    parameter[15] = 0x01


def kepiting_kanan_2(A, alpa, direksi):
    sinus_pattern2(A, 7, 3, 6, alpa, 1, direksi)
    invers_kinematik2(x, y, z)
    rad_to_deg2()
    kanan2()
    translate4(outdeg12)
    translate5(outdeg22)
    translate6(outdeg32)

    parameter[16] = 4
    parameter[17] = L4
    parameter[18] = H4
    parameter[19] = 0x88
    parameter[20] = 0x01

    parameter[21] = 5
    parameter[22] = L5
    parameter[23] = H5
    parameter[24] = 0x88
    parameter[25] = 0x01

    parameter[26] = 6
    parameter[27] = L6
    parameter[28] = H6
    parameter[29] = 0x88
    parameter[30] = 0x01


def kepiting_kanan_3(A, alpa, direksi):
    sinus_pattern3(A, 7, 3, 6, alpa, 0, direksi)
    invers_kinematik3(var_x3, var_y3, var_z3)
    rad_to_deg3()
    kanan3()
    translate7(outdeg13)
    translate8(outdeg23)
    translate9(outdeg33)

    parameter[31] = 7
    parameter[32] = L7
    parameter[33] = H7
    parameter[34] = 0x88
    parameter[35] = 0x01

    parameter[36] = 8
    parameter[37] = L8
    parameter[38] = H8
    parameter[39] = 0x88
    parameter[40] = 0x01

    parameter[41] = 9
    parameter[42] = L9
    parameter[43] = H9
    parameter[44] = 0x88
    parameter[45] = 0x01


def kepiting_kiri_1(A, alpa, direksi):
    sinus_pattern4(A, 7, 3, 6, alpa, 1, direksi)
    invers_kinematik4(var_x4, var_y4, var_z4)
    rad_to_deg4()
    kiri1()
    translatea(outdeg14)
    translateb(outdeg24)
    translatec(outdeg34)

    parameter[46] = 10
    parameter[47] = La
    parameter[48] = Ha
    parameter[49] = 0x88
    parameter[50] = 0x01

    parameter[51] = 20
    parameter[52] = Lb
    parameter[53] = Hb
    parameter[54] = 0x88
    parameter[55] = 0x01

    parameter[56] = 12
    parameter[57] = Lc
    parameter[58] = Hc
    parameter[59] = 0x88
    parameter[60] = 0x01


def kepiting_kiri_2(A, alpa, direksi):
    sinus_pattern5(A, 7, 3, 6, alpa, 0, direksi)
    invers_kinematik5(var_x5, var_y5, var_z5)
    rad_to_deg5()
    kiri2()
    translated(outdeg15)
    translatee(outdeg25)  # +1
    translatef(outdeg35)

    parameter[61] = 13
    parameter[62] = Ld
    parameter[63] = Hd
    parameter[64] = 0x88
    parameter[65] = 0x01

    parameter[66] = 14
    parameter[67] = Le
    parameter[68] = He
    parameter[69] = 0x88
    parameter[70] = 0x01

    parameter[71] = 15
    parameter[72] = Lf
    parameter[73] = Hf
    parameter[74] = 0x88
    parameter[75] = 0x01


def kepiting_kiri_3(A, alpa, direksi):
    sinus_pattern6(A, 7, 3, 6, alpa, 1, direksi)
    invers_kinematik6(var_x6, var_y6, var_z6)
    rad_to_deg6()
    kiri3()
    translateg(outdeg16)
    translateh(outdeg26)
    translatei(outdeg36)

    parameter[76] = 19
    parameter[77] = Lg
    parameter[78] = Hg
    parameter[79] = 0x88
    parameter[80] = 0x01

    parameter[81] = 17
    parameter[82] = Lh
    parameter[83] = Hh
    parameter[84] = 0x88
    parameter[85] = 0x01

    parameter[86] = 18
    parameter[87] = Li
    parameter[88] = Hi
    parameter[89] = 0x88
    parameter[90] = 0x01


def berdiri(x, y, z):
    Kanan_depan(x, y, z)
    Kanan_tengah(x, y, z)
    Kanan_belakang(x, y, z)
    Kiri_depan(x, y, z)
    Kiri_tengah(x, y, z)
    Kiri_belakang(x, y, z)
    packet_kirim(AX_WRITE, 4, 18)
    # print "berdiri"


def maju():
    kanan_1(5.5, 300, 0)  # 315(4,125,1)
    kanan_2(5.5, 90, 1)  # (4,90,1)
    kanan_3(5.5, 60, 1)  # (4,55,1)
    kiri_1(5.5, 60, 1)  # (4,55,1)
    kiri_2(5.5, 90, 1)  # (4,90,1)
    kiri_3(5.5, 290, 0)  # 300(4,125,1)
    packet_kirim(AX_WRITE, 4, 18)
    print
    "maju"


def majukecil():
    kanan_1(2.75, 120, 1)  # 315(4,125,1)
    kanan_2(2.75, 90, 1)  # (4,90,1)
    kanan_3(2.75, 60, 1)  # (4,55,1)
    kiri_1(2.75, 60, 1)  # (4,55,1)
    kiri_2(2.75, 90, 1)  # (4,90,1)
    kiri_3(2.75, 120, 1)  # 300(4,125,1)
    packet_kirim(AX_WRITE, 4, 18)


# print "maju"


def mundur():
    kanan_1(3, 300, 1)
    kanan_2(3, 90, 0)
    kanan_3(3, 60, 0)
    kiri_1(3, 60, 0)
    kiri_2(3, 90, 0)
    kiri_3(3, 320, 1)
    packet_kirim(AX_WRITE, 4, 18)
    print
    "mundur"


def belokkiri():
    kanan_1(4, 90, 1)  # 6
    kanan_2(4, 90, 1)  # 2
    kanan_3(4, 90, 1)  # 6
    kiri_1(1, 30, 1)  # 1
    kiri_2(2, 90, 1)  # 0
    kiri_3(3, 130, 1)  # 2
    packet_kirim(AX_WRITE, 4, 18)


# print "belokkiri"

def belokkanan():
    kanan_1(1, 130, 1)
    kanan_2(0.75, 90, 1)
    kanan_3(3, 30, 1)  # 2
    kiri_1(4, 90, 1)  # 6
    kiri_2(4, 90, 1)  # 1
    kiri_3(4, 90, 1)
    packet_kirim(AX_WRITE, 4, 18)


# print"belokkanan"

def jalan(left, right):
    kanan_1(right, 300, 0)  # 315
    kanan_2(right, 90, 1)  # 3.5
    kanan_3(right, 60, 1)
    kiri_1(left, 60, 1)
    kiri_2(left, 90, 1)  # 3.5
    kiri_3(left, 300, 0)
    packet_kirim(AX_WRITE, 4, 18)


def kepiting_kanan():
    kepiting_kanan_1(3, 30, 0)
    kepiting_kanan_2(2, 180, 0)
    kepiting_kanan_3(3, 150, 1)
    kepiting_kiri_1(3, 150, 1)
    kepiting_kiri_2(2, 0, 1)
    kepiting_kiri_3(3, 30, 0)
    packet_kirim(AX_WRITE, 4, 18)


def kepiting_kiri():
    kepiting_kanan_1(3, 30, 1)
    kepiting_kanan_2(2, 180, 1)
    kepiting_kanan_3(3, 150, 0)
    kepiting_kiri_1(3, 150, 0)
    kepiting_kiri_2(2, 0, 0)
    kepiting_kiri_3(3, 30, 1)
    packet_kirim(AX_WRITE, 4, 18)


def putarkiri_sedang():
    kanan_1(4, 90, 1)  # 6
    kanan_2(4, 90, 1)  # 2
    kanan_3(4, 90, 1)  # 6
    kiri_1(4, 90, 0)  # 1
    kiri_2(4, 90, 0)  # 0
    kiri_3(4, 90, 0)  # 2
    packet_kirim(AX_WRITE, 4, 18)


def putarkanan_sedang():
    kanan_1(4, 90, 0)  # 6
    kanan_2(4, 90, 0)  # 2
    kanan_3(4, 90, 0)  # 6
    kiri_1(4, 90, 1)  # 1
    kiri_2(4, 90, 1)  # 0
    kiri_3(4, 90, 1)  # 2
    packet_kirim(AX_WRITE, 4, 18)


def tengok():
    kanan_1(4, 90, 1)  # 6
    kanan_2(4, 90, 0)  # 2
    kanan_3(4, 90, 1)  # 6
    kiri_1(4, 90, 1)  # 1
    kiri_2(4, 90, 0)  # 0
    kiri_3(4, 90, 1)  # 2
    packet_kirim(AX_WRITE, 4, 18)


def putarkanan_kecil():
    kanan_1(1.5, 90, 0)  # 6
    kanan_2(1.5, 90, 0)  # 2
    kanan_3(1.5, 90, 0)  # 6
    kiri_1(1.5, 90, 1)  # 1
    kiri_2(1.5, 90, 1)  # 0
    kiri_3(1.5, 90, 1)  # 2
    packet_kirim(AX_WRITE, 4, 18)


def putar_kanankecilx():
    kanan_1(1, 90, 0)  # 6
    kanan_2(1, 90, 0)  # 2
    kanan_3(1, 90, 0)  # 6
    kiri_1(1, 90, 1)  # 1
    kiri_2(1, 90, 1)  # 0
    kiri_3(1, 90, 1)  # 2
    packet_kirim(AX_WRITE, 4, 18)


def putarkiri_kecil():
    kanan_1(1.5, 90, 1)  # 6
    kanan_2(1.5, 90, 1)  # 2
    kanan_3(1.5, 90, 1)  # 6
    kiri_1(1.5, 90, 0)  # 1
    kiri_2(1.5, 90, 0)  # 0
    kiri_3(1.5, 90, 0)  # 2
    packet_kirim(AX_WRITE, 4, 18)


def putar_kirikecilx():
    kanan_1(1, 90, 1)  # 6
    kanan_2(1, 90, 1)  # 2
    kanan_3(1, 90, 1)  # 6
    kiri_1(1, 90, 0)  # 1
    kiri_2(1, 90, 0)  # 0
    kiri_3(1, 90, 0)  # 2
    packet_kirim(AX_WRITE, 4, 18)


def main_coba0():
    berdiri(0, 6, 7)
    print
    "berdiri"
    while True:
        global sudut
        sudut = 0
        while (sudut <= 360):
            sudut += 8
            # cekUV()
            # thermal1102()
            # maju()
            mundur()


# ------------------inisialiasi---------------#
clpwm = 0
crpwm = 0


def get_velocity():
    global lpwm
    global rpwm
    lpwm = clpwm
    rpwm = crpwm


def diamCode():
    global clpwm
    global crpwm
    crpwm = -1
    clpwm = -1


def mundurCode():
    global crpwm
    global clpwm
    crpwm = -2
    clpwm = -2


def belokkananCode():
    global clpwm
    global crpwm
    clpwm = -3
    crpwm = -3


def putarkanansedangCode():
    global clpwm
    global crpwm
    clpwm = -5
    crpwm = -5


def putarkanankecilCode():
    global clpwm
    global crpwm
    clpwm = -4
    crpwm = -4


#	print "putarkanankecilCode"
def kepitingkananCode():
    global clpwm
    global crpwm
    clpwm = -6
    crpwm = -6


def belokkiriCode():
    global clpwm
    global crpwm
    clpwm = -7
    crpwm = -7


def putarkirisedangCode():
    global clpwm
    global crpwm
    clpwm = -9
    crpwm = -9


def putarkirikecilCode():
    global clpwm
    global crpwm
    clpwm = -8
    crpwm = -8


def kepitingkiriCode():
    global clpwm
    global crpwm
    clpwm = -10
    crpwm = -10


def majuCode():
    global clpwm
    global crpwm
    clpwm = -11
    crpwm = -11


def putarkanansedangspCode():
    global clpwm
    global crpwm
    clpwm = -12
    crpwm = -12


def putarkanankecilspCode():
    global clpwm
    global crpwm
    clpwm = -13
    crpwm = -13


def mundurlambatCode():
    global clpwm
    global crpwm
    clpwm = -14
    crpwm = -14


def kepitingkirispCode():
    global clpwm
    global crpwm
    clpwm = -15
    crpwm = -15


def tengokCode():
    global clpwm
    global crpwm
    clpwm = -16
    crpwm = -16


def majuspecialCode():
    global clpwm
    global crpwm
    clpwm = -17
    crpwm = -17


def putar_kanankecilxCode():
    global clpwm
    global crpwm
    clpwm = -18
    crpwm = -18


def putar_kirikecilxCode():
    global clpwm
    global crpwm
    clpwm = -19
    crpwm = -19


def gerak():
    global sudut
    sudut = 0
    # print "sudut",sudut
    while (sudut <= 360):
        sudut += 22.5
        # print "sudut",sudut
        if ((sudut == 270) or (sudut == 360)):

            get_velocity()
        elif ((rpwm == -1) and (lpwm == -1)):

            berdiri(0, 6, 7)

        elif ((rpwm == -2) and (lpwm == -2)):

            mundur()
            time.sleep(0.01)

        elif ((rpwm == -3) and (lpwm == -3)):

            belokkanan()
            time.sleep(0.002)
        elif ((lpwm == -4) and (rpwm == -4)):
            putarkanan_kecil()
        # time.sleep(0.05)
        elif ((rpwm == -5) and (lpwm == -5)):

            putarkanan_sedang()
        # time.sleep(0.009)
        elif ((rpwm == -6) and (lpwm == -6)):

            kepiting_kanan()
        elif ((rpwm == -7) and (lpwm == -7)):

            belokkiri()
            time.sleep(0.002)

        elif ((lpwm == -8) and (rpwm == -8)):
            putarkiri_kecil()
        #	time.sleep(0.09)

        elif ((rpwm == -9) and (lpwm == -9)):

            putarkiri_sedang()
        # time.sleep(0.002)
        elif ((rpwm == -10) and (lpwm == -10)):

            kepiting_kiri()

        elif ((rpwm == -11) and (lpwm == -11)):
            maju()
        # time.sleep(0.009)
        elif ((rpwm == -12) and (lpwm == -12)):
            putarkanan_sedang()
        # time.sleep(0.009)
        elif ((rpwm == -13) and (lpwm == -13)):
            putarkiri_sedang()
        #	time.sleep(0.009)
        # print "putar kiri spesial"
        elif ((rpwm == -14) and (lpwm == -14)):
            mundur()
            time.sleep(0.04)
        elif ((rpwm == -15) and (lpwm == -15)):
            kepiting_kiri()
        # time.sleep(0.004)
        elif ((rpwm == -16) and (lpwm == -16)):
            tengok()
            time.sleep(0.09)
        # print "goyang inul"
        elif ((rpwm == -17) and (lpwm == -17)):

            maju()
            time.sleep(0.005)
        elif ((rpwm == -18) and (lpwm == -18)):
            putar_kanankecilx()
        elif ((rpwm == -19) and (lpwm == -19)):
            putar_kirikecilx()
        else:

            jalan(lpwm, rpwm)
    #	time.sleep(0.0005)
    # print"jalan"


def normalisasi_velocity():
    global clpwm
    global crpwm

    clpwm = lpwm
    crpwm = rpwm


# //------------ Program------------------------//
def ultrakisa():
    ser.flushInput()
    global b1
    ser.write('A')
    var = ser.readline()
    b1 = int(var)
    type(b1)


#	print b1
def ultrakide():
    ser.flushInput()
    global b2
    ser.write('B')
    var = ser.readline()
    b2 = int(var)
    type(b2)


def ultradeki():
    ser.flushInput()
    global b3
    ser.write('C')
    var = ser.readline()
    b3 = int(var)
    type(b3)


def ultradeka():
    ser.flushInput()
    global b4
    ser.write('D')
    var = ser.readline()
    b4 = int(var)
    type(b4)


def ultrakade():
    ser.flushInput()
    global b5
    ser.write('E')
    var = ser.readline()
    b5 = int(var)
    type(b5)


def ultrakasa():
    ser.flushInput()
    global b6
    ser.write('F')
    var = ser.readline()
    b6 = int(var)
    type(b6)


def ultrabeka():
    ser.flushInput()
    global b8
    ser.write('G')
    var = ser.readline()
    b8 = int(var)
    type(b8)


def batasdepankosong():
    global hasdeka
    global hasdeki
    global b3
    global b4
    hasdeka = 0
    hasdeki = 0
    for i in range(0, 2):
        ultradeka()
        ultradeki()
        if hasdeki > b3:
            hasdeki = hasdeki
        elif hasdeki <= b3:
            hasdeki = b3
        if hasdeka > b4:
            hasdeka = hasdeka
        elif hasdeka <= b4:
            hasdeka = b4


#	print"hasdeka",hasdeka,"hasdeki",hasdeki
GPIO.setup("P9_15", GPIO.IN)


def infrared5():
    global balikinfra
    GPIO.setup("P9_15", GPIO.IN)
    balikinfra = GPIO.input("P9_15")
    print
    "ir_balik", balikinfra


def ultraruangberangkat2():
    global balikultradepandekat2
    global ddd
    global has8
    global arah
    global balikinfra
    arah_cmps()
    infrared5()
    time.sleep(0.2)
    has8 = 0
    if ((arah == 3) or (arah == 1)):
        diamCode()
        for i in range(0, 15):
            ultrabeka()
            if has8 > b8:
                has8 = has8
            elif has8 <= b8:
                has8 = b8
        # time.sleep(0.5)
        print
        "has8=", has8
        if (has8 <= 42):
            if (balikinfra == 0):
                balikultradepandekat2 = 1
                print
                "balik=1"
            elif (balikinfra == 1):
                balikultradepandekat2 = 0
                print
                "balik=0"
        elif (has8 > 42):
            balikultradepandekat2 = 0
            print
            "balik=0"
    elif ((arah != 3) or (arah != 1)):
        print
        "home aman"
    ddd = ddd + 1


#	print "has8",has8
#	print "balikultradepandekat2",balikultradepandekat2
#	print"konting",ddd
# def
def cekstartmana():
    #	print"aaa"
    global b1
    global b2
    global b3
    global b4
    global b5
    global b6
    global b8
    global has10
    global has20
    global has30
    global has40
    global has50
    global has60
    global has80
    global siaga1
    global siaga2
    has10 = 0
    has20 = 0
    has30 = 0
    has40 = 0
    has50 = 0
    has60 = 0
    has80 = 0
    for i in range(0, 9):
        # print"bbbbbbbb"
        ultrakide()
        ultradeka()
        ultradeki()
        ultrakade()
        ultrakisa()
        ultrakasa()
        ultrabeka()
        if has10 > b1:
            has10 = has10
        elif has10 <= b1:
            has10 = b1
        if has20 > b2:
            has20 = has20
        elif has20 <= b2:
            has20 = b2
        if has30 > b3:
            has30 = has30
        elif has30 <= b3:
            has30 = b3
        if has40 > b4:
            has40 = has40
        elif has40 <= b4:
            has40 = b4
        if has50 > b5:
            has50 = has50
        elif has50 <= b5:
            has50 = b5
        if has60 > b6:
            has60 = has60
        elif has60 <= b6:
            has60 = b6
        if has80 > b8:
            has80 = has80
        elif has80 <= b8:
            has80 = b8
    print
    has10, has20, has30, has40, has50, has60, has80

    if ((has10 < 14) or (has20 < 10) or (has30 < 10) or (has40 < 10) or (has50 < 10) or (has60 < 10) or (has80 < 10)):
        print
        "robotsiaga1 pengecekan"
        if ((has10 < 14) and (has20 >= 10) and (has30 >= 10) and (has40 >= 10) and (has50 >= 10) and (has60 >= 10) and (
                has80 >= 10)):
            print
            "siaga 2 dari pengecekan siaga 1 ke 1"
            siaga1 = 0
            siaga2 = 1
        elif ((has10 >= 14) and (has20 < 10) and (has30 >= 10) and (has40 >= 10) and (has50 >= 10) and (
                has60 >= 10) and (has80 >= 10)):
            print
            "siaga 2 dari pengecekan siaga 1 ke 2"
            siaga1 = 0
            siaga2 = 1
        elif ((has10 >= 14) and (has20 >= 10) and (has30 < 10) and (has40 >= 10) and (has50 >= 10) and (
                has60 >= 10) and (has80 >= 10)):
            print
            "siaga 2 dari pengecekan siaga 1 ke 3"
            siaga1 = 0
            siaga2 = 1
        elif ((has10 >= 14) and (has20 >= 10) and (has30 >= 10) and (has40 < 10) and (has50 >= 10) and (
                has60 >= 10) and (has80 >= 10)):
            print
            "siaga 2 dari pengecekan siaga 1 ke 4"
            siaga1 = 0
            siaga2 = 1
        elif ((has10 >= 14) and (has20 >= 10) and (has30 >= 10) and (has40 >= 10) and (has50 < 10) and (
                has60 >= 10) and (has80 >= 10)):
            print
            "siaga 2 dari pengecekan siaga 1 ke 5"
            siaga1 = 0
            siaga2 = 1
        elif ((has10 >= 14) and (has20 >= 10) and (has30 >= 10) and (has40 >= 10) and (has50 >= 10) and (
                has60 < 10) and (has80 >= 10)):
            print
            "siaga 2 dari pengecekan siaga 1 ke 6"
            siaga1 = 0
            siaga2 = 1
        elif ((has10 >= 14) and (has20 >= 10) and (has30 >= 10) and (has40 >= 10) and (has50 >= 10) and (
                has60 >= 10) and (has80 < 10)):
            print
            "siaga 2 dari pengecekan siaga 1 ke 7"
            siaga1 = 0
            siaga2 = 1
        elif ((has10 >= 14) and (has20 >= 10) and (has30 < 10) and (has40 < 10) and (has50 >= 10) and (
                has60 >= 10) and (has80 >= 10)):
            print
            "siaga 2 dari pengecekan siaga 1 ke 8"
            siaga1 = 0
            siaga2 = 1
        else:
            print
            "alhamdulillah lulus siaga 1"
            siaga1 = 1
            siaga2 = 0
    elif ((has10 >= 14) and (has20 >= 10) and (has30 >= 10) and (has40 >= 10) and (has50 >= 10) and (has60 >= 10) and (
            has80 >= 10)):
        print
        "robot masuk siaga 2"
        siaga1 = 0
        siaga2 = 1


# GPIO.setup("P9_26", GPIO.IN,pull_up_down=GPIO.PUD_UP)#TOMBOLSTART P9_26

def tombol_start():
    global start
    GPIO.input("P8_7")
    if ((GPIO.input("P8_7")) == 1):
        start = 1
        sound = 1
        print
        "start dipencet"
    elif ((GPIO.input("P8_7")) == 0):
        start = 0
        sound = 0
        print
        "tombol mati"


def cek_start():
    global passtart
    global countnilai
    global b4
    global b3
    global b6
    global keluarnyasar
    global countnilaiy
    global has677
    global has866
    jauhsisi()
    ultradeka()
    ultradeki()
    # print "sensdeka :",sensdeka4
    # print "sensdeki :",sensdeki4
    if ((b3 > 16) or (b4 > 16)):
        print
        "======gada depan"
        ultradeki()
        #	ultra_deka(0.03)
        while (((b3 > 20) or (b4 > 20)) and (countnilai < 25)):
            putarkanansedangCode()
            print
            "putar kanan"
            ultradeki()
            ultradeka()
            countnilai += 1
            # print "data kanan :",senskanan4,"countnilai :",countnilai
            # elif (b1<9):

            # print "==========================kanan deket"
            print
            "b3", b3, "b4", b4
        # print "counilai",countnilai
    if ((b3 <= 7) or (b4 <= 7)):
        print
        "ada depan"
        for i in range(0, 7):
            ultrakasa()
            if (has677 > b6):
                has677 = has677
            elif (has677 <= b6):
                has677 = b6
        while (((has677 > 8) or (has866 > 10)) and (countnilaiy < 10)):
            print
            "putar kiri"
            putarkirikecilCode()
            # ultra_beka(0.03)
            # for i in range(0,5):
            #	ultra_kanan(0)
            #	if (has677 > b6):
            #       	has677=has677
            #	elif (has677 <= b6):
            #       	has677=b6
            for i in range(0, 5):
                ultrakasa()
                ultrabeka()
                if (has677 > b6):
                    has677 = has677
                elif (has677 <= b6):
                    has677 = b6
                if (has866 > b8):
                    has866 = has866
                elif (has866 <= b8):
                    has866 = b8
            # if (b6<=9):
            countnilaiy += 1
            print
            "countnilaiy", countnilaiy
        # print "data beka :",sensbeka4,"beki :",sensbeki4,"countnilai :",countnilai
    diamCode()
    time.sleep(0.125)


# keluarnyasar=1
#	print "======================================================================hampir siap jalan"
def cekstart2():
    global has35
    global has45
    global kontinga
    global konmajju
    global kontinggg
    global has100
    global has600
    global kontinggzz
    global b1
    global b6
    global belumkeluarhome
    has35 = 0
    has45 = 0
    kontinga = 0
    #	print" masuk cekstart2"
    for i in range(0, 25):  # 25
        ultradeka()
        ultradeki()
        if has35 > b3:
            has35 = has35
        elif has35 <= b3:
            has35 = b3
        if has45 > b4:
            has45 = has45
        elif has45 <= b4:
            has45 = b4
        # print "has35",has35,"has45",has45
        #		print"kontingputarawal",kontinga
        kontinga = kontinga + 1
    while ((has35 > 20) and (has45 > 20)):
        has35 = 0
        has45 = 0
        putarkanansedangCode()
        print
        "putar kanan di home"
        for i in range(0, 5):
            ultradeka()
            ultradeki()
            if has35 > b3:
                has35 = has35
            elif has35 <= b3:
                has35 = b3
            if has45 > b4:
                has45 = has45
            elif has45 <= b4:
                has45 = b4
        print
        "has35", has35, "has45", has45
    while (((has35 > 14) and (has45 > 14)) and (konmajju <= 28)):  # batasdepan12cmsebelumbfareza
        has35 = 0
        has45 = 0
        majuspecialCode()
        # time.sleep(0.00005)
        for i in range(0, 5):
            ultradeka()
            ultradeki()
            if has35 > b3:
                has35 = has35
            elif has35 <= b3:
                has35 = b3
            if has45 > b4:
                has45 = has45
            elif has45 <= b4:
                has45 = b4
        konmajju = konmajju + 1
    #		print"konmajju",konmajju
    if (konmajju <= 120):
        ultrakasa()
        while (b6 > 6):
            # print "putar kiri"
            putarkirisedangCode()
            # ultra_beka(0.03)
            ultrakasa()
    #		print"tresing1"
    #	print"belumkeluarhome:",belumkeluarhome
    #	#print "kontinggg",kontinggg
    #	print"proximana1",proximana1,"proximana2",proximana2,"proximana3",proximana3
    #	print "siap jalan222"
    diamCode()
    time.sleep(0.3)


# =================================pompa==================================
GPIO.setup("P9_12", GPIO.OUT)


def air1():
    GPIO.output("P9_12", GPIO.HIGH)
    print
    "air======================================"


def air2():
    GPIO.output("P9_12", GPIO.LOW)
    print
    "matiin air"


def air3():
    air1()
    time.sleep(1)
    air2()
    time.sleep(1)


# ==========================================thermal======================
def thermalbus1():
    global thermal1
    thermal1 = busth.read_byte_data(addressth, reg1)
    return thermal1


def thermalbus2():
    global thermal2
    thermal2 = busth.read_byte_data(addressth2, reg2)
    return thermal2


def thermalbus3():
    global thermal3
    thermal3 = busth.read_byte_data(addressth3, reg3)
    return thermal3


def thermalreg1():
    global thermal123
    global reg1

    reg1 = 0x00

    thermal123 = thermalbus1()
    revision1 = thermal123

    reg1 = 0x01

    thermal123 = thermalbus1()
    ambient1 = thermal123

    for i in range(0, 8):
        reg1 = reg1 + 1
        thermalbus1()
        pixel1[i] = thermal1
    time.sleep(0.004)
    print
    "thermal1 :", pixel1[0], pixel1[1], pixel1[2], pixel1[3], pixel1[4], pixel1[5], pixel1[6], pixel1[7]


def thermalreg2():
    global thermal321
    global reg2

    reg2 = 0x00

    thermal321 = thermalbus2()
    revision2 = thermal321

    reg2 = 0x01

    thermal321 = thermalbus2()
    ambient2 = thermal321

    for j in range(0, 8):
        reg2 = reg2 + 1
        thermalbus2()
        pixel2[j] = thermal2
    time.sleep(0.004)
    print
    "thermal2 :", pixel2[0], pixel2[1], pixel2[2], pixel2[3], pixel2[4], pixel2[5], pixel2[6], pixel2[7]


def thermalreg3():
    global thermal111
    global reg3

    reg3 = 0x00

    thermal111 = thermalbus3()
    revision3 = thermal111

    reg3 = 0x01

    thermal111 = thermalbus3()
    ambient3 = thermal111

    for i in range(0, 8):
        reg3 = reg3 + 1
        thermalbus3()
        pixel3[i] = thermal3
    time.sleep(0.004)
    print
    "thermal3 :", pixel3[0], pixel3[1], pixel3[2], pixel3[3], pixel3[4], pixel3[5], pixel3[6], pixel3[7]


def thermal1102():
    global misiapi
    global skipperataan
    global apikanan
    global apikiri
    global flame1
    global flame2
    global flame3
    global cono
    #	cekUV(3)
    if (api == 1):
        print
        "ledlilin AKTIF"
        # diamCode()
        # time.sleep(0.3)
        thermalreg1()
        thermalreg2()
        thermalreg3()
        flamesen1()
        flamesen2()
        flamesen3()
        ledlilin()
        if ((pixel1[0] >= 60) or (pixel1[1] >= 70) or (pixel1[2] >= 70) or (pixel1[3] >= 70) or (pixel1[4] >= 70) or (
                pixel1[5] >= 70) or (pixel1[6] >= 70) or (pixel1[7] >= 70) and (flame3 == 1)):
            print
            "api ada di depan"
            # air3()
            # time.sleep(1)
            # tengokCode()
            # time.sleep(0.2)
            cekUV(3)
            print
            "cek uv 1 nilai api:", api
            if (api == 1):
                print
                "masih ada api di depan"
                tengokCode()
                time.sleep(0.2)
                cekUV(3)
                countkipas = 0
                while ((api == 1) and (countkipas < 1)):
                    tengokCode()
                    time.sleep(0.2)
                    air3()
                    cekUV(3)
                    countkipas += 1
                    print
                    "COUNTKIPAS ", countkipas
                time.sleep(0.004)
                print
                "cek uv 3 nilai api:", api
                cono = 0
                if (api == 0):
                    diamCode()
                    cekUV(3)
                    time.sleep(0.5)
                    if (api == 0):
                        print
                        "api di depan udah mati1,putaar"
                        misiapi = 1
                        kanputarbalik()
                elif (api == 1):
                    while (cono < 3):  # while
                        mundurCode()
                        time.sleep(0.1)
                        cono += 1
                        diamCode()
                        time.sleep(0.2)
                        if (cono == 2):
                            tengokCode()
                            time.sleep(0.125)
                            air3()
                        diamCode()
                        time.sleep(0.8)
                        cekUV(3)
                        time.sleep(2)
                        if (api == 0):
                            print
                            "api di depanmati 2,putarbalik"
                            misiapi = 1
                            kanputarbalik()
                    diamCode()
                    time.sleep(0.5)
                    cekUV(3)
                    time.sleep(1)
                    if (api == 0):
                        print
                        "api di depan udah mati 3,putarbalik"
                        misiapi = 1
                        kanputarbalik()

            elif (api == 0):
                diamCode()
                cekUV(3)
                time.sleep(0.5)
                if (api == 0):
                    print
                    "api di depan udah mati0 ====,putarbalik"
                    misiapi = 1
                    kanputarbalik()

        if (((pixel2[0] >= 60) or (pixel2[1] >= 35) or (pixel2[2] >= 35) or (pixel2[3] >= 35) or (pixel2[4] >= 35) or (
                pixel2[5] >= 35) or (pixel2[6] >= 35) or (pixel2[7] >= 35)) or (flame2 == 1)):
            print
            "api ada dikiri, flamekiri", flame2
            putther3()
        if (((pixel3[0] >= 50) or (pixel3[1] >= 35) or (pixel3[2] >= 35) or (pixel3[3] >= 35) or (pixel3[4] >= 35) or (
                pixel3[5] >= 35) or (pixel3[6] >= 35) or (pixel3[7] >= 35)) or (flame1 == 1)):
            print
            "api di kanan, flamekanan", flame1
            putther()
        flame1 = 0
        flame2 = 0


#########################################


def jauhsisi():
    global countingka
    global countingki
    ultrakasa()
    global b1
    global b6
    countingka = 0
    countingki = 0
    if (b6 <= 7):
        while ((b6 < 8) and (countingka < 80)):
            kepitingkiriCode()  # lambat
            ultrakasa()
            countingka += 1
            print
            "kepitingkiri"
    if (b1 <= 9):
        while ((b1 < 9) and (countingki < 12)):
            kepitingkananCode()  # lambat
            ultrakisa()
            countingki += 1
            print
            "kepitingkanan"


def infrared4():
    global ir_compas
    GPIO.setup("P8_11", GPIO.IN)
    ir_compas = GPIO.input("P8_11")
    print
    "ir_compas", ir_compas


pembandingdepan2 = 0
mmm = 0


def infracompas():
    global mmm
    global arah
    global pembandingdepan2
    global balikultradepandekat2
    global ir_compas
    diamCode()
    infrared4()
    if (ir_compas == 1):
        pembandingdepan2 = 0
    elif (ir_compas == 0):
        pembandingdepan2 = 1

    mmm = mmm + 1
    time.sleep(0.5)


#       print "ir_compas",ir_compas
#      print "pembandingdepan2",pembandingdepan2
# print"konting mmm",mmm
infrakompas1 = 0
infrakompas2 = 0
infrakompas3 = 0
kon_ir = 1


def nyasar1():
    global garisnyasar1
    global garisnyasar2
    global garisnyasar3
    global kon_ir
    global infrakompas1
    global infrakompas2
    global infrakompas3
    global arah
    global konnyasar1
    global garis
    global garisapi
    global nyasar
    global arahkeluar
    global pembandingdepan2
    if (keluarnyasar == 0):
        print
        "belum keluar ruangan ya jadi fungsi nyasar belum dipanggil"
    elif (keluarnyasar == 1):
        # print"konnyasar1",konnyasar1
        # print"kon_ir",kon_ir
        infracompas()
        arah_cmps()
        if (konnyasar1 == 1):
            garisnyasar1 = arah
        elif (konnyasar1 == 2):
            garisnyasar2 = arah
        elif (konnyasar1 == 3):
            garisnyasar3 = arah
        if (kon_ir == 1):
            infrakompas1 = pembandingdepan2
        elif (kon_ir == 2):
            infrakompas2 = pembandingdepan2
        elif (kon_ir == 3):
            infrakompas3 = pembandingdepan2

        if ((garisnyasar1 == garisnyasar3) and (infrakompas1 == infrakompas3)):
            nyasar = 1
            konnyasar1 = 0
            kon_ir = 0
            garisnyasar1 = 0
            garisnyasar2 = 0
            garisnyasar3 = 0
            infrakompas1 = 0
            infrakompas2 = 0
            infrakompas3 = 0
        #			print "tadi teh nyasar "
        if (konnyasar1 == 3):
            konnyasar1 = 0
        if (kon_ir == 3):
            kon_ir = 0

        konnyasar1 = konnyasar1 + 1
        kon_ir = kon_ir + 1


#		print"garisnyasar1:",garisnyasar1,"garisnyasar2:",garisnyasar2,"garisnyasar3",garisnyasar3
#		print"infrakompas1",infrakompas1,"infrakompas2",infrakompas2,"infrakompas3",infrakompas3

def kanputarbalik():
    global countning
    global b3
    global b4
    ultradeka()
    ultradeki()
    print
    "kanputar"
    while (((b3 > 7) and (b4 > 7)) and (countning < 22)):
        putarkanansedangCode()
        time.sleep(0.09)  # print "putar kanan"
        ultradeki()
        ultradeka()
        countning += 1
        print
        countning
    ledlilin()


# diamCode()
# time.sleep(1)


####============ Pemanggilan Compass ===========###
def bearing255():
    bear = buscm.read_byte_data(addresscmps, 1)
    return bear


def cmps():
    # print "a"
    global bear255
    bear255 = bearing255()
    time.sleep(0.0004)


#	print "nilai kompas ",bear255

def arah_cmps():
    global arah
    cmps()
    #	print "bear255 = ",bear255,arah

    if ((bear255 >= 1) and (bear255 < 75)):
        arah = 2
    elif ((bear255 >= 204) and (bear255 < 258)):
        arah = 2
    elif ((bear255 >= 75) and (bear255 < 111)):
        arah = 1
    elif ((bear255 >= 111) and (bear255 < 140)):  # 149-217
        arah = 4
    else:
        arah = 3
    print
    "arah : ", arah


# =========================flame===========================================
GPIO.setup("P9_18", GPIO.IN)
GPIO.setup("P9_23", GPIO.IN)
GPIO.setup("P8_16", GPIO.IN)


def flamesen1():
    print
    "a"
    global flame1
    if (GPIO.input("P9_18") == 0):
        flame1 = 1
        print
        "==============================adakanan"


def flamesen2():
    print
    "ac"
    global flame2
    if (GPIO.input("P9_23") == 0):
        flame2 = 1
        print
        "=============ada2kiri"


#		kesayangan di kiri kena flame kiri"
#	print GPIO.input("P8_26")
def flamesen3():
    print
    "bc"
    global flame3
    if (GPIO.input("P8_16") == 0):
        flame3 = 1
        print
        "=============ada3depan"


def ultraruangbalik2():
    global eee
    global has4
    global arah
    global pembandingdepan
    global balikultradepandekat2
    global ir_pulang
    diamCode()
    infrared3()
    if (ir_pulang == 1):
        pembandingdepan = 0
    #		buzzer()
    elif (ir_pulang == 0):
        pembandingdepan = 1

    eee = eee + 1
    time.sleep(0.5)


def cari_api():
    global api
    global b4
    global b1
    global c_maju
    global c_ul
    global arah
    global infrared_atas
    global infrared_belka
    print
    " cek uviiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii 1"
    flamesen3()
    if (flame3 == 1):
        cekUV(5)
    ultrakisa()
    arah_cmps()
    infrared_belka = GPIO.input("P9_15")
    print
    arah
    if (arah == 4):
        if (infrared_belka == 1):
            ultrakasa()
            while (b6 > 10):
                kepitingkananCode()
                ultrakasa()
            ultrakade()
            while (b5 < 13):
                putarkirikecilCode()
                print
                "putar kiri", b5
                ultrakade()
        c_maju = 0
        c_ul = 0
        ultradeka()
        infrared_atas = (GPIO.input("P8_11"))
        print
        infrared_atas
        while ((infrared_atas == 1) and (c_maju <= 35)):
            majuspecialCode()
            ultradeka()
            infrared_atas = (GPIO.input("P8_11"))
            c_maju = c_maju + 1
            print
            "cmaju", c_maju
        cekUV(4)
        time.sleep(0.2)
        if (api == 0):
            print
            "cek uviiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii21"
            ultrakade()
            while (b5 < 30):
                putarkirikecilCode()
                print
                "putar kiri", b5
                ultrakade()
            cekUV(5)
            diamCode()
            time.sleep(0.1)
            ultrakade()
            print
            "!", b5
            while ((b5 > 14) and (api == 0)):
                putarkanankecilCode()
                print
                "putarkanan", b5
                ultrakade()
            print
            "UV wes dicekkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk11111111"
            if (api == 1):
                ultrakisa()
                if (b1 > 19):
                    ultrakade()
                    while (b5 > 13):
                        putarkanankecilCode()
                        ultrakade()
                elif (b1 < 19):
                    ultrakide()
                    while (b2 > 17):
                        putarkirikecilCode()
                        ultrakide()
            elif (api == 0):
                diamCode()
                time.sleep(0.1)
                ultrakisa()
                if (b1 > 19):
                    ultrakade()
                    while (b5 > 17):
                        putarkanankecilCode()
                        ultrakade()
                        print
                        "kan", b5
                elif (b1 < 19):
                    ultrakide()
                    while (b2 > 19):
                        putarkirikecilCode()
                        ultrakide()
                print
                "cek finish1"
        elif ((api == 1) and (infrared_belka == 1)):
            majuCode()
            time.sleep(0.4)
            print
            "cek uviiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii ada api 9999999999999999999991111111111"
            diamCode()
            time.sleep(0.15)
        elif ((api == 1) and (infrared_belka == 0)):
            putarkanankecilCode()
            time.sleep(0.5)
            diamCode()
            time.sleep(0.5)
    elif (arah == 2):
        ultrakisa()
        while (b1 > 16):
            kepitingkiriCode()
            ultrakisa()
        ultrakide()
        while (b2 > 21):
            putarkirikecilCode()
            print
            "putar kiriawal", b2
            ultrakide()
        ultradeka()
        c_maju = 0
        c_ul = 0
        infrared_atas = GPIO.input("P8_11")
        while ((c_ul <= 10) and (c_maju <= 45)):
            majuspecialCode()
            ultradeka()
            print
            b4, "-", c_ul
            if (b4 <= 20):  # arah2 b4>=33
                c_ul = c_ul + 1
            c_maju = c_maju + 1
            print
            "==", c_maju
        c_maju = 0
        c_ul = 0
        cekUV(4)
        if (api == 0):
            print
            "cek uviiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii 22"
            ultrakisa()
            if (b1 < 19):
                diamCode()
                time.sleep(0.1)
                putarkanankecilCode()
                time.sleep(0.9)
                diamCode()
                time.sleep(0.1)
                cekUV(5)
                ultrakide()
                while (b2 > 21):
                    putarkirikecilCode()
                    ultrakide()
            if (api == 0):
                ultrakisa()
                if (b1 < 19):
                    ultrakide()
                    while (b2 < 36):
                        putarkanankecilCode()
                        print
                        "putar kanan"
                        ultrakide()
                    cekUV(5)
                    diamCode()
                    time.sleep(0.1)
                    print
                    "UV wes dicekkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk"
                    if (api == 1):
                        ultrakide()
                        while (b2 > 16):
                            putarkirikecilCode()
                            ultrakide()
                    elif (api == 0):
                        ultrakisa()
                        if (b1 > 50):
                            ultrakade()
                            while (b5 > 24):
                                putarkanankecilCode()
                                ultrakade()
                        elif (b1 < 50):
                            ultrakide()
                            while (b2 > 16):
                                putarkirikecilCode()
                                ultrakide()
                        print
                        "cek finish"

            elif (api == 1):
                ultrakide()
                while (b2 > 23):
                    putarkirikecilCode()
                    ultrakide()
                majuCode()
                time.sleep(0.3)
        elif (api == 1):
            majuCode()
            time.sleep(0.2)
            print
            "cek uviiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii ada api 999999999999999999999"
            #	print"majuuuu"
            # time.sleep(0.8)
            diamCode()
            time.sleep(0.1)

    elif ((arah == 1) or (arah == 3)):
        ultrakisa()
        while (b1 > 10):
            kepitingkiriCode()
            ultrakisa()
        ultrakide()
        while (b2 > 14):
            putarkirikecilCode()
            print
            "putar kiriawal", b2
            ultrakide()
        ultrabeka()
        c_maju = 0
        c_ul = 0
        infrared_atas = GPIO.input("P8_11")
        while ((b8 < 42) or (c_maju <= 30)):
            majuspecialCode()
            ultrabeka()
            print
            b8, "-", c_ul
            if (b8 >= 42):  # arah2 b4>=33
                c_ul = c_ul + 1
            c_maju = c_maju + 1
            print
            "==", c_maju
        cekUV(4)
        if (api == 0):
            print
            "cek uviiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii 22"
            ultrakisa()
            if (b1 < 19):
                diamCode()
                time.sleep(0.1)
                putarkanankecilCode()
                time.sleep(0.4)
                diamCode()
                time.sleep(0.1)
                cekUV(5)
                ultrakide()
                while (b2 > 14):
                    putarkirikecilCode()
                    ultrakide()
            if (api == 0):
                ultrakisa()
                if (b1 > 19):
                    ultrakade()
                    while (b5 < 36):
                        putarkirikecilCode()
                        print
                        "putar kiri"
                        ultrakade()
                    cekUV(5)
                    diamCode()
                    time.sleep(0.1)
                    ultrakade()
                    while (b5 > 24):
                        putarkanankecilCode()
                        print
                        "putarkanan", b5
                        ultrakade()
                    print
                    "UV wes dicekkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk"
                    if (api == 1):
                        if (b1 > 19):
                            ultrakade()
                            while (b5 > 24):
                                putarkanankecilCode()
                                ultrakade()
                        elif (b1 < 19):
                            ultrakide()
                            while (b2 > 24):
                                putarkirikecilCode()
                                ultrakide()
                    elif (api == 0):
                        ultrakisa()
                        if (b1 > 40):
                            ultrakade()
                            while (b5 > 24):
                                putarkanankecilCode()
                                ultrakade()
                        elif (b1 < 40):
                            ultrakide()
                            while (b2 > 24):
                                putarkirikecilCode()
                                ultrakide()
                        print
                        "cek finish"

            elif (api == 1):
                ultrakide()
                while (b2 > 23):
                    putarkirikecilCode()
                    ultrakide()
                majuCode()
                time.sleep(0.3)
        elif (api == 1):
            majuCode()
            time.sleep(0.4)
            print
            "cek uviiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii ada api 999999999999999999999"
            #	print"majuuuu"
            # time.sleep(0.8)
            diamCode()
            time.sleep(0.1)


# ===============================infrared====================================
GPIO.setup("P8_8", GPIO.IN)
GPIO.setup("P8_11", GPIO.IN)
GPIO.setup("P9_15", GPIO.IN)


def infrared1():
    global infrared_bawah
    global infrared_atas
    global infrared_belka
    global jalanmisi
    # GPIO.setup("P8_8",GPIO.IN)#sebenernya ini bAWAH
    # GPIO.setup("P8_9",GPIO.IN)#sebernya ini yang atas

    infrared_atas = (GPIO.input("P8_11"))
    infrared_bawah = (GPIO.input("P8_8"))
    infrared_belka = (GPIO.input("P9_15"))
    print
    infrared_atas, "---", infrared_bawah, "--", infrared_belka
    if ((infrared_atas == 1) and (infrared_bawah == 0) and (infrared_belka == 1)):
        print
        "ANJINGGG!!!!!!kiri"
        # diamCode()
        #		print "diamCode,mundur, putarkiri"
        # time.sleep(0.125)
        mundurCode()
        time.sleep(0.5)
        diamCode()
        time.sleep(0.125)
        putarkirisedangCode()
        time.sleep(1.4)  # 150
    elif ((infrared_atas == 1) and (infrared_bawah == 0) and (infrared_belka == 0)):
        print
        "ANJINGGGG!!!!!!!!!kanan"
        mundurCode()
        time.sleep(0.9)
        diamCode()
        time.sleep(0.125)
        putarkanansedangCode()
        time.sleep(1.4)


GPIO.setup("P9_15", GPIO.IN)


def infrared3():
    global ir_pulang
    # GPIO.setup("P8_11",GPIO.IN)
    ir_pulang = GPIO.input("9_15")


#        print "ir_pulang:",ir_pulang
# print "bawah",infrared_bawah
GPIO.setup("P8_15", GPIO.IN)
soundawal = 0


def sound_act():
    global sound
    global soundawal
    # sound=1
    soundawal = GPIO.input("P8_15")
    if (soundawal == 0):  # 3800:10000konting#4500kaliblomba
        sound = 1
        ledmic()
        print
        "ada suara"
    else:
        sound = 0
        print
        "gada suara"
    time.sleep(0.1)


#	countingsound=0

GPIO.setup("P8_17", GPIO.OUT)  # LEDMIC
GPIO.setup("P8_18", GPIO.OUT)  # LEDLILIN


def ledlilin():
    if (api == 1):
        # while (apiruang==1):
        GPIO.output("P8_18", GPIO.HIGH)
        print
        "nyalaapi"

    else:
        GPIO.output("P8_18", GPIO.LOW)
        print
        "nyalaapioff"


def ledmic():
    for i in range(0, 4):
        GPIO.output("P8_17", GPIO.HIGH)
        time.sleep(0.1)
        print
        "blink mic"
        GPIO.output("P8_17", GPIO.LOW)
        time.sleep(0.1)


# ==================Emergency Brake Limit Switch=======================
GPIO.setup("P8_7", GPIO.IN)


def limitswitch():
    if (GPIO.input("P8_7") == 1):
        print
        "Switch Aktif"
        diamCode()
        time.sleep(1)
        mundurCode()
        time.sleep(0.3)
        if ((susurkanan == 1) and (susurkiri == 0)):
            print
            "putarkiri<<<<<<<"
            putarkirisedangCode()
            time.sleep(0.55)
        elif ((susurkanan == 0) and (susurkiri == 1)):
            print
            "putarkanan>>>>>>"
            putarkanansedangCode()
            time.sleep(0.25)
    else:
        print
        "aman"


def putther3():
    global countapi3
    global countapi6
    global flame1
    global flame2
    global flame3
    flame1 = 0
    flame2 = 0
    flame3 = 0
    print
    "masuk putther3"
    thermalreg1()
    countapi3 = 0
    countapi6 = 0
    cekUV(3)
    while (((pixel1[0] <= 40) and (pixel1[1] <= 40) and (pixel1[2] <= 40) and (pixel1[3] <= 40) and (
            pixel1[4] <= 40) and (pixel1[5] <= 40) and (pixel1[6] <= 40) and (pixel1[7] <= 40)) and (api == 1) and (
                   flame3 == 0)):
        putar_kirikecilxCode()
        time.sleep(0.1)
        flamesen3()
        print
        "putarkiriputther3"
        thermalreg1()
        # countapi3=countapi3+1
        cekUV(3)
        # print countapi3


def putther():
    global countapi36
    global countapi63
    global flame1
    global flame2
    print
    "masuk putther"
    #	print "masuk putther"
    flame1 = 0
    flame2 = 0
    flame3 = 0
    thermalreg1()
    countapi36 = 0
    countapi63 = 0
    cekUV(3)
    while (((pixel1[0] <= 40) and (pixel1[1] <= 40) and (pixel1[2] <= 40) and (pixel1[3] <= 40) and (
            pixel1[4] <= 40) and (pixel1[5] <= 40) and (pixel1[6] <= 40) and (pixel1[7] <= 50)) and (api == 1)):
        putar_kanankecilxCode()
        #           time.sleep(0.1)
        print
        "putarkananputther"
        thermalreg1()
        flamesen3()
        cekUV(3)
        # print "konter 36",countapi36


# flamesen3()
'''	while(((pixel1[0]<=70) and (pixel1[1]<=50)and (pixel1[2]<=55)and (pixel1[3]<=55)and (pixel1[4]<=55)and (pixel1[5]<=55)and (pixel1[6]<=55) and (pixel1[7]<=55))and(api==1)and(flame3==1)):
                majuspecialCode()
     #           time.sleep(0.1)
		print "maju special dikit biar deket api"
                thermalreg1()
                flamesen3()
		cekUV(3)
                print "konter 36",countapi36'''
# 	while(((pixel1[0]<=40) and (pixel1[1]<=40)and (pixel1[2]<=100)and (pixel1[3]<=100)and (pixel1[4]<=100)and (pixel1[5]<100)and (pixel1[6]<=100) and (pixel1[7]<=100))and(api==1)):
#               putarkirikecilCode()
#           time.sleep(0.1)
#              print "putarkananputther"
#             thermalreg1()
#            flamesen3()
#           cekUV(3)
# print "konter 36",countapi36


# ====================UVtron============================================
GPIO.setup("P8_12", GPIO.IN, pull_up_down=GPIO.PUD_UP)


def cekUV(count):
    global countingapi
    global api
    global apiruang
    global ap
    for i in range(0, count):
        if (GPIO.input("P8_12") == 1):
            countingapi = countingapi + 1
        else:
            countingapi = 0
    if (countingapi >= count):
        api = 1
        apiruang = 1
        garisapi = garis
        print
        "ada api"
    else:
        api = 0
        apiruang = 0
        print
        "gk ada api"
    ap = ap + 1
    #		print ap
    return api


# ============================proxi===========================================
ADC.setup()


def aadc():
    print
    "depan", ADC.read("P9_40"), "kide", ADC.read("P9_35"), "belki", ADC.read("P9_38"), "kade", ADC.read(
        "P9_37"), "beka", ADC.read("P9_36")


def infraredruang1():
    global ir_khususruang1
    GPIO.setup("P8_11", GPIO.IN)
    ir_khususruang1 = GPIO.input("P8_11")


def ir_khususruang1keluararah3():
    global kontingirnya
    global arah
    global pembandingdepanruang1
    global balikultradepandekat2
    global ir_pulang
    diamCode()
    infraredruang1()

    if (ir_khususruang1 == 1):
        pembandingdepanruang1 = 0
        # buzzer()
    elif (ir_khususruang1 == 0):
        pembandingdepanruang1 = 1
    kontingirnya = kontingirnya + 1
    time.sleep(0.5)
    #       print "ir_khususruang1",ir_khususruang1
    print
    "pembandingdepanruang1", pembandingdepanruang1


#     print"kontingirnya",kontingirnya
def geserkananruang1():
    global ultrakir1
    global kontingkekiri
    ultrakir1 = 0
    print
    "geser kiri ruang 1"
    for i in range(0, 1):
        ultrakisa()
        if (ultrakir1 >= b1):
            ultrakir1 = ultrakir1
        elif (ultrakir1 <= b1):
            ultrakir1 = b1
    if (ultrakir1 > 16):
        ultrakisa()
        while ((kontingkekiri < 15) or (b1 < 14) and (b2 < 17)):
            kepitingkiriCode()
            ultrakisa()
            kontingkekiri += 1


susurawalan = 14045
keluarhomehusus = 0


def susurawali():
    global arah
    global detect_garis
    global arahawal
    global arahahir
    global nyasar
    global arahpul
    global pulangkhusus
    global susurawalan
    global balikultradepandekat2
    global keluarlah
    global done
    global garis
    global susurkanan
    global susurkiri
    global keluarhomehusus
    global misiapi
    ultraruangberangkat2()
    print
    "susurawalan sebelum", susurawalan
    if (misiapi == 0):
        if (detect_garis == 1):
            if (arahawal == 1) and (balikultradepandekat2 == 0):
                arahpul = 3
                pulangkhusus = 1
                susurawalan = 1  # susurkanan
                keluarlah = 1
            elif (arahawal == 1) and (balikultradepandekat2 == 1):
                print
                " maju dulu langsung susurkiri"
                arahpul = 3
                pulangkhusus = 1
                susurawalan = 4  # maju dulu susurkiri
                keluarlah = 1
                majuhome()
            elif (arahawal == 3) and (balikultradepandekat2 == 0):
                arahpul = 0
                pulangkhusus = 1
                susurawalan = 3  # cari arah pintu asli
                keluarlah = 0
                arahawal = 0
                print
                "cari pintu"
            elif (arahawal == 3) and (balikultradepandekat2 == 1):
                print
                " ruangan khusus untuk boneka"
                arahpul = 0  # 1
                pulangkhusus = 1
                susurawalan = 3  # susurkanan#5
                keluarlah = 0  # 1
                arahawal = 0  # teu aya
                # done=1
            elif (arahawal == 2):
                arahpul = 4
                susurawalan = 2  # susurkiri
                keluarlah = 1
                geserkananruang1()
                majuhome()
            elif (arahawal == 4):
                arahpul = 2
                keluarlah = 1  # done
                if (keluarhomehusus == 0):
                    susurawalan = 666
                    keluarhomehusus = 1
                elif (keluarhomehusus == 1):
                    susurawalan = 777
        print
        "susurawalan setelah", susurawalan


def proxi():
    global arah
    global misiapi
    global mulai
    global slesemisi
    global asal
    global garis
    global garisapi
    global garisruang
    global counttt
    global garpul
    global satebeka
    global statebeki
    global proxidipakai
    global proximana1
    global proximana2
    global proximana3
    global proximana4
    global proximana5
    global state1
    global state2
    global statekade
    global garisbalik2hitung
    global isengproxi1
    global isengproxi2
    global isengproxi3
    global isengproxi4
    global isengproxi5
    global garisbalik55hitung
    global done
    global batasproxidepan
    global batasproxikade
    global batasproxikide
    global batasproxibeki
    global batasproxibeka
    global p1
    global p2
    global p3
    p1 = ADC.read("P9_40")
    p2 = ADC.read("P9_35")
    p3 = ADC.read("P9_37")
    if (proximana1 == 1):
        if (p1 <= batasproxidepan):  # 0.95
            proxidipakai = 1
        else:
            proxidipakai = 0
    elif (proximana2 == 1):
        if (p2 <= batasproxikide):  # 0.95
            proxidipakai = 1
        else:
            proxidipakai = 0
    elif (proximana3 == 1):
        if (p3 <= batasproxikade):  # 0.95
            proxidipakai = 1
        else:
            proxidipakai = 0
    elif (proximana4 == 1):
        if (p3 <= batasproxidepan2):
            proxidipakai = 1
        else:
            proxidipakai = 0
    #	print"proxidipakai",proxidipakai,"proximana1:",p1,"proximana2:",p2,"proximana3:",p3
    if (misiapi == 0):
        if (proxidipakai == 1):
            # arah_cmps()
            # if(arah==3):
            #	 pintukhususruang1()
            # input()
            # print "garisNOL",garis
            # print state1,state2
            # else:
            print
            "garisNOL", garis
            garis += 1
            print
            garis
            garisapi = garis + 1
            misiproxi()
            garisruang = garis
    # if ((proxidipakai==1) and (state2==1)):
    #			print "home"
    #	proxidipakai=0
    # print state1,state2

    if (misiapi == 1):
        print
        proxidipakai
        print
        "masuk misiapi=1"
        #      done=1
        if (proxidipakai == 1):
            if (rotation == 0):
                print
                "masuk cek garpul"
                arah_cmps()
                diamCode()
                time.sleep(0.3)
                pintukhususruang1()
            elif (rotation == 1):
                misiproxi()
            elif ((proxidipakai == 1) and (state2 == 1)):
                #				print "home balik"
                done = 1


def proxi2():
    global misiapi
    global mulai
    global slesemisi
    global asal
    global garis
    global garisapi
    global garisruang
    global counttt
    global garpul
    global satebeka
    global statebeki
    global proxidipakai
    global proximana1
    global proximana2
    global proximana3
    global proximana4
    global proximana5
    global state1
    global state2
    global statekade
    global garisbalik2hitung
    global isengproxi1
    global isengproxi2
    global isengproxi3
    global isengproxi4
    global isengproxi5
    global garisbalik55hitung
    global done
    global batasproxidepan
    global batasproxikade
    global batasproxikide
    global batasproxibeki
    global batasproxibeka
    global arah
    global arahpul
    #	arahpul=4
    arah_cmps()
    if (proximana1 == 1):
        if (ADC.read("P9_40") <= batasproxidepan):  # 0.95
            proxidipakai = 1
        else:
            proxidipakai = 0
    elif (proximana2 == 1):
        if (ADC.read("P9_35") <= batasproxikide):  # 0.95
            proxidipakai = 1
        else:
            proxidipakai = 0
    elif (proximana3 == 1):
        if (ADC.read("P9_37") <= batasproxikade):  # 0.95
            proxidipakai = 1
        else:
            proxidipakai = 0
    # print"proxidipakai",proxidipakai,"proximana1:",ADC.read("P9_40"),"proximana2:",ADC.read("P9_35"),"proximana3:",ADC.read("P9_37")
    if (proxidipakai == 1):
        diamCode()
        time.sleep(0.8)
        # ultradeka()
        # while(b4>70):
        #	majuCode()
        #	ultradeka()
        if (rotation == 1):
            if (arah == arahpul):
                majuCode()
                time.sleep(1.1)
                done = 1
        elif (rotation == 0):
            if ((arah == 2) and ((arahpul == 3) or (arahpul == 1))):
                ultradeka()
                print
                "deka=", b4, "arah=", arah
                # while(b4>60):
                majuCode()
                time.sleep(1.2)
                #	ultradeka()
                #	print "deka",b4
                # rah==2):
                # arah_cmps
                putarkanankecilCode()
                time.sleep(1.7)
                ultrabeka()
                ultradeka()
                # while((b8<60)and(b4>95)):
                majuCode()
                time.sleep(1.3)
            #	ultrabeka()
            #	ultradeka()
            #	print"beka",b6,"deka",b4
            elif ((arah == 4) and ((arahpul == 3) or (arahpul == 1))):
                ultrabeka()
                print
                "beka", b8
                while (b8 < 75):
                    majuCode()
                    time.sleep(0.1)
                    ultrabeka()
                arah_cmps()
                while (arah != 3):
                    arah_cmps()
                    putarkirisedangCode()
                    time.sleep(0.3)
                ultrakisa()
                while (b1 > 30):
                    majuCode()
                    ultrakisa()
            elif (arah == 1):
                ultradeka()
                print
                "deka", b4
                while (b4 < 16):
                    majuCode()
                    time.sleep(0.1)
                    ultradeka()
                arah_cmps()
                if (balikultradepandekat2 == 1):
                    while (arah != 4):
                        arah_cmps()
                        putarkirisedangCode()
                        time.sleep(0.2)
                elif (balikultradepandekat2 == 0):
                    while (arah != 2):
                        arah_cmps()
                        putarkanansedangCode()
                        time.sleep(0.2)
                ultrakisa()
                while (b1 > 30):
                    majuCode()
                    ultrakisa()


has3home = 10
has4home = 10


def majuhome():
    global has3home
    global has4home
    print
    "masuk majuhome4"
    while ((has3home > 12) and (has4home > 12)):  # batasdepan12cmsebelumbfareza
        has3home = 5
        has4home = 5
        majuspecialCode()
        print
        "has3home=", has3home
        print
        "has4home=", has4home
        # time.sleep(0.00005)
        for i in range(0, 5):
            ultradeka()
            ultradeki()
            if has3home > b3:
                has3home = has3home
            elif has3home <= b3:
                has3home = b3
            if has4home > b4:
                has4home = has4home
            elif has4home <= b4:
                has4home = b4
            # putarkanansedangCode()
    # time.sleep(0.5)
    diamCode()
    time.sleep(0.2)


kontingkekiri = 0


def gesercekapi():
    global konkis
    global has1a1
    global has1a2
    global konkas
    global susurkanan
    global susurkiri
    if (susurkanan == 1):
        arah_cmps()
        if ((arah == 2) or (arah == 1) or (arah == 3)):
            ultrakisa()
            while (b1 > 14):  # has1a1>7
                kepitingkiriCode()
                ultrakisa()
        elif (arah == 4):
            ultrakasa()
            while (b6 > 13):
                kepitingkananCode()
                ultrakasa()
        diamCode()
        time.sleep(0.1)
    elif (susurkiri == 1):
        arah_cmps()
        if ((arah == 2) or (arah == 1) or (arah == 3)):
            ultrakisa()
            while (b1 > 13):  # has1a1>7
                kepitingkiriCode()
                ultrakisa()
        elif (arah == 4):
            ultrakasa()
            while (b6 > 13):
                kepitingkananCode()
                ultrakasa()
        diamCode()
        time.sleep(0.1)


def manipulasimasuk3():
    print
    "MANIPULASIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII"
    arah_cmps()
    if (arah == 4):
        ultrakasa()
        while (b6 > 13):
            kepitingkananCode()
            ultrakasa()
        ultrakade()
        while (b5 > 17):
            putarkanankecilCode()
            ultrakade()
        ultradeka()
        ultradeki()
        ultrakisa()
        while (((b4 < 110) or (b3 < 110)) and (b1 < 80)):
            mundurCode()
            ultradeka()
            ultradeki()
            ultrakisa()
            print
            b4, "-", b3, "-", b1
        diamCode()
        time.sleep(0.1)
        if (susurkanan == 1):
            ultrakasa()
            while (b6 < 17):
                kepitingkiriCode()
                ultrakasa()
            putarkirikecilCode()
            time.sleep(2)
        elif (susurkiri == 1):
            ultrakasa()
            while (b6 < 15):
                kepitingkiriCode()
                ultrakasa()
                print
                "kepiting", b6
            putarkanankecilCode()
            time.sleep(2)  #
            print
            "sudah putar kanaaaaaan"
    else:
        ultrakisa()
        while (b1 > 18):
            kepitingkiriCode()
            ultrakisa()
        ultrakide()
        while (b2 > 23):
            putarkirikecilCode()
            ultrakide()
        ultradeka()
        ultradeki()
        ultrabeka()
        ultrakasa()
        while (((b4 < 60) or (b3 < 60)) and (b8 > 13)):
            mundurCode()
            ultradeka()
            ultradeki()
            ultrabeka()
            ultrakasa
            print
            "mundur", b4, "-", b3, "-", b6, "-", b8
        diamCode()
        time.sleep(0.1)
        if (susurkanan == 1):
            ultrakisa()
            while (b1 < 18):
                kepitingkananCode()
                ultrakisa()
            putarkirikecilCode()
            time.sleep(2.5)
        elif (susurkiri == 1):
            ultrakisa()
            while (b1 < 18):
                kepitingkananCode()
                ultrakisa()
            putarkanankecilCode()
            time.sleep(2)  #
            print
            "sudah putar kanaaaaaan"


def mainpro():
    global mulai
    global countpu
    global done
    global start
    global state3
    global state4
    global majucon
    global b3
    global siaga1
    global siaga2
    global state55
    global sound
    global batasa
    if (done == 1):
        diamCode()
        print
        "misi selesai"
        sys.exit()
    elif (mulai == 0):
        # tombol_start()                        #manipulasi tombol start dan stop pada 1 kill plug
        sound_act()  # 500
        #		start=1
        #		print "masuk pengecekan tombol atau sound"
        if (sound == 0):  # kondisi stand by robot
            print
            "robot standby"
            diamCode()
        elif (sound == 1):  # start menggunakan tombol atau sound
            print
            "pengecekan kondisi start"
            # print"a"
            cekstartmana()
            # siaga1=1
            # berputar sampai kondisi ultra depan dan kanan dekat, lalu berputar balik
            if ((siaga1 == 1) and (siaga2 == 0)):
                cek_start()
                # proxiawal()
                # while((state3==1)or(state4==1)or(statebelka1==1)or(statebelki1==1)):#50
                #	Susur_Kananspesial()
                #	print"Susur_Kananspesial"
                # countpu=countpu+1
                # print countpu
                # countkeluarhome+=1
                # print "susur spesial countkeluarhome:"
                # ultra_deki(0.03)
                # while((majucon<=3)and(b3>9)):
                while ((majucon <= 3) and ((b3 > 9) and (b4 > 9))):
                    majuCode()
                    ultradeki()
                    ultradeka()
                    majucon = majucon + 1
                    time.sleep(0.01)
                    print
                    "majucon", majucon
            elif ((siaga1 == 0) and (siaga2 == 1)):
                cekstart2()
                while (garis < 1):
                    print
                    "Susur_Kanan"  # ()#KALO ADA BULAT PUTIH HOME
                    Susur_Kananspesial()  # KALO GA ADA HOME BULAT PUTIH
            # time.sleep(0.5)
            mulai = 1  # agar tidak mengecek kembali tombol start atau sound

    elif (mulai == 1):
        print
        "arahjalan"
        arahjalan()


def arahjalan():
    global misiapi
    global countgaris
    global asal
    global jalanmisi
    global asal
    global garisapi
    global susurkiri
    global susurkanan
    global garisbalik1
    global garisbalik2
    global garpul
    global susurawalan
    global keluarhomehusus
    global arah
    global arahpul
    global rotation
    global susurkanan
    global susurkiri
    #	print " masuk arah jalan,susurawalan",susurawalan
    global arahnow
    # arah_cmps()
    if (misiapi == 0):
        if (api == 0):
            if (susurawalan == 1):
                Susur_Kanan()
            elif (susurawalan == 4):
                Susur_Kiri()
            elif (susurawalan == 3):
                Susur_Kiri()
            elif (susurawalan == 5):
                Susur_Kanan()
            elif (susurawalan == 2):
                Susur_Kiri()
            elif (susurawalan == 666):
                Susur_Kanan()
            elif (susurawalan == 777):
                Susur_Kiri()
            elif (susurawalan == 14045):
                Susur_Kananspesial()
        elif (api == 1):
            print
            "susurkirispesialmisiapi0adaapi "
            Susur_Kirispesial()
    elif (misiapi == 1):
        if (garpul == 0):
            print
            "garpul=0 misiapi=1"
            Susur_Kananspesial()
        elif (garpul == 1):
            print
            "garpul=1 misiapi=1 dan sudah keluar ruangan api rotation=", rotation
            if (rotation == 0):
                print
                arahnow
                arah = arahnow
                if ((arah == 2) and (arahpul == 2)):
                    Susur_Kirikembali()
                elif ((arah == 4) and (arahpul == 4)):
                    Susur_Kanankembali()
                elif (arah == 2):
                    Susur_Kirikembali()
                elif (arah == 1):
                    print
                    "masuk susur kanan kembali"
                    Susur_Kanankembali()
                elif (arah == 4):
                    print
                    "masuk susur kiri kembali"
                    Susur_Kirikembali()
            elif (rotation == 1):
                print
                susurkiri
                print
                "masuk susur kembali sesungghunya"
                if (susurkanan == 1):
                    Susur_Kanankembali()
                elif (susurkiri == 1):
                    Susur_Kirikembali()
            '''if(garisapi<=3):#tadinya 4 sebelum garpul
				print "garisapi<=3  ",garisapi
				#majuCode()
				#time.sleep(0.3)
				Susur_Kiri()
				susurkiri=1
				susurkanan=0
				garisbalik1=1
				garisbalik2=0
				print "susur kiri mainpro balik"
			else:
				#garisapi=4
				Susur_Kanan()
				susurkanan=1
				susurkiri=0
				garisbalik1=0
				garisbalik2=1	
			'''  # print "susur kanan mainpro balik"


infrared123 = 0
depanruangan1 = 0
arahnow = 0


def pintukhususruang1():
    global arah
    global ir_khususruang1
    global depanruangan1
    global garpul
    global garisapi
    global arahnow
    global countmaju
    print
    "MASUK PINTUUUUUUU KHUSSUS RUANG1111111111111"
    infraredruang1()
    print
    "arah=", arah
    print
    "infrared=", ir_khususruang1
    if (ir_khususruang1 == 0):
        depanruangan1 = 0
    elif (ir_khususruang1 == 1):
        depanruangan1 = 1
    if ((depanruangan1 == 1) and (arah == 3)):
        print
        "pintu salaaaaaahhhhhhhhhhhh"
        garisapi = garisapi + 1
        mundurCode()
        time.sleep(2.2)
        putarkirikecilCode()
        time.sleep(1.8)
        majuCode()
        time.sleep(1.8)
    else:
        if (misiapi == 1):
            print
            "garpul=1"
            garpul = 1
            countmaju = 0
            infrared_atas = (GPIO.input("P8_11"))
            if ((arah == 2) and (infrared_atas == 1)):
                ultrakisa()
                while (b1 > 15):
                    kepitingkiriCode()
                    ultrakisa()
                ultrakide()
                while (b2 > 26):
                    putarkirisedangCode()
                    ultrakide()
            elif ((arah == 2) and (infrared_atas == 0)):
                ultrakasa()
                while (b6 > 15):
                    kepitingkananCode()
                    ultrakasa()
            elif (arah == 1):
                while ((b4 > 30) and (countmaju < 18)):
                    majuCode()
                    time.sleep(0.1)
                    ultradeka()
                    countmaju = countmaju + 1
                    print
                    countmaju, "majudikit mau pulang", b4
            elif ((arah == 3) or (arah == 4)):
                ultrakasa()
                while (b6 > 12):
                    kepitingkananCode()
                    ultrakasa()
                ultrakade()
                while (b5 > 19):
                    putarkanansedangCode()
                    ultrakade()
            # mundurCode()
            # time.sleep(1.4)
            arah_cmps()
            arahnow = arah


# ====================MISIPROXI========================================================
def misiproxi():
    global countmaju
    global garis
    global ruang
    global lorong
    global startruang
    global apiruang
    global misiapi
    global susur
    global countingapi
    global api
    global kondisicek
    global cekcek
    global start
    global stop
    global mulai
    global asal
    global countgaris
    global nyasar
    global jalanmisi
    global countgarismisi
    global slesemisi
    global depanhome
    global skipperataan
    global loopmuterkanan
    global loopmuterkiri
    global susurkanan
    global susurkiri
    global putarkanansekali
    global garisapi
    global done
    global tambahgar
    global garisbalik1
    global garisbalik2
    global counm
    global counn
    global coun12
    global counci
    global keluarnyasar
    global arahpulang
    global pulangkhusus
    global balikultradepandekat2
    global pembandingdepan
    global ir_pulang
    global garisnyasar1
    global garisnyasar2
    global garisnyasar3
    global arahpul
    global arah
    global hasdeka
    global hasdeki
    global proxidipakai
    global proximana1
    global proximana2
    global proximana3
    global garpul
    global belumkeluarhome
    global pembandingdepanruang1
    global keluarhomehusus
    global keluarlah
    global detect_garis
    global susurawalan
    global b2
    global b1
    global b5
    global b6
    print
    "masuk cek misiproxi"
    #	print "pulangkhusus =",pulangkhusus,"balikultradepandekat2=",balikultradepandekat2,"pembandingdepan=",pembandingdepan
    #	print"proxidipakai",proxidipakai,"proximana1",proximana1,"proximana2",proximana2
    # print "countgaris:",countgaris
    diamCode()
    arah_cmps()
    cek_nyasar()
    nyasar1()
    time.sleep(0.4)
    #	print"garisnyasar1:",garisnyasar1,"garisnyasar2:",garisnyasar2,"garisnyasar3",garisnyasar3
    #	print "garisbalik1:",garisbalik1,"garisbalik2",garisbalik2
    if (apiruang == 1):
        print
        "di ruangan ada api"
        if (misiapi == 1):
            #				print "api berhasil dipadamkan"
            if (api == 0):
                #					print "keluar dari ruang yang ada api"
                #					print"garisnyasar1:",garisnyasar1,"garisnyasar2:",garisnyasar2,"garisnyasar3",garisnyasar3
                if (garisapi == (garis + 1)):
                    majuCode()
                    time.sleep(0.2)
                    apiruang = 0
        elif (misiapi == 0):
            #				print "mundur karna api belom padam"
            # majuCode()
            # time.sleep(1)
            mundurCode()  # 0.4                                                       ini ya 1..
            time.sleep(0.1)
            # coun12=0
            # putarkanansedangCode()
            # time.sleep(0.5)
            ultradeka()
            ultradeki()
            while ((coun12 <= 5) and ((b3 > 11) or (b4 > 11))):  # coun12<=35
                putarkanansedangCode()
                ultradeka()
                ultradeki()
                # time.sleep(1.50)#1.18
                coun12 += 1
            #					print "coun12",coun12
            ultradeka()
            ultradeki()
            while ((counci <= 100) and ((b3 > 9) and (b4 > 9))):  # counci<=5000
                majuCode()
                ultradeka()
                ultradeki()
                counci += 1
            #					print"counmaju",counci
            # majuCode()
            # time.sleep(0.5)
            garisapi -= 1
            garis -= 1
        # cari_api_KW_lagi()

    elif (apiruang == 0):
        #		print "CEK INI PENTING","garis api:",garisapi,"garis ruang:",garisruang,"arahawal",arah,"garis:",garis,"arahpul:",arahpul
        #		print "ada kemungkinan dalam ruang ada api"
        if (misiapi == 1):  # jalan pulang
            # if((garisapi==garisruang)and(garpul==0)):
            #	print"keluar ruang api","arah:",arah
            #	majuCode()
            #	time.sleep(0.3)
            if (garisbalik1 == 1):
                if (pulangkhusus == 0):
                    #					print "pulangkhusus sama dengan nol"
                    ir_khususruang1keluararah3()
                    if ((arah == 1) and (arahpul == 4) and (pembandingdepanruang1 == 0)):
                        majuCode()
                        time.sleep(1)  # 0.8
                        print
                        "balikkhususruang1"
                        done = 1
                    elif ((garisapi == 1) and (arah == arahpul)):
                        print
                        "tempatbalik1"
                        majuCode()
                        time.sleep(1)  # 0.8
                        done = 1
                    elif ((garisapi == 1) and (arah != arahpul)):
                        garisapi += 1
                        print
                        "mundur garis balik1"
                        mundurCode()
                        time.sleep(0.65)
                        if (susurkiri == 1):
                            putarkanansedangCode()
                            time.sleep(0.9)
                            majuCode()
                            time.sleep(0.9)
                        elif (susurkanan == 1):
                            putarkirisedangCode()
                            time.sleep(1.18)
                            majuCode()
                            time.sleep(0.9)
                    elif ((garisapi >= 1) and (arah != arahpul)):
                        print
                        "mundur garis balik1 11111111111"
                        mundurCode()
                        time.sleep(0.65)
                        putarkanansedangCode()
                        time.sleep(0.9)
                        majuCode()
                        time.sleep(0.9)
                elif (pulangkhusus == 1):
                    #					diamCode()
                    arah_cmps()
                    print
                    " PENGECEKAN RUANGAN PULANGKHUSUS1"
                    print
                    garisapi
                    if ((garisapi == 1) and (arah == arahpul)):
                        ultraruangbalik2()
                        print
                        "masuk kondisi yang dicari"
                        if ((balikultradepandekat2 == 1) and (pembandingdepan == 1)):
                            #                                                	print"tempatbalikmantap"
                            majuCode()
                            time.sleep(1)  # 0.8
                            done = 1
                        elif ((balikultradepandekat2 == 0) and (pembandingdepan == 0)):
                            #                                                        print"tempatbalikmantap2"
                            majuCode()
                            time.sleep(1)  # 0.8
                            done = 1
                        else:
                            garisapi += 1
                            print
                            "GARIS KERASSSSSSSSSSSS"
                            # mundurCode()
                            # time.sleep(0.65)
                            if (susurkiri == 1):
                                #	mundurCode()
                                #	time.sleep(0.5)
                                putarkanansedangCode()
                                time.sleep(0.9)
                                print
                                "putar bosssq"
                                majuCode()
                                time.sleep(0.9)
                            elif (susurkanan == 1):
                                putarkirisedangCode()
                                time.sleep(1.18)

                                majuCode()
                                time.sleep(0.9)
                    elif ((garisapi == 1) and (arah != arahpul)):
                        #						print "ga masuk kondisi yang dcari"
                        garisapi += 1
                        #                                              print "mundur garis balik1"
                        mundurCode()
                        time.sleep(0.65)
                        if (susurkiri == 1):
                            putarkanansedangCode()
                            time.sleep(0.9)
                            majuCode()
                            time.sleep(0.9)
                        elif (susurkanan == 1):
                            putarkirisedangCode()
                            time.sleep(1.18)
                            majuCode()
                            time.sleep(0.9)
                    elif ((garisapi >= 1) and (arah != arahpul)):
                        print
                        "mundur garis balik1 1"
                        mundurCode()
                        time.sleep(0.65)
                        putarkanansedangCode()
                        time.sleep(1.2)
                        majuCode()
                        time.sleep(0.9)
                    elif ((garisapi >= 1) and (arah == arahpul)):
                        print
                        "mundur garis balik1 kncul"
                        majuCode()
                        time.sleep(0.65)
                        putarkanansedangCode()
                        time.sleep(0.9)
                        majuCode()
                        time.sleep(1.9)
                        belokkananCode()
                        time.sleep(0.5)
            #				print "pulangkhusus", pulangkhusus
            #				print "balikultradepandekat2:",balikultradepandekat2
            #				print "pembandingdepan",pembandingdepan

            elif (garisbalik2 == 1):
                print
                "pulankhusus", pulangkhusus
                print
                "garis api", garisapi
                if (pulangkhusus == 0):
                    #					print"arah:",arah,"arah pulang",arahpul
                    if ((garisapi == 5) and (arah == arahpul)):
                        majuCode()
                        time.sleep(0.8)
                        done = 1
                    #						print"balik2 kondisi1"
                    elif ((garisapi == 5) and (arah != arahpul)):
                        garisapi -= 1
                        #						print "mundur garis balik2"
                        mundurCode()
                        time.sleep(0.65)
                        if (susurkiri == 1):
                            putarkanansedangCode()
                            time.sleep(0.9)
                            majuCode()
                            time.sleep(0.9)
                        elif (susurkanan == 1):
                            putarkirisedangCode()
                            time.sleep(1.18)
                            majuCode()
                            time.sleep(0.9)
                    elif ((garisapi <= 7) and (arah != arahpul)):
                        mundurCode()
                        time.sleep(0.65)
                        #                                               print "kondisi3 balik2"
                        if (susurkiri == 1):
                            putarkanansedangCode()
                            time.sleep(0.9)
                            majuCode()
                            time.sleep(0.9)
                        elif (susurkanan == 1):
                            putarkirisedangCode()
                            time.sleep(1.18)
                            majuCode()
                            time.sleep(0.9)
                elif (pulangkhusus == 1):
                    arah_cmps()
                    #					print"masuk pengecekan pulangkhusus garisbalik2"
                    if ((arahpul == 1) and (balikultradepandekat2 == 0) and (arah == 4)):
                        majuCode()
                        time.sleep(1)
                        done = 1
                    # print"masuk pengecekan pulangkhusus garisbalik2"
                    elif ((garisapi == 5) and (arah == arahpul)):
                        ultraruangbalik2()
                        #                                             print "masuk kondisi yang dicari2"
                        if ((balikultradepandekat2 == 1) and (pembandingdepan == 1)):
                            #                                                      print"tempatbalikmantap2"
                            majuCode()
                            time.sleep(1)  # 0.8
                            done = 1
                        elif ((balikultradepandekat2 == 0) and (pembandingdepan == 0)):
                            #                                                       print"tempatbalikmantap32"
                            majuCode()
                            time.sleep(1)  # 0.8
                            done = 1
                        else:
                            garisapi -= 1
                            #                                                	print "mundur garis balik2"
                            mundurCode()
                            time.sleep(0.65)
                            if (susurkiri == 1):
                                putarkanansedangCode()
                                time.sleep(0.9)
                                majuCode()
                                time.sleep(0.9)
                            elif (susurkanan == 1):
                                putarkirisedangCode()
                                time.sleep(1.18)
                                majuCode()
                                time.sleep(0.9)
                    elif ((garisapi == 5) and (arah != arahpul)):
                        garisapi -= 1
                        #                                               print "mundur garis balik2"
                        mundurCode()
                        time.sleep(0.65)
                        if (susurkiri == 1):
                            putarkanansedangCode()
                            time.sleep(0.9)
                            majuCode()
                            time.sleep(0.9)
                        elif (susurkanan == 1):
                            putarkirisedangCode()
                            time.sleep(1.18)
                            majuCode()
                            time.sleep(0.9)
                    elif ((garisapi <= 7) and (arah != arahpul)):
                        mundurCode()
                        time.sleep(0.65)
                        #                                                print "kondisi3 balik2"
                        if (susurkiri == 1):
                            putarkanansedangCode()
                            time.sleep(0.9)
                            majuCode()
                            time.sleep(0.9)
                        elif (susurkanan == 1):
                            putarkirisedangCode()
                            time.sleep(1.18)
                            majuCode()
                            time.sleep(0.9)
        elif (misiapi == 0):
            print
            "misi=0"
            if (garis == 1):
                print
                "garis=", garis
                susurawali()
                # majuCode()
                ultrakasa()
                if (keluarlah == 0):
                    ultrakasa()
                    while (b6 < 15):
                        ultrakasa()
                        kepitingkiriCode()
                        time.sleep(0.1)
                    mundurCode()  # 0.4                        $
                    time.sleep(1.2)
                    print
                    "keluarlah=0"
                    putarkanansedangCode()  # putarkirisedangCode()
                    time.sleep(1)
                    garis = 0
                    detect_garis = 0
                    arahjalan()
                elif (keluarlah == 1):
                    print
                    "keluarlah=1"
                    countmaju = 0
                    arah_cmps()
                    if (arah == 2):
                        ultrakisa()
                        while (b1 > 11):
                            kepitingkiriCode()
                            ultrakisa()
                            print
                            "kepiting ", b1
                        ultrakide()
                        while (b2 > 23):
                            putarkirisedangCode()
                            ultrakide()
                            print
                            "putarkiri", b2
                    elif (arah == 4):
                        ultrakasa()
                        while (b6 > 16):
                            kepitingkananCode()
                            ultrakasa()
                        ultrakade()
                        while (b5 > 20):
                            putarkanansedangCode()
                            ultrakade()
                    ultradeka()
                    while ((b4 > 16) and (countmaju < 15)):
                        majuCode()
                        time.sleep(0.1)
                        ultradeka()
                        countmaju = countmaju + 1
                        print
                        countmaju, "misio", b4
                    keluarnyasar = 1
                    belumkeluarhome = 0
            elif (garis >= 2):
                gesercekapi()
                cari_api()
                diamCode()
                time.sleep(0.8)
                if ((api == 0) and (nyasar == 0)):
                    manipulasimasuk3()
                    # cari_api2()
                    # if((api==0)and(nyasar==0)):
                    # mundurCode()
                    # time.sleep(0.45)
                    if (susurkanan == 1):
                        # putarkanan_sedang()
                        # print"Putar Kanaaaaaaaaaaaaaaaaaaan"
                        # time.sleep(1.9)
                        counm = 0
                        hasdeka = 0
                        hasdeki = 0
                        batasdepankosong()
                        if ((hasdeka > 10) or (hasdeki > 10)):
                            while ((counm <= 2) and ((b3 > 10) or (b4 > 10))):
                                majuCode()
                                ultradeka()
                                ultradeki()
                                # time.sleep(0.9)
                                # print "counmaju", counm
                                counm += 1
                    #							print counm
                    #							print"majuuuuuuuuuuuuuuuu"
                    else:
                        counn = 0
                        putarkanansedangCode()
                        time.sleep(0.5)
                        ultradeka()
                        ultradeki()
                        while ((counn <= 2) and ((b3 > 10) or (b4 > 10))):
                            majuCode()
                            ultradeka()
                            ultradeki()
                            counn += 1
                            print
                            counn

                elif ((nyasar == 1) and (api == 0)):
                    # majuCode()
                    # time.sleep(1)
                    # print "masuk misi proxi nyasar yeh a"
                    nyasar = 0
                    garisapi = garisapi - 2
                    garis = garis - 2
                    majuCode()
                    time.sleep(1.5)
                    #                        		print"ah maju teuuuuu"
                    #					print "GARIS API",garisapi,"GARIS:",gari
                    print
                    "maju karna nyasar"
        print
        "keluarhomehusus", keluarhomehusus
        if ((keluarhomehusus == 1) and (garis >= 2)):
            print
            "harusnya susurkiriiiiiiiii"
            majuCode()
            time.sleep(0.1)
            susurkanan = 0
            susurkiri = 1
            susurawalan = 777


#	print"==================================================================== garis proxi ========================================="
def Susur_Kananspesial():
    global mpi
    global last_error
    global lpwm
    global rpwm
    global susurkanan
    global susurkiri
    global b1
    global b2
    global b3
    global b4
    global b5
    global b6
    global b7
    global b8
    global b9
    global proximana1
    global proximana2
    global proximana3
    global proximana4
    global proximana5
    global loopmuterkanan
    global konk
    proximana1 = 0
    proximana2 = 1
    proximana3 = 0
    proximana4 = 0
    # proximana5=0
    susurkanan = 1
    susurkiri = 0
    ultrakade()
    ultradeka()
    ultradeki()
    print
    b3, b4, b5
    #	infrared
    #        thermal1102()
    limitswitch()
    # ledlilin()
    proxi()
    # time.sleep(2)
    if (b4 < 13) and (b3 < 13):  # 1
        #               infrared1()
        limitswitch()
        #               thermal1102()
        proxi()
        ultradeka()
        ultradeki()
        while (((b4 < 15) and (b3 < 15)) or (b5 < 14)):  # 2
            #                   infrared1()
            limitswitch()
            #                     thermal1102()
            proxi()
            putarkirisedangCode()
            time.sleep(0.3)
            print
            "putar kiri sedang"
            ultradeka()  ###### 0.09
            ultradeki()  ###### 0.09
            ultrakade()
            break
    elif (b5 > 20):  # 18 #17#3
        #          infrared1(
        limitswitch()
        #          thermal1102()
        proxi()
        loopmuterkanan = 0
        while ((b4 >= 10) and (b3 >= 10) and (b5 >= 15)):  # 4
            if (loopmuterkanan <= 50):
                proxi()
                belokkananCode()
                print
                "belokkanan"
                ultradeka()
                ultradeki()
                ultrakade()
                limitswitch()
                konk = 0
                loopmuterkanan += 1
                print
                loopmuterkanan
                proxi()
            elif (loopmuterkanan > 50):
                proxi()
                loopmuterkanan = 0
                ultradeka()
                ultradeki()
                while ((b3 > 10) and (b4 > 10)):
                    majuCode()
                    # time.sleep(0.1)
                    print
                    "majudekatin"
                    ultradeka()
                    ultradeki()
                    # konk+=1
                    proxi()
    elif (b6 <= 5):  # 13#15#12
        #       infrared1()
        limitswitch()
        #       thermal1102()
        proxi()
        while (b6 <= 5):  # 15
            print
            "kepiting kiri"
            kepitingkiriCode()
            #              infrared1()
            limitswitch()
            #              thermal1102()
            ultrakasa()
            proxi()
            break
    else:
        #    infrared1()
        limitswitch()
        #    thermal1102()
        proxi()
        error = (b5 - 13)  # b5
        P = (kp1 * error)
        It = ((error + last_error) * ki1)
        rate = (error - last_error)
        D = (rate * kd1)
        MV = (P + D + It)
        lpwm = 6 + MV
        rpwm = 6 - MV
        last_error = error
        if (rpwm > 6):
            rpwm = 6
        if (rpwm < 0):
            lpwm = 0
        normalisasi_velocity()
        # print "normalisasi velocity"
        print
        "error", error
        print
        "It", It
        print
        "MV", MV
        print
        "pid"
        print
        "rpwm", rpwm
        print
        "lpwm", lpwm
        # "susurkanan", susurkanan print "susurkiri", susurkiri
        time.sleep(Var_waktu)


def Susur_Kirispesial():
    global mpi
    global last_error
    global lpwm
    global rpwm
    global susurkanan
    global susurkiri
    global b1
    global b2
    global b3
    global b4
    global b5
    global b6
    global b7
    global b8
    global b9
    global b10
    global proximana1
    global proximana3
    proximana1 = 1
    proximana2 = 0
    proximana3 = 0
    # proximana4=0
    # proximana5=0
    susurkiri = 1
    susurkanan = 0
    ultradeka()
    ultradeki()
    ultrakide()
    print
    "Msuk def susurkiriiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiispecial"
    limitswitch()
    thermal1102()
    proxi()
    print
    b2, b3, b4
    if (b4 < 12) and (b3 < 12):
        print
        "cuedekkkkkkkkkkkkkkkkkkkk"
        limitswitch()
        thermal1102()
        proxi()
        while ((b4 < 12) or (b3 < 12) or (b2 < 15)):
            # infrared1()
            limitswitch()
            thermal1102()
            proxi()
            putarkanankecilCode()
            time.sleep(0.1)
            # lpwm=-9
            # rpwm=-9
            print
            "putar kanan sedang"
            # ultra_deka(0.09)####### 0.09
            # ultra_deki(0.09)####### 0.09
            thermal1102()
            ultrakide()
            ultradeka()
            ultradeki()
            print
            b3, b4, b2
            # time.sleep(0.22)
    #                        break
    elif (b2 > 18):  # 17
        # infrared1()
        limitswitch()
        thermal1102()
        proxi()
        # maju() time.sleep(0.4)
        while ((b3 >= 11) and (b4 >= 11) and (b2 >= 18)):
            # infrared1()
            limitswitch()
            thermal1102()
            # thermalreg()
            proxi()
            print
            "belok kiri"
            belokkiriCode()
            # putarkirisedangCode()
            # time.sleep(0.2)
            ultradeka()
            ultradeki()
            ultrakide()  # 0.05
            # print "kade",ultra_kadewi
            thermal1102()
            # print "belokkanan"
            # time.sleep(0.1)
            #                       ultra()
            #               b4=(var[1])
            #               b3=(var[2])
            #               b5=(var[0])
            print
            b3, b4, b2
    #                       break
    elif (b1 <= 7):  # 15#12
        # infrared1()
        thermal1102()
        limitswitch()
        proxi()
        while (b1 <= 7):  # 15
            kepitingkananCode()
            # infrared1()
            thermal1102()
            limitswitch()
            proxi()
            ultrakisa()
    #              print "kepiting kanan "
    #       print b3,b4,b2
    #                      break
    # putarkirisedangCode()
    # time.sleep(0.025)
    else:
        # infrared1()
        limitswitch()
        thermal1102()
        proxi()
        error = (b2 - 10)
        P = (kp2 * error)
        It = ((error + last_error) * ki2)
        rate = (error - last_error)
        D = (rate * kd2)
        MV = (P + D + It)
        lpwm = 2.5 - MV
        rpwm = 2.5 + MV
        last_error = error
        if (rpwm > 2.5):
            rpwm = 2.5
        if (rpwm < 0):
            rpwm = 0
        if (lpwm > 2.5):
            lpwm = 2.5
        if (lpwm < 0):
            lpwm = 0
        normalisasi_velocity()
        # print "ikilho"
        # print "pid"
        # print "rpwm",rpwm
        # print "e",error
        # print "P",P
        # print "It",It
        # print "MV",MV
        # print "pid"
        # print "rpwm",rpwm
        # print "lpwm",lpwm
        # "susurkanan", susurkanan print "susurkiri", susurkiri
        time.sleep(Var_waktu)


def Susur_Kanan():
    global mpi
    global last_error
    global lpwm
    global rpwm
    global susurkanan
    global susurkiri
    global b1
    global b2
    global b3
    global b4
    global b5
    global b6
    global b7
    global b8
    global b9
    global proximana1
    global proximana2
    global proximana3
    global proximana4
    global proximana5
    proximana1 = 0
    proximana2 = 1
    proximana3 = 0
    proximana4 = 0
    proximana5 = 0
    susurkanan = 1
    susurkiri = 0
    ultrakade()
    ultradeka()
    ultradeki()
    print
    b3, b4, b5
    infrared1()
    thermal1102()
    limitswitch()
    ledlilin()
    proxi()
    #	#time.sleep(2)
    if (b4 <= 16) and (b3 <= 16):  # 14
        infrared1()
        limitswitch()
        thermal1102()
        proxi()
        ultradeka()
        ultradeki()
        while ((b4 < 15) and (b3 < 15) and (b5 < 39)):
            infrared1()
            limitswitch()
            thermal1102()
            proxi()
            putarkirisedangCode()
            time.sleep(0.35)  # 0.15
            print
            "putar kiri sedang"
            ultradeka()  ###### 0.09
            ultradeki()  ###### 0.09
            ultrakade()
            break
    elif (b5 > 20):  # 18 #17#19
        infrared1()
        limitswitch()
        thermal1102()
        proxi()
        # maju() time.sleep(0.4)
        while ((b4 >= 10) and (b3 >= 10) and (b5 >= 15)):
            infrared1()
            limitswitch()
            thermal1102()
            proxi()
            print
            "belok kanan"
            belokkananCode()
            thermal1102()
            # proxi()
            ultradeka()
            ultradeki()
            ultrakade()  # 0.05
            print
            "belokkanan>>>>>>>>>>>>>>>>>>>>>>>"
            break
    elif (b6 <= 5) or (b5 <= 8):  # 13#15#12
        infrared1()
        limitswitch()
        thermal1102()
        proxi()
        while (b6 <= 5) or (b5 <= 5):  # 15
            print
            "kepiting kiri"
            ultrakasa()
            kepitingkiriCode()
            #	infrared1()
            limitswitch()
            thermal1102()
            proxi()
            break
    else:
        infrared1()
        proxi()
        limitswitch()
        thermal1102()
        error = (b5 - 18)  # b5#16
        P = (kp1 * error)
        It = ((error + last_error) * ki1)
        rate = (error - last_error)
        D = (rate * kd1)
        MV = (P + D + It)
        lpwm = 6 + MV
        rpwm = 6 - MV
        last_error = error
        if (rpwm > 6):
            rpwm = 6
        if (rpwm < 0):
            rpwm = 0
        if (lpwm > 6):
            lpwm = 6
        if (lpwm < 0):
            lpwm = 0
        normalisasi_velocity()
        # print "normalisasi velocity"
        print
        "error", error
        #	print "P",P
        #		print "It",It
        #		print "MV",MV
        print
        "rpwm", rpwm
        print
        "lpwm", lpwm
        # "susurkanan", susurkanan print "susurkiri", susurkiri
        time.sleep(Var_waktu)


def Susur_Kiri():
    global mpi
    global last_error
    global lpwm
    global rpwm
    global susurkanan
    global susurkiri
    global b1
    global b2
    global b3
    global b4
    global b5
    global b6
    global b7
    global b8
    global b9
    global b10
    global proximana1
    global proximana2
    global proximana3
    proximana1 = 1
    proximana2 = 0
    proximana3 = 0
    susurkiri = 1
    susurkanan = 0
    ultrakide()
    ultradeka()
    ultradeki()
    ultrakisa()
    print
    b3, b4, b2, b1
    # print "proximana",proximana1,"-",proximana2,"-",proximana3
    infrared1()
    limitswitch()
    thermal1102()
    ledlilin()
    proxi()
    if (b4 < 15) and (b3 < 15):
        limitswitch()
        thermal1102()
        proxi()
        while ((b4 < 15) or (b3 < 15) or (b2 < 19)):
            infrared1()
            limitswitch()
            thermal1102()
            proxi()
            putarkanansedangCode()
            time.sleep(0.2)
            # lpwm=-9
            # rpwm=-9
            print
            "putar kanan sedang"
            # ultra_deka(0.09)####### 0.09
            # ultra_deki(0.09)####### 0.09
            # print "putarkanan"
            ultrakide()
            ultradeka()
            ultradeki()
            print
            b3, b4, b2
        # time.sleep(0.22)
    #			break
    elif (b2 >= 21):  # 17
        infrared1()
        limitswitch()
        thermal1102()
        proxi()
        # maju() time.sleep(0.4)
        while ((b3 >= 13) and (b4 >= 13) and (b2 >= 21)):
            infrared1()
            limitswitch()
            thermal1102()
            # thermalreg()
            proxi()
            print
            "belok kiri"
            belokkiriCode()
            ultradeka()
            ultradeki()
            ultrakide()  # 0.05
            # print "kade",ultra_kadewi
            proxi()
            # print "belokkanan"
            # time.sleep(0.1)
            #			ultra()
            #		b4=(var[1])
            #		b3=(var[2])
            #		b5=(var[0])
            print
            b3, b4, b2
    #			break
    elif (b1 <= 4) or (b2 <= 2):  # 15#12
        infrared1()
        thermal1102()
        limitswitch()
        proxi()
        while ((b1 <= 4) or (b2 <= 2)):  # 15
            #		print "kepiting kiri"
            # kepitingkiriCode()
            kepitingkananCode()
            infrared1()
            thermal1102()
            limitswitch()
            proxi()
            #			ultra_kade(0.03)
            ultrakisa()
            #		b5=(var[2])
            print
            "kepiting kanan "
            #			print b3,b4,b2
            break
    # putarkirisedangCode()
    # time.sleep(0.025)
    else:
        infrared1()
        limitswitch()
        thermal1102()
        proxi()
        error = (b2 - 18)
        P = (kp2 * error)
        It = ((error + last_error) * ki2)
        rate = (error - last_error)
        D = (rate * kd2)
        MV = (P + D + It)
        lpwm = 6 - MV
        rpwm = 6 + MV
        last_error = error
        if (rpwm > 6):
            rpwm = 6
        if (rpwm < 0):
            rpwm = 0
        if (lpwm > 6):
            lpwm = 6
        if (lpwm < 0):
            lpwm = 0
        normalisasi_velocity()
        # print "normalisasi velocity"
        print
        "error", error
        print
        "rpwm", rpwm
        print
        "lpwm", lpwm
        # "susurkanan", susurkanan print "susurkiri", susurkiri
        time.sleep(Var_waktu)


def Susur_Kanankembali():
    global mpi
    global last_error
    global lpwm
    global rpwm
    global susurkanan
    global susurkiri
    global b1
    global b2
    global b3
    global b4
    global b5
    global b6
    global b7
    global b8
    global b9
    global proximana1
    global proximana2
    global proximana3
    global proximana4
    global proximana5
    proximana1 = 1
    proximana2 = 1
    proximana3 = 1
    #	susurkanan=1
    ultrakide()
    ultrakade()
    ultradeka()
    ultradeki()
    print
    b1, b2, b3, b4, b5
    limitswitch()
    proxi2()
    perempatan()
    if ((b4 <= 16) and (b3 <= 16)):  # 14
        limitswitch()
        perempatan()
        proxi2()
        ultrakade()
        ultradeka()
        ultradeki()
        while ((b4 < 15) and (b3 < 15) and (b5 < 18)):
            proxi2()
            perempatan()
            limitswitch()
            putarkirisedangCode()
            time.sleep(0.2)
            print
            "putar kiri sedang"
            ultradeka()
            ultradeki()
            ultrakade()
            perempatan()
    elif (b5 > 26):  # 18 #17#19
        limitswitch()
        perempatan()
        proxi2()
        while (b5 >= 26):
            proxi2()
            perempatan()
            belokkananCode()
            limitswitch()
            perempatan()
            ultradeka()
            ultradeki()
            ultrakade()  # 0.05
            print
            "belokkanan>>>>>>>>>>>>>>>>>>>>>>>"
        # break
    else:
        perempatan()
        limitswitch()
        proxi2()
        error = (b5 - 19)  # b5#16
        P = (kp1 * error)
        It = ((error + last_error) * ki1)
        rate = (error - last_error)
        D = (rate * kd1)
        MV = (P + D + It)
        lpwm = 6 + MV
        rpwm = 6 - MV
        last_error = error
        if (rpwm > 6):
            rpwm = 6
        if (rpwm < 0):
            rpwm = 0
        if (lpwm > 6):
            lpwm = 6
        if (lpwm < 0):
            lpwm = 0
        normalisasi_velocity()
        time.sleep(Var_waktu)


def Susur_Kirikembali():
    global mpi
    global last_error
    global lpwm
    global rpwm
    global susurkanan
    global susurkiri
    global b1
    global b2
    global b3
    global b4
    global b5
    global b6
    global b8
    global proximana1
    global proximana2
    global proximana3
    proximana1 = 1
    proximana2 = 0
    proximana3 = 0
    susurkiri = 1
    susurkanan = 0
    ultrakide()
    ultradeka()
    ultradeki()
    ultrakisa()
    print
    b1, b2, b3, b4, b5
    proxi2()
    limitswitch()
    perempatan()
    if (b4 < 15) and (b3 < 15):
        perempatan()
        limitswitch()
        while ((b4 < 15) and (b3 < 15)):  # or (b2<14)):
            proxi2()
            perempatan()
            limitswitch()
            putarkanansedangCode()
            time.sleep(0.2)
            print
            "putar kanan sedang"
            ultrakide()
            ultradeka()
            ultradeki()
            print
            b3, b4, b2
    elif (b2 >= 21):  # 17
        proxi2()
        perempatan()
        limitswitch()
        while ((b3 >= 13) and (b4 >= 13) and (b2 >= 21)):
            proxi2()
            perempatan()
            limitswitch()
            print
            "belok kiri"
            belokkiriCode()
            ultradeka()
            ultradeki()
            ultrakide()  # 0.05
    elif (b1 <= 4) or (b2 <= 2):  # 15#12
        perempatan()
        limitswitch()
        while ((b1 <= 4) or (b2 <= 2)):  # 15
            kepitingkananCode()
            perempatan()
            limitswitch()
            ultrakisa()
            print
            "kepiting kanan "
    else:
        proxi2()
        perempatan()
        limitswitch()
        error = (b2 - 19)
        P = (kp2 * error)
        It = ((error + last_error) * ki2)
        rate = (error - last_error)
        D = (rate * kd2)
        MV = (P + D + It)
        lpwm = 6 - MV
        rpwm = 6 + MV
        last_error = error
        if (rpwm > 6):
            rpwm = 6
        if (rpwm < 0):
            rpwm = 0
        if (lpwm > 6):
            lpwm = 6
        if (lpwm < 0):
            lpwm = 0
        print
        "PID kiri"
        normalisasi_velocity()
        time.sleep(Var_waktu)


# rotation=0
# arahpul=4
def perempatan():
    global b1
    global b2
    global b3
    global b4
    global b5
    global b6
    global b8
    global rotation
    global arah
    global arahpul
    global susurkanan
    global susurkiri
    ultrakade()
    ultrakide()
    ultradeka()
    ultradeki()
    ultrabeka()
    ultrakasa()
    ultrakisa()
    arah_cmps()
    #	arahpul=3
    print
    b1, "-", b2, "-", b3, "-", b4, "-", b5, "-", b6, "-", b8
    print
    "arahpul=", arahpul
    if (((b1 > 50) and (b6 > 50)) and (b8 > 50) and (rotation == 0)):
        majuCode()
        time.sleep(0.4)
        diamCode()
        time.sleep(0.8)
        print
        "di perempatan"
        if ((arahpul == 2) and (arah == 2)):
            while (arah == 2):
                putarkirisedangCode()
                time.sleep(0.2)
                print
                "putarkiri<<<<"
                arah_cmps()
            majuCode()
            time.sleep(0.9)
            print
            "2-2"
            susurkanan = 1
            susurkiri = 0
            rotation = 1
        elif ((arahpul == 4) and (arah == 2)):
            while ((has3home > 12) and (has4home > 12)):  # batasdepan12cmsebelumbfareza
                has3home = 5
                has4home = 5
                majuspecialCode()
                print
                "has3home=", has3home
                print
                "has4home=", has4home
                # time.sleep(0.00005)
                for i in range(0, 5):
                    ultradeka()
                    ultradeki()
                    if has3home > b3:
                        has3home = has3home
                    elif has3home <= b3:
                        has3home = b3
                    if has4home > b4:
                        has4home = has4home
                    elif has4home <= b4:
                        has4home = b4
            # putarkanansedangCode()
            # time.sleep(0.5)
            diamCode()
            time.sleep(0.2)

            while (arah == 2):
                putarkirisedangCode()
                time.sleep(0.6)
                arah_cmps()
            majuCode()
            time.sleep(0.6)
            print
            "4-2"
            susurkiri = 1
            susurkanan = 0
            rotation = 1
        elif ((arahpul == 4) and (arah == 4)):
            while (arah == 4):
                putarkanansedangCode()
                time.sleep(0.6)
                print
                "putarkanan<<<<<<"
                arah_cmps()
            majuCode()
            time.sleep(0.6)
            print
            "4-4"
            susurkiri = 1
            susurkanan = 0
            rotation = 1

        elif ((arahpul == 2) and (arah == 4)):
            while (arah == 4):
                putarkanansedangCode()
                time.sleep(0.6)
                arah_cmps()
            majuCode()
            time.sleep(0.6)
            print
            "2-4"
            susurkanan = 1
            susurkiri = 0
            rotation = 1
        elif ((arahpul == 3) and (arah == 2)):
            majuCode()
            time.sleep(1)
            print
            "3-2"
            susurkanan = 1
            susurkiri = 0
            rotation = 1
        elif ((arahpul == 3) and (arah == 4)):
            majuCode()
            time.sleep(1)
            print
            "3-4"
            susurkiri = 1
            susurkanan = 0
            rotation = 1
        elif ((arahpul == 3) and (arah == 3)):
            if (balikultradepandekat2 == 0):
                while (arah == 3):
                    putarkirisedangCode()
                    time.sleep(0.6)
                    arah_cmps()
                majuCode()
                time.sleep(1)
                print
                "3-3-0"
                susurkiri = 1
                susurkanan = 0
                rotation = 1
            elif (balikultradepandekat2 == 1):
                while (arah == 3):
                    putarkanansedangCode()
                    time.sleep(0.1)
                    arah_cmps()
                diamCode()
                time.sleep(0.8)
                ultrakasa()
                # while(b6>20):
                majuCode()
                time.sleep(1)
                print
                "3-3-1"
                susurkiri = 1
                susurkanan = 0
                rotation = 1

    else:
        print
        "jalanterus"


def cek_nyasar():
    global arah
    global detect_garis
    global arahawal
    global arahahir
    global nyasar
    global arahpul
    global pulangkhusus
    if (misiapi == 0):
        print
        "masuk cek nyasar"
        arah_cmps()
        detect_garis += 1
        print
        "detect garisnya ceknyasar:", detect_garis
        if (detect_garis == 1):
            arahawal = arah
            if (arahawal == 1):
                arahpul = 3
                pulangkhusus = 1
            elif (arahawal == 3):
                arahpul = 1
                pulangkhusus = 1
            elif (arahawal == 2):
                arahpul = 4
            elif (arahawal == 4):
                arahpul = 2
        #			print "arahawal: ",arahawal
        #			print "arah pulang",arahpul
        elif (detect_garis >= 2):
            arahahir = arah


#			print "arahahir: ",arahahir
#			print "hasil perbandingan = arahawal:",arahawal, " arahahir:", arahahir,"arahpul",arahpul
#		print"detek garis",detect_garis

# w=====================================================================================
class MyThread(Thread):
    def __init__(self):
        Thread.__init__(self)  # You must do this before anything else
        self.daemon = True  # Causes thread termination of program exit

    # threadzz
    def run(self):
        try:
            while True:
                # mainpro()
                # putar_kanankecilxCode()
                Susur_Kanankembali()
            # thermal1102()
            # thermal1102()
        except (KeyboardInterrupt, SystemExit):
            pass


class MyThread1(Thread):
    def __init__(self):
        Thread.__init__(self)  # You must do this before anything else
        self.daemon = True  # Causes thread termination of program exit

    def run(self):
        try:
            while True:
                gerak()
        #		time.sleep(0.001)
        # Susur_Kanan()
        # maju()
        # main_coba()
        except (KeyboardInterrupt, SystemExit):
            pass


class MyThread2(Thread):
    def __init__(self):
        Thread.__init__(self)  # You must do this before anything else
        self.daemon = True  # Causes thread termination of program exit

    def run(self):
        try:
            while True:
                ultra()
        #		time.sleep(0.001)
        #              Susur_Kanan()
        # maju()
        # main_coba()
        except (KeyboardInterrupt, SystemExit):
            pass


def multithread():
    if __name__ == '__main__':
        thread = MyThread()
        thread1 = MyThread1()
        # thread2= MyThread2()
        thread.start()
        thread1.start()
        # thread2.start()
        try:
            while thread.is_alive():
                thread.join()
                thread1.join()
            # thread2.join()
        except (KeyboardInterrupt, SystemExit):
            print
            'Shutting down ...'


# //----------------ONLY Thread do not use---------
# //------------------ Do not use untill here----------------------//
while True:
    multithread()

#	flamesen3()	
#	aadc()
#	thermalreg3()
#	infrared1()	
##	main_coba0()
#	limitswitch()
#	ultrakisa()
#	ultrakide()
#	ultradeki()
#	ultradeka()
#	ultrakade()
#	ultrakasa()
#	ultrabeka()
#	print"-",b1,"-",b2,"-",b3,"-",b4,"-",b5,"-",b6,"-",b8
