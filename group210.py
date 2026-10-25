import numpy as np
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm
from modbus_scale_client import modbus_scale_client

import tools

RDK = Robolink()
RDK.setRunMode(RUNMODE_SIMULATE)
# RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

## IPv4 address of the Mazzer scale of robot 2.
IP_MAZZER_2 = "192.168.21.3"

## IPv4 address of the Rancilio scale of robot 2.
IP_RANCILIO_2 = "192.168.21.4"

client = modbus_scale_client.ModbusScaleClient(host = IP_RANCILIO_2)
tls = tools.Tools(RDK)

def resetSim():
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

def matrixInvert(matrix):
    Matrix_I = np.eye(4)

    R = matrix[:3, :3]
    t = matrix[:3, 3]

    R_T = R.T
    t_T = -R_T @ t

    Matrix_I[:3,:3] = R_T
    Matrix_I[:3, 3] = t_T

    return  Matrix_I

def getRx(angle):
    Rx = np.array([
        [1,              0,                     0],
        [0,         np.cos(angle), -np.sin(angle)],
        [0,         np.sin(angle),  np.cos(angle)]
    ])
    return Rx

def getRy(angle):
    Ry = np.array([
        [np.cos(angle), 0, -np.sin(angle)],
        [0,             1,              0],
        [np.sin(angle), 0,  np.cos(angle)]
    ])
    return Ry

def getRz(angle):
    Rz = np.array([
        [np.cos(angle),    -np.sin(angle),       0],
        [np.sin(angle),     np.cos(angle),       0],
        [0,                      0,              1]
    ])
    return Rz

def get1():
    R = np.eye(3)
    return R

def getT(R, vector):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = vector

    return T

def calc_rotation(centre, start, degrees):

    v = start - centre         

    return (centre + getRz(np.radians(degrees)) @ v)

def vis_unlock_scale():
    visual_program = RDK.Item("Show_Mazzer_Scale_Read", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()

def vis_lock_scale():
    visual_program = RDK.Item("Show_Mazzer_Scale_Lock", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()

def get_WDT_frame():

    WDT_world = np.array([589.8, -105.3, 86.5])
    WDT_local = np.array([0, 0, 0])
    P_world   = np.array([481.2, -103.9, 37.3])
    P_local   = np.array([108, 0, -42.3])

    WDT_d = np.array([P_local[0] - WDT_local[0], P_local[1] - WDT_local[1]])
    UR_d  = np.array([P_world[0] - WDT_world[0], P_world[1] - WDT_world[1]])

    WDT_cross = WDT_d[0]*UR_d[1] - WDT_d[1]*UR_d[0]
    WDT_dot   = np.dot(WDT_d, UR_d)
    theta     = np.arctan2(WDT_cross, WDT_dot)

    R = getRz(theta)
    #R = getRz(theta) @ getRy(np.radians(-2.9))    # fixs pitch
    UR_T_WDT_np = getT(R, WDT_world)
    return(UR_T_WDT_np)

def getCup():# (m - Jaxon)
    J_intermediate = [-95.327610, -67.618179, -80.367105, -121.098487, 90.578057, -240.407542]

    UR_T_CD_np = np.array([[ 0,     0,    -1,    -591.9 ],
                          [ 0,     1,     0,    -215.4 ],
                          [ 1,     0,     0,     214.1 ],
                          [  0.000000,     0.000000,     0.000000,     1.000000 ]])
    
    CD_T_CDC_np = np.array([[ 1,     0,     0,    -10.9 ],
                            [ 0,     1,     0,     69.5],
                            [ 0,     0,     1,    -40.3],
                            [ 0,     0,     0,     1]])

    CD_T_CDO_np = np.array([[ 1,     0,     0,    -11.3 ],
                            [ 0,     1,     0,     69.2],
                            [ 0,     0,     1,    -74.8],
                            [ 0,     0,     0,     1]])

    MT_T_CDCi_np = np.array([[ 1,     0,     0,      -20],
                            [ 0,     1,     0,  0],
                            [ 0,     0,     1,  102.82],
                            [ 0,     0,     0,     1]])
    MT_T_CDC_np = np.array([[ 1,     0,     0,      0],
                                [ 0,     1,     0,  0],
                                [ 0,     0,     1,  102.82],
                                [ 0,     0,     0,     1]])

    UR_T_CDCi_np = UR_T_CD_np @ CD_T_CDC_np @ matrixInvert(MT_T_CDCi_np)
    UR_T_CDC_np = UR_T_CD_np @ CD_T_CDC_np @ matrixInvert(MT_T_CDC_np)
    UR_T_CDO_np = UR_T_CD_np @ CD_T_CDO_np @ matrixInvert(MT_T_CDC_np)
    
    UR_T_CD = rm.Mat(UR_T_CD_np.tolist())
    UR_T_CDCi = rm.Mat(UR_T_CDCi_np.tolist()) #Cup dispenser hover closed
    UR_T_CDC = rm.Mat(UR_T_CDC_np.tolist()) #Cup dispenser closed
    UR_T_CDO = rm.Mat(UR_T_CDO_np.tolist()) #Cup dispenser open

    tls.mazzer_tool_attach_r_ati()
    UR5.MoveJ(J_intermediate, blocking=True)
    UR5.MoveJ(UR_T_CDCi, blocking=True)
    sleep(1)
    UR5.MoveL(UR_T_CDC, blocking=True)
    UR5.MoveJ(UR_T_CDO, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_CDC, blocking=True)
    UR5.MoveL(UR_T_CDCi, blocking=True)
    UR5.MoveJ(J_intermediate, blocking=True)
    tls.mazzer_tool_detach_r_ati()

def placeCupRancilio(): # (n - Jaxon)
    intermediate_0 = [-99.500658, -71.447990, -145.525786, -150.086183, -8.803035, -212.850094]
    intermediate_1 = [-100.897369, -71.770865, -145.306458, -148.991403, -10.188578, -213.827318]
    intermediate_2 = [-159.336675, -63.682714, -141.844440, -153.885224, -113.745689, -218.627199]

    CTO_point = [-104.5,	0,	186.62]
    CTS_tuned_point = [-120,	    0,	186.62]

    CD_point = [-591.9,	-215.4,	214.1]
    C_point = [-170,	68.3,	30.7]
    Ci_point = [-170,	68.3,	-15]
    Ci2_point = [-155,	68.3,	30.7]

    R_point = [-388.3,	-327.9,	42.6]
    RSP_point = [157.52,	-19,	32] # RS pan
    RSP2_point = [50,	-19,	32] # rs pan backup

    Rancilio_point1 = np.array([0,   0,      0])
    Rancilio_point2 = np.array([0,	-38,	0])
    World_point1 = np.array([-388.3,	-327.9,	42.6])
    World_point2 = np.array([-421.2,	-308.6,	42.6])
    diff_UR = World_point2 - World_point1
    diff_MS = Rancilio_point2 - Rancilio_point1
    cross = diff_MS[0]*diff_UR[1] - diff_MS[1]*diff_UR[0]
    dot   = diff_MS[0]*diff_UR[0] + diff_MS[1]*diff_UR[1]
    angle_R = np.arctan2(cross, dot)

    UR_T_CD_np = getT(getRy(np.pi/2), CD_point)
    CD_T_Ci_np = getT(get1(), Ci_point)
    CD_T_Ci2_np = getT(get1(), Ci2_point)
    CD_T_C_np = getT(get1(), C_point)
    CT_T_C_np = getT(get1(), CTO_point)
    TCP_T_CT = getT(getRz(np.deg2rad(-50)), [0,0,0])

    UR_T_R_np = getT(getRz(angle_R), R_point)
    R_T_RSP_np = getT(get1(), RSP_point)
    R_T_RSP2_np = getT(get1(), RSP2_point)
    CT_T_RSP_np = getT(getRy(np.pi/2) @ getRx(np.pi), CTS_tuned_point)

    UR_T_TCP0_np = UR_T_CD_np @ CD_T_Ci_np @ matrixInvert(CT_T_C_np) @ matrixInvert(TCP_T_CT) #Cup Dispenser int
    UR_T_TCP1_np = UR_T_CD_np @ CD_T_C_np @ matrixInvert(CT_T_C_np) @ matrixInvert(TCP_T_CT) #Cup Dispenser
    UR_T_TCP3_np = UR_T_CD_np @ CD_T_Ci2_np @ matrixInvert(CT_T_C_np) @ matrixInvert(TCP_T_CT) #Cup Dispenser int 2
    UR_T_TCP2_np = UR_T_R_np @ R_T_RSP_np @ matrixInvert(CT_T_RSP_np) @ matrixInvert(TCP_T_CT) #Rancilio
    UR_T_TCP4_np = UR_T_R_np @ R_T_RSP2_np @ matrixInvert(CT_T_RSP_np) @ matrixInvert(TCP_T_CT) #Rancilio exit

    UR_T_CDi = rm.Mat(UR_T_TCP0_np.tolist())
    UR_T_CDi2 = rm.Mat(UR_T_TCP3_np.tolist())
    UR_T_CD = rm.Mat(UR_T_TCP1_np.tolist())
    UR_T_R = rm.Mat(UR_T_TCP2_np.tolist())
    UR_T_R2 = rm.Mat(UR_T_TCP4_np.tolist())

    tls.cup_tool_attach_r_ati()
    tls.cup_tool_open_ur5()
    UR5.MoveJ(intermediate_0, blocking=True)
    UR5.MoveL(UR_T_CDi, blocking=True)
    UR5.MoveL(UR_T_CD, blocking=True)
    tls.cup_tool_shut_ur5()
    UR5.MoveL(UR_T_CDi2, blocking=True)
    UR5.MoveL(intermediate_1, blocking=True)
    UR5.MoveL(intermediate_2, blocking=True)
    UR5.MoveJ(UR_T_R, blocking=True)
    tls.cup_tool_open_ur5()
    UR5.MoveJ(UR_T_R2, blocking=True)
    # UR5.MoveL(intermediate_2, blocking=True)
    UR5.MoveL(intermediate_1, blocking=True)
    tls.cup_tool_shut_ur5()
    tls.cup_tool_detach_r_ati()

def unlockRancilioScale(): # (o - Jaxon) 
    intermediate_0 = [-88.898774, -70.953538, -130.619496, -67.564767, 90.669574, -232.837611] #Between getting tool and lock

    Rancilio_point1 = np.array([0,   0,      0])
    Rancilio_point2 = np.array([0,	-38,	0])
    World_point1 = np.array([-388.3,	-327.9,	42.6])
    World_point2 = np.array([-421.2,	-308.6,	42.6])
    diff_UR = World_point2 - World_point1
    diff_MS = Rancilio_point2 - Rancilio_point1
    cross = diff_MS[0]*diff_UR[1] - diff_MS[1]*diff_UR[0]
    dot   = diff_MS[0]*diff_UR[0] + diff_MS[1]*diff_UR[1]
    angle_RS = np.arctan2(cross, dot)

    RS_point = [-388.3,	-327.9,	42.6]
    RSS_point = [40,	21.53,	-15] #Rancilio scale lock left
    RSS_point2 = [7.5,	21.53,	-15] #Rancilio scale unlock left (first col)
    RSS_point3 = [7.5,	21.53,	-32] #push down (3rd col)
    MT_point = [0,	0,	102.82]

    UR_T_RS_np = getT(getRz(angle_RS), RS_point)
    UR_T_RSS_np = getT(get1(), RSS_point)
    UT_T_RSS2_np = getT(get1(), RSS_point2) #unlock point
    UT_T_RSS3_np = getT(get1(), RSS_point3) #push down
    MT_T_RSS_np = getT(getRy(np.pi) @ getRx(np.pi/4), MT_point)
    TCP_T_CT = getT(getRz(np.deg2rad(-140)), [0,0,0])

    UR_T_TCP_np = UR_T_RS_np @ UR_T_RSS_np @ matrixInvert(MT_T_RSS_np) @ matrixInvert(TCP_T_CT)  #Locked point
    UR_T_TCP2_np = UR_T_RS_np @ UT_T_RSS2_np @ matrixInvert(MT_T_RSS_np) @ matrixInvert(TCP_T_CT)  #unlocked point
    UR_T_TCP3_np = UR_T_RS_np @ UT_T_RSS3_np @ matrixInvert(MT_T_RSS_np) @ matrixInvert(TCP_T_CT)  #push down point

    UR_T_RS = rm.Mat(UR_T_TCP_np.tolist())
    UR_T_RS2 = rm.Mat(UR_T_TCP2_np.tolist())
    UR_T_RS3 = rm.Mat(UR_T_TCP3_np.tolist())
    client = modbus_scale_client.ModbusScaleClient(host = IP_RANCILIO_2)

    #tls.mazzer_tool_attach_r_ati()
    UR5.MoveJ(intermediate_0, blocking=True)
    client.tare()
    UR5.MoveJ(UR_T_RS, blocking=True)
    UR5.MoveJ(UR_T_RS2, blocking=True)
    UR5.MoveJ(UR_T_RS3, blocking=True)
    UR5.MoveJ(UR_T_RS2, blocking=True)

def grinderOn():
    J_1 = [98.370000, -75.250000, 128.140000, -184.010000, -11.500000, -211.550000]
    #J_1 = [-59.830000, -113.900000, -124.070000, -99.660000, -172.880000, -164.750000]
    J_2 = [-53.750000, -105.760000, -124.070000, 7.040000, 118.440000, -183.950000]
    int1 = [95.781338, -59.044544, 116.215955, -152.890144, -10.839911, -256.647057]
    int2 = [96.340000, -115.930000, 124.070000, 339.660000, 34.580000, -36.610000]

    UR_T_M_np = np.array([[ -0.8818,    0.1913,    -0.4312,    500.1 ],
                        [ -0.4717,   -0.3576,     0.8060,   -423.7 ],
                        [ 0.0000,     0.9141,     0.4055,    316.9 ],
                        [ 0.000000, 0.000000, 0.000000, 1.000000 ]])

    R_I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    R_face     = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
    #R_approach = R_face @ getRx(np.radians(20)) @ getRy(np.radians(35))
    R_approach = R_face @ getRy(np.radians(25))


    M_T_MBOi_np = getT(R_approach, [90.9, -188.3, -144.2])
    M_T_MBO_np = getT(R_approach, [82.699, -182.63, -149.649])
      
    M_T_MBCi_np = getT(R_approach, [96, -182.6, -159.2-12]) 
    M_T_MBC_np = getT(R_approach, [86.617, -175.86, -164.812-12]) 

    MT_T_CGO_np = np.array([[ 1,     0,     0,      0],
                                [ 0,     1,     0,  0],
                                [ 0,     0,     1,  102.82],
                                [ 0,     0,     0,     1]])

    UR_T_MBOi_np = UR_T_M_np @ M_T_MBOi_np @ matrixInvert(MT_T_CGO_np)
    UR_T_MBCi_np = UR_T_M_np @ M_T_MBCi_np @ matrixInvert(MT_T_CGO_np)
    UR_T_MBO_np = UR_T_M_np @ M_T_MBO_np @ matrixInvert(MT_T_CGO_np)
    UR_T_MBC_np = UR_T_M_np @ M_T_MBC_np @ matrixInvert(MT_T_CGO_np)
    

    UR_T_MBOi = rm.Mat(UR_T_MBOi_np.tolist())
    UR_T_MBCi = rm.Mat(UR_T_MBCi_np.tolist())
    UR_T_MBO = rm.Mat(UR_T_MBO_np.tolist())
    UR_T_MBC = rm.Mat(UR_T_MBC_np.tolist())

    UR5.MoveJ(J_1, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_MBOi, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_MBO, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_MBOi, blocking=True)
    sleep(15)
    UR5.MoveJ(UR_T_MBCi, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_MBC, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_MBCi, blocking=True)
    sleep(1)
    # sleep(5)
    # UR5.MoveJ(int1, blocking=True)
    # UR5.MoveJ(int2, blocking=True)

def pullHotWater(): # (p - Jaxon)
    intermediate_0 = [-98.996794, -61.359390, -147.361249, 24.632723, 62.874704, -267.028668]
    intermediate_1 = [-117.597322, -95.289533, -132.121703, 44.118223, 81.079990, -268.547203]

    int1 = [-87.292774, -87.620596, -108.619335, -100.550638, 51.417359, -98.833744]
    int2 = [-87.290000, -79.320000, -122.030000, -166.780000, -38.640000, -98.830000]

    Rancilio_point1 = np.array([0,   0,      0])
    Rancilio_point2 = np.array([0,	220.42,	0])
    World_point1 = np.array([-414.1, -478.5,	350.3])
    World_point2 = np.array([-602.6, -364.9,	350.3])
    diff_UR = World_point2 - World_point1
    diff_MS = Rancilio_point2 - Rancilio_point1
    cross = diff_MS[0]*diff_UR[1] - diff_MS[1]*diff_UR[0]
    dot   = diff_MS[0]*diff_UR[0] + diff_MS[1]*diff_UR[1]
    angle_CM = np.arctan2(cross, dot)

    CM_point = [-414.1, -478.5,	350.3]
    CMB_point = [49.1,	38.6,	-55]
    button_on_press = [40,	38.6,	-55]
    button_off_hover = [49.1,	38.6,	-72]
    button_off_press = [40,	38.6,	-72]
    MT_point = [0,	0,	102.82]

    UR_T_CM_np = getT(getRz(angle_CM), CM_point)
    CM_T_CMB_np = getT(get1(), CMB_point)
    CM_T_CMB2_np = getT(get1(), button_on_press)
    CM_T_CMB3_np = getT(get1(), button_off_hover)
    CM_T_CMB4_np = getT(get1(), button_off_press)
    MT_T_CMB = getT(getRy(-np.pi/2)@getRx(-np.pi), MT_point)

    UR_T_TCP_np = UR_T_CM_np @ CM_T_CMB_np @ matrixInvert(MT_T_CMB)
    UR_T_TCP2_np = UR_T_CM_np @ CM_T_CMB2_np @ matrixInvert(MT_T_CMB)
    UR_T_TCP3_np = UR_T_CM_np @ CM_T_CMB3_np @ matrixInvert(MT_T_CMB)
    UR_T_TCP4_np = UR_T_CM_np @ CM_T_CMB4_np @ matrixInvert(MT_T_CMB)

    UR_T_TCP = rm.Mat(UR_T_TCP_np.tolist()) #Button on hover
    UR_T_TCP2 = rm.Mat(UR_T_TCP2_np.tolist()) #Button on press
    UR_T_TCP3 = rm.Mat(UR_T_TCP3_np.tolist()) #Button off hover
    UR_T_TCP4 = rm.Mat(UR_T_TCP4_np.tolist()) #Button off press

    client = modbus_scale_client.ModbusScaleClient(host = IP_RANCILIO_2)

    UR5.MoveJ(int1, blocking=True)
    UR5.MoveJ(int2, blocking=True)
    UR5.MoveJ(UR_T_TCP, blocking=True)
    UR5.MoveJ(UR_T_TCP2, blocking=True)
    UR5.MoveJ(UR_T_TCP, blocking=True)
    UR5.MoveJ(UR_T_TCP3, blocking=True)
    #Code the reads scale
    while(1):
        print(client.read())
        if client.read() >= 32:
            break
    UR5.MoveJ(UR_T_TCP4, blocking=True)
    UR5.MoveJ(UR_T_TCP3, blocking=True)
    UR5.MoveJ(intermediate_1, blocking=True)
    UR5.MoveJ(intermediate_0, blocking=True)

def lockRancilioScale(): # (q - Jaxon)
    intermediate_0 = [-88.898774, -70.953538, -130.619496, -67.564767, 90.669574, -232.837611] #Between getting tool and lock

    Rancilio_point1 = np.array([0,   0,      0])
    Rancilio_point2 = np.array([0,	-38,	0])
    World_point1 = np.array([-388.3,	-327.9,	42.6])
    World_point2 = np.array([-421.2,	-308.6,	42.6])
    diff_UR = World_point2 - World_point1
    diff_MS = Rancilio_point2 - Rancilio_point1
    cross = diff_MS[0]*diff_UR[1] - diff_MS[1]*diff_UR[0]
    dot   = diff_MS[0]*diff_UR[0] + diff_MS[1]*diff_UR[1]
    angle_RS = np.arctan2(cross, dot)

    RS_point = [-388.3,	-327.9,	42.6]
    RSS_point = [40,	21.53,	-15] #Rancilio scale lock left
    RSS_point2 = [7.5,	21.53,	-15] #Rancilio scale unlock left (first col)
    RSS_point3 = [40,	21.53,	-32] #push down (3rd col)
    MT_point = [0,	0,	102.82]

    UR_T_RS_np = getT(getRz(angle_RS), RS_point)
    UR_T_RSS_np = getT(get1(), RSS_point)
    UT_T_RSS2_np = getT(get1(), RSS_point2) #unlock point
    UT_T_RSS3_np = getT(get1(), RSS_point3) #push down
    MT_T_RSS_np = getT(getRy(np.pi) @ getRx(np.pi/4), MT_point)
    TCP_T_CT = getT(getRz(np.deg2rad(-140)), [0,0,0])

    UR_T_TCP_np = UR_T_RS_np @ UR_T_RSS_np @ matrixInvert(MT_T_RSS_np) @ matrixInvert(TCP_T_CT) #Locked point
    UR_T_TCP2_np = UR_T_RS_np @ UT_T_RSS2_np @ matrixInvert(MT_T_RSS_np) @ matrixInvert(TCP_T_CT) #unlocked point
    UR_T_TCP3_np = UR_T_RS_np @ UT_T_RSS3_np @ matrixInvert(MT_T_RSS_np) @ matrixInvert(TCP_T_CT) #push down point

    UR_T_RS = rm.Mat(UR_T_TCP_np.tolist())
    UR_T_RS2 = rm.Mat(UR_T_TCP2_np.tolist())
    UR_T_RS3 = rm.Mat(UR_T_TCP3_np.tolist())

    UR5.MoveJ(intermediate_0, blocking=True)
    UR5.MoveJ(UR_T_RS2, blocking=True)
    UR5.MoveJ(UR_T_RS, blocking=True)
    UR5.MoveJ(UR_T_RS3, blocking=True)
    UR5.MoveJ(UR_T_RS, blocking=True)
    UR5.MoveJ(intermediate_0, blocking=True)
    tls.mazzer_tool_detach_r_ati()

def serveCoffee(): # (r - Jaxon)
    intermediate_0 = [-99.500658, -71.447990, -145.525786, -150.086183, -8.803035, -212.850094]
    intermediate_1 = [-100.897369, -71.770865, -145.306458, -148.991403, -10.188578, -213.827318]
    intermediate_2 = [-159.336675, -63.682714, -141.844440, -153.885224, -113.745689, -218.627199]
    intermediate_3 = [-150.610339, -68.519970, -143.081678, -147.684074, -119.549354, -218.622039]

    int1 = [-167.320000, -69.150000, -142.370001, -146.440001, -162.710004, -214.502032]
    int2 = [-205.860000, -77.290000, -144.410000, -136.270000, -150.510000, -217.630000]
    int3 = [-195.710000, -52.880000, -144.410000, -162.710000, -106.710000, -218.940000]
    int4 = [-195.710000, -40.680000, -144.410000, -191.190000, -106.710000, -218.940000]
    int5 = [-195.710000, -40.680000, -126.100000, -233.900000, -106.710000, -218.940000]
    int6 = [-194.710000, -58.980000, -95.590000, -233.900000, -106.710000, -218.940000]

    CTS_tuned_point = [-120,	    0,	186.62]

    R_point = [-388.3,	-327.9,	42.6]
    RSP_point = [157.52,	-19,	32] # RS pan
    RSP2_point = [90,	-19,	29]
    Serve_point = [-420,	150,	0]
    ServeI_point = [-480,	150,	0]

    Rancilio_point1 = np.array([0,   0,      0])
    Rancilio_point2 = np.array([0,	-38,	0])
    World_point1 = np.array([-388.3,	-327.9,	42.6])
    World_point2 = np.array([-421.2,	-308.6,	42.6])
    diff_UR = World_point2 - World_point1
    diff_MS = Rancilio_point2 - Rancilio_point1
    cross = diff_MS[0]*diff_UR[1] - diff_MS[1]*diff_UR[0]
    dot   = diff_MS[0]*diff_UR[0] + diff_MS[1]*diff_UR[1]
    angle_R = np.arctan2(cross, dot)

    TCP_T_CT = getT(getRz(np.deg2rad(-50)), [0,0,0])

    UR_T_R_np = getT(getRz(angle_R), R_point)
    R_T_RSP_np = getT(get1(), RSP_point)
    R_T_RSP2_np = getT(get1(),RSP2_point )
    CT_T_RSP_np = getT(getRy(np.pi/2) @ getRx(np.pi), CTS_tuned_point)

    UR_T_SERVE = getT(getRz(np.pi), Serve_point)
    UR_T_SERVEi = getT(getRz(np.pi), ServeI_point)

    UR_T_TCP2_np = UR_T_R_np @ R_T_RSP_np @ matrixInvert(CT_T_RSP_np) @ matrixInvert(TCP_T_CT) #Rancilio
    UR_T_TCP4_np = UR_T_R_np @ R_T_RSP2_np @ matrixInvert(CT_T_RSP_np) @ matrixInvert(TCP_T_CT) #Rancilio
    UR_T_TCP_np = UR_T_SERVE@ matrixInvert(CT_T_RSP_np) @ matrixInvert(TCP_T_CT) #Serve
    UR_T_TCP3_np = UR_T_SERVEi@ matrixInvert(CT_T_RSP_np) @ matrixInvert(TCP_T_CT) #Serve int

    UR_T_R = rm.Mat(UR_T_TCP2_np.tolist())
    UR_T_R2 = rm.Mat(UR_T_TCP4_np.tolist())
    UR_T_S = rm.Mat(UR_T_TCP_np.tolist())
    UR_T_Si = rm.Mat(UR_T_TCP3_np.tolist())

    tls.cup_tool_attach_r_ati()
    UR5.MoveJ(intermediate_0, blocking=True)
    tls.cup_tool_open_ur5()
    UR5.MoveL(intermediate_1, blocking=True)
    UR5.MoveL(UR_T_R2, blocking=True)
    UR5.MoveL(UR_T_R, blocking=True)
    tls.cup_tool_shut_ur5()
    UR5.MoveL(UR_T_R2, blocking=True)
    UR5.MoveJ(int2, blocking=True)
    UR5.MoveJ(UR_T_Si, blocking=True)
    tls.cup_tool_open_ur5()
    UR5.MoveL(UR_T_S, blocking=True)
    UR5.MoveJ(int3, blocking=True)
    UR5.MoveJ(int4, blocking=True)
    UR5.MoveJ(int5, blocking=True)
    UR5.MoveJ(int6)
    tls.cup_tool_shut_ur5()
    tls.cup_tool_detach_r_ati()

def unlockScale(): # (b)
    UR1 = np.array([439.4, -277.9, 41.9]) #Mazzer scale global
    UR2 = np.array([406.4, -296.9, 41.6]) # Second mazzer scale global
    MS1 = np.array([0,	0,	0]) #Mazzer scale local
    MS2 = np.array([0, -38,	0]) # Mazzer scale local

    diff_UR = UR2 - UR1
    diff_MS = MS2 - MS1

    cross = diff_MS[0]*diff_UR[1] - diff_MS[1]*diff_UR[0]
    dot   = diff_MS[0]*diff_UR[0] + diff_MS[1]*diff_UR[1]
    angle_MS = np.arctan2(cross, dot)

    MS_point = np.array([439.4, -277.9, 41.9])
    MSL_point = np.array([31.5,	-59.53,	-15]) #Mazzer Scale Right Lock
    MT_point = np.array([0, 0, 110])
    MSU_point = np.array([0,	-59.53,	-20]) #Mazzer Scale Right Unlock

    MS_int_point1 = [-15.638633, -83.109581, -114.678509, -71.904552, 90.033612, -20.127040]

    UR_T_MS_np = getT(getRz(angle_MS), MS_point)
    MS_T_MSL_np = getT(get1(), MSL_point) #Point before unlock
    MS_T_MSU_np = getT(get1(), MSU_point) #Point after unlock
    MT_T_MSL_np = getT(getRx(np.pi), MT_point) 

    UR_T_MSL_np = UR_T_MS_np @ MS_T_MSL_np @ matrixInvert(MT_T_MSL_np)
    UR_T_MSU_np = UR_T_MS_np @ MS_T_MSU_np @ matrixInvert(MT_T_MSL_np)

    UR_T_MSL = rm.Mat(UR_T_MSL_np.tolist())
    UR_T_MSU = rm.Mat(UR_T_MSU_np.tolist())

    #Movement Program
    tls.mazzer_tool_attach_r_ati()
    UR5.MoveJ(MS_int_point1)
    UR5.MoveJ(UR_T_MSL, blocking=True)
    sleep(1)
    UR5.MoveL(UR_T_MSU, blocking = True)
    sleep(0.5)
    UR5.MoveJ(MS_int_point1)
    tls.mazzer_tool_detach_r_ati()

def turnMazzaOn(): # (c)
    mazzerOnButton = np.array([90.9, -188.3, -144.2]) 
    mazzerOffButton = np.array([93.7, -182.6, -159.2]) 

def unlockMazzerScale(): # (b - Adam)
    visual_program = RDK.Item("Show_Mazzer_Scale_Rancilio_Tool", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()

    


    #UR T MazzerScale
    UR_T_ms_np = np.array([[1, 0, 0, 439.4],
                           [0, 1, 0, -277.9],
                           [0, 0, 1, 41.9],
                           [0, 0, 0, 1]])
    
    theta = np.arctan2(-1254,722)
    # angle = np.deg2rad(theta)
    
    #UR T MazzerScale (with rotation)
    UR_T_ms_np[ :3, :3] = getRz(theta)

    #UR T left lock (not used)
    ms_T_ll_np = np.array([[1, 0, 0, 31.5],
                            [0, 1, 0, 21.53],
                            [0, 0, 1, -15],
                            [0, 0, 0, 1]])

    #UR T right lock ()
    ms_T_rl_np = np.array([[1, 0, 0, 31.5],
                            [0, 1, 0, -59.53],
                            [0, 0, 1, -15],
                            [0, 0, 0, 1]])

    #Mazzer Tool Tip T right lever
    MTT_T_RL_np = np.eye(4)
    MTT_T_RL_np[ :3, :3] = getRx(np.deg2rad(-180 - 40))

    #Tool Connection Point T Mazza Tool Frame
    TCP_T_MT_np = np.eye(4)
    TCP_T_MT_np[ :3, :3] = getRz(np.deg2rad(-50))

    #Mazzer Tool Frame T Mazza Tool Tip
    MTF_T_MTT_np = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 102.82],
                            [0, 0, 0, 1]])
    MTF_T_MTT_np[ :3, :3] =  getRz(np.deg2rad(180))


    UR_T_ll_np = UR_T_ms_np  @ ms_T_ll_np
    UR_T_rl_np = UR_T_ms_np  @ ms_T_rl_np


# Right on keypoint
    # UR_T_TCP_np = UR_T_ms_np @ ms_T_rl_np @ matrixInvert(MTT_T_RL_np) @ matrixInvert(MTF_T_MTT_np) @ matrixInvert(TCP_T_MT_np)

# SLightly above lever point
    MTT_T_RL_1_np = np.array([[1, 0, 0, -10],
                            [0, 1, 0, -3],
                            [0, 0, 1, -4],
                            [0, 0, 0, 1]])

    UR_T_TCP_np = UR_T_ms_np @ ms_T_rl_np @ matrixInvert(MTT_T_RL_1_np) @ matrixInvert(MTT_T_RL_np) @ matrixInvert(MTF_T_MTT_np) @ matrixInvert(TCP_T_MT_np)

# horizontal push
    MTT_T_RL_2_np = np.array([[1, 0, 0, 39],  # <- horizontal move
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    UR_T_TCP_push_np = UR_T_ms_np @ ms_T_rl_np @ matrixInvert(MTT_T_RL_2_np) @matrixInvert(MTT_T_RL_1_np) @ matrixInvert(MTT_T_RL_np) @ matrixInvert(MTF_T_MTT_np) @ matrixInvert(TCP_T_MT_np)

# vertical press
    MTT_T_RL_3_np = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 18.5], # < - push down
                            [0, 0, 0, 1]])
    UR_T_TCP_press_np = UR_T_ms_np @ ms_T_rl_np @ matrixInvert(MTT_T_RL_3_np) @ matrixInvert(MTT_T_RL_2_np) @ matrixInvert(MTT_T_RL_1_np) @ matrixInvert(MTT_T_RL_np) @ matrixInvert(MTF_T_MTT_np) @ matrixInvert(TCP_T_MT_np)


    # UR_T_ms = rm.Mat(UR_T_ms_np.tolist())
    # UR_T_ll = rm.Mat(UR_T_ll_np.tolist())
    # UR_T_rl = rm.Mat(UR_T_ll_np.tolist())
    UR_T_TCP = rm.Mat(UR_T_TCP_np.tolist())
    UR_T_TCP_push = rm.Mat(UR_T_TCP_push_np.tolist())
    UR_T_TCP_press = rm.Mat(UR_T_TCP_press_np.tolist())

    tls.mazzer_tool_attach_l_ati()
    # UR5.MoveJ(UR_T_ms, blocking=True)
    # time.sleep(2)
    # UR5.MoveJ(UR_T_rl, blocking=True)


    i0 = [101.382566, -101.812574, 125.101561, -113.291131, -89.994282, -223.617740]
    i1 = [103.033239, -82.535820, 115.118739, -108.783936, -51.008353, -241]

    UR5.MoveJ(i0, blocking=True)
    UR5.MoveJ(i1, blocking=True)

    UR5.MoveL(UR_T_TCP, blocking=True)
    time.sleep(1)
    UR5.MoveL(UR_T_TCP_push, blocking=True)
    vis_unlock_scale()
    time.sleep(1)

    UR5.MoveL(UR_T_TCP_press, blocking=True)
    time.sleep(1)
    UR5.MoveL(UR_T_TCP_push, blocking=True)
    time.sleep(1)
    UR5.MoveL([101.040317, -89.737465, 122.480776, -107.396598, -51.509888, -243.472947], blocking=True)

def lockMazzerScale(): # (b - Adam)


    #UR T MazzerScale
    UR_T_ms_np = np.array([[1, 0, 0, 439.4],
                           [0, 1, 0, -277.9],
                           [0, 0, 1, 41.9],
                           [0, 0, 0, 1]])
    
    theta = np.arctan2(-1254,722)
    # angle = np.deg2rad(theta)
    
    #UR T MazzerScale (with rotation)
    UR_T_ms_np[ :3, :3] = getRz(theta)

    #UR T left lock (not used)
    ms_T_ll_np = np.array([[1, 0, 0, 31.5],
                            [0, 1, 0, 21.53],
                            [0, 0, 1, -15],
                            [0, 0, 0, 1]])

    #UR T right lock ()
    ms_T_rl_np = np.array([[1, 0, 0, 31.5],
                            [0, 1, 0, -59.53],
                            [0, 0, 1, -15],
                            [0, 0, 0, 1]])

    #Mazzer Tool Tip T right lever
    MTT_T_RL_np = np.eye(4)
    MTT_T_RL_np[ :3, :3] = getRx(np.deg2rad(-180 - 40))

    #Tool Connection Point T Mazza Tool Frame
    TCP_T_MT_np = np.eye(4)
    TCP_T_MT_np[ :3, :3] = getRz(np.deg2rad(-50))

    #Mazzer Tool Frame T Mazza Tool Tip
    MTF_T_MTT_np = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 102.82],
                            [0, 0, 0, 1]])
    MTF_T_MTT_np[ :3, :3] =  getRz(np.deg2rad(180))

    UR_T_ll_np = UR_T_ms_np  @ ms_T_ll_np
    UR_T_rl_np = UR_T_ms_np  @ ms_T_rl_np


# Right on keypoint
    # UR_T_TCP_np = UR_T_ms_np @ ms_T_rl_np @ matrixInvert(MTT_T_RL_np) @ matrixInvert(MTF_T_MTT_np) @ matrixInvert(TCP_T_MT_np)

# SLightly above lever point
    MTT_T_RL_1_np = np.array([[1, 0, 0, 42],
                            [0, 1, 0, -3],
                            [0, 0, 1, -5],
                            [0, 0, 0, 1]])
    UR_T_TCP_np = UR_T_ms_np @ ms_T_rl_np @ matrixInvert(MTT_T_RL_1_np) @ matrixInvert(MTT_T_RL_np) @ matrixInvert(MTF_T_MTT_np) @ matrixInvert(TCP_T_MT_np)

    
# horizontal push
    MTT_T_RL_2_np = np.array([[1, 0, 0, -60],  # <- horizontal move
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    UR_T_TCP_push_np = UR_T_ms_np @ ms_T_rl_np @ matrixInvert(MTT_T_RL_2_np) @matrixInvert(MTT_T_RL_1_np) @ matrixInvert(MTT_T_RL_np) @ matrixInvert(MTF_T_MTT_np) @ matrixInvert(TCP_T_MT_np)

# vertical press
    MTT_T_RL_3_np = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 21], # < - push down
                            [0, 0, 0, 1]])
    UR_T_TCP_press_np = UR_T_ms_np @ ms_T_rl_np @ matrixInvert(MTT_T_RL_3_np) @ matrixInvert(MTT_T_RL_2_np) @ matrixInvert(MTT_T_RL_1_np) @ matrixInvert(MTT_T_RL_np) @ matrixInvert(MTF_T_MTT_np) @ matrixInvert(TCP_T_MT_np)


    # UR_T_ms = rm.Mat(UR_T_ms_np.tolist())
    # UR_T_ll = rm.Mat(UR_T_ll_np.tolist())
    # UR_T_rl = rm.Mat(UR_T_ll_np.tolist())
    UR_T_TCP = rm.Mat(UR_T_TCP_np.tolist())
    UR_T_TCP_push = rm.Mat(UR_T_TCP_push_np.tolist())
    UR_T_TCP_press = rm.Mat(UR_T_TCP_press_np.tolist())

    # tls.mazzer_tool_attach_l_ati()
    # UR5.MoveJ(UR_T_ms, blocking=True)
    # time.sleep(2)
    # UR5.MoveJ(UR_T_rl, blocking=True)
    J_intermediate = [-48.946587, -84.582052, -114.542571, -70.572116, 90.218247, -192.954747]
    i0 = [89.627469, -110.506418, 107.845857, -63.966418, -55.313743, -256.891531]
    i1 = [101.040317, -89.737465, 122.480776, -107.396598, -51.509888, -243.472947]
    i2 = [103.487608, -80.719220, 113.070925, -108.910896, -50.901359, -240.431414]
    i_final = [105.741261, -100.967002, 127.604530, -115.595132, -86.559667, -233.489680]

    # UR5.MoveJ(J_intermediate, blocking=True)
    UR5.MoveJ(i0, blocking=True)
    UR5.MoveJ(i1, blocking=True)

    UR5.MoveL(UR_T_TCP, blocking=True)
    time.sleep(1)
    UR5.MoveL(UR_T_TCP_push, blocking=True)
    vis_lock_scale()
    time.sleep(1)
    UR5.MoveL(UR_T_TCP_press, blocking=True)
    time.sleep(1)
    UR5.MoveL(UR_T_TCP_push, blocking=True)
    time.sleep(1)

    UR5.MoveJ(i2, blocking=True)
    UR5.MoveJ(i_final, blocking=True)
    tls.mazzer_tool_detach_l_ati()



    # j_intermediate = [152.420000, -46.640000, -97.630000, -75.770000, 149.760000, 51.700000]
    # UR5.MoveJ(j_intermediate, blocking=True)

def weighRancilioTool(): # (a - Jaxon)
    int_point1 = [130.820000, -103.730000, 103.730000, -30.510000, -4.070000, -231.070000]
    int_point2 = [130.820000, -103.730000, 134.240000, -32.540000, 73.220000, -231.070000]
    int_point3 = [149.427186, -84.770823, 136.659521, -51.525757, 120.667164, -219.223144]

    int_point4 = [138.290000, -81.360000, 134.240000, -57.660000, 109.600000, -220.570000]
    int_point5 = [138.290000, -105.760000, 128.140000, -20.340000, 109.600000, -220.570000]

    UR1 = np.array([439.4, -277.9, 41.9]) #Mazzer scale global
    UR2 = np.array([406.4, -296.9, 41.6]) # Second mazzer scale global
    MS1 = np.array([0,	0,	0]) #Mazzer scale local
    MS2 = np.array([0, -38,	0]) # Mazzer scale local

    diff_UR = UR2 - UR1
    diff_MS = MS2 - MS1

    cross = diff_MS[0]*diff_UR[1] - diff_MS[1]*diff_UR[0]
    dot   = diff_MS[0]*diff_UR[0] + diff_MS[1]*diff_UR[1]
    angle_MS = np.arctan2(cross, dot)

    MS_point = np.array([439.4, -277.9, 41.9])
    MS_ballbearing_hover_point = np.array([-12.6, -19, 20])
    MS_ballbearing_point = np.array([-12.6, -19, 10.5]) #Point IRL from excel sheet
    MS_ballbearing_point2 = np.array([-32, -19, 10.5]) #move back

    RT_point = np.array([-32, 0, 28.07])
    TCP_point = np.array([0, 0, 0])

    UR_T_MS_np = getT(getRz(angle_MS), MS_point)
    MS_T_BB_np = getT(get1(), MS_ballbearing_point)
    Hover_T_BB_np = getT(get1(), MS_ballbearing_hover_point)
    RT_T_BB_np = getT(getRx(np.pi) @ getRy(-np.pi/2), RT_point)
    TCP_T_RT_np = getT(getRz(np.deg2rad(-50)), TCP_point)
    BACKUP_np = getT(get1(),MS_ballbearing_point2) 
    hover_np = UR_T_MS_np @ Hover_T_BB_np @ matrixInvert(RT_T_BB_np) @ matrixInvert(TCP_T_RT_np)
    back_np = UR_T_MS_np @ BACKUP_np @ matrixInvert(RT_T_BB_np) @ matrixInvert(TCP_T_RT_np)
    Final_np = UR_T_MS_np @ MS_T_BB_np @ matrixInvert(RT_T_BB_np) @ matrixInvert(TCP_T_RT_np)

    Hover = rm.Mat(hover_np.tolist())
    UR_T_RT = rm.Mat(Final_np.tolist()) 
    backup = rm.Mat(back_np.tolist())

    tls.rancilio_tool_attach_l_ati()
    sleep(1)
    UR5.MoveJ(int_point1, blocking=True)
    UR5.MoveJ(int_point2 , blocking=True)
    UR5.MoveJ(int_point3, blocking=True)
    UR5.MoveJ(Hover, blocking=True)
    UR5.MoveL(UR_T_RT)
    tls.student_tool_detach()
    visual_program = RDK.Item("Show_Mazzer_Scale_Rancilio_Tool", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()
    UR5.MoveL(backup, blocking=True)
    UR5.MoveJ(int_point4 , blocking=True)
    UR5.MoveJ(int_point5, blocking=True)
    if (RDK.RunMode() == RUNMODE_SIMULATE):
            tls.rancilio_tool_detach_l_ati()

def removeRancilioToolMazzer(): # (f - Jaxon)
    int1 = [-29.410000, -77.290000, -105.760000, -120.000000, 30.510000, 140.210000]
    int2 = [-17.240000, -101.690000, -142.370000, -122.030000, -61.020000, 140.210000]
    int3 = [-10.067410, -82.691043, -145.138027, -140.513625, -74.279443, 141.491984]

    UR1 = np.array([439.4, -277.9, 41.9]) #Mazzer scale global
    UR2 = np.array([406.4, -296.9, 41.6]) # Second mazzer scale global
    MS1 = np.array([0,	0,	0]) #Mazzer scale local
    MS2 = np.array([0, -38,	0]) # Mazzer scale local

    diff_UR = UR2 - UR1
    diff_MS = MS2 - MS1

    cross = diff_MS[0]*diff_UR[1] - diff_MS[1]*diff_UR[0]
    dot   = diff_MS[0]*diff_UR[0] + diff_MS[1]*diff_UR[1]
    angle_MS = np.arctan2(cross, dot)

    MS_point = np.array([439.4, -277.9, 41.9])
    MS_ballbearing_hover_point = np.array([-32, -19, 14.5]) #Hover behind
    MS_ballbearing_lift_point = np.array([-12.1, -19, 25]) #lift
    MS_ballbearing_liftH_point = np.array([-26, -19, 20]) #hover and lift
    MS_ballbearing_point = np.array([-12.1, -19, 10.5]) #Point IRL from excel sheet

    RT_point = np.array([-32, 0, 28.07])
    TCP_point = np.array([0, 0, 0])

    UR_T_MS_np = getT(getRz(angle_MS), MS_point)
    MS_T_BB_np = getT(get1(), MS_ballbearing_point)
    Hover_T_BB_np = getT(get1(), MS_ballbearing_hover_point)
    Lift_T_BB_np = getT(get1(), MS_ballbearing_lift_point)
    HL_T_BB_np = getT(get1(), MS_ballbearing_liftH_point)
    RT_T_BB_np = getT(getRx(np.pi) @ getRy(-np.pi/2), RT_point)
    TCP_T_RT_np = getT(getRz(np.deg2rad(-50)), TCP_point)

    Lift_np = UR_T_MS_np @ Lift_T_BB_np @ matrixInvert(RT_T_BB_np) @ matrixInvert(TCP_T_RT_np)
    hover_np = UR_T_MS_np @ Hover_T_BB_np @ matrixInvert(RT_T_BB_np) @ matrixInvert(TCP_T_RT_np)
    HL_np = UR_T_MS_np @ HL_T_BB_np @ matrixInvert(RT_T_BB_np) @ matrixInvert(TCP_T_RT_np)
    Final_np = UR_T_MS_np @ MS_T_BB_np @ matrixInvert(RT_T_BB_np) @ matrixInvert(TCP_T_RT_np)

    Hover = rm.Mat(hover_np.tolist())
    Lift = rm.Mat(Lift_np.tolist())
    HL = rm.Mat(HL_np.tolist())
    UR_T_RT = rm.Mat(Final_np.tolist())

    tool = RDK.Item('Show_Mazzer_Scale_Rancilio_Tool', ITEM_TYPE_PROGRAM)
    tool.RunProgram()
    hidetool = RDK.Item('Hide_Mazzer_Scale_Rancilio_Tool', ITEM_TYPE_PROGRAM)
    showtool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)

    UR5.MoveJ(int1, blocking=True)
    UR5.MoveJ(int2, blocking=True)
    UR5.MoveJ(Hover, blocking=True)
    UR5.MoveL(UR_T_RT, blocking=True)
    hidetool.RunProgram()
    showtool.setVisible(True, False)
    tls.student_tool_attach()
    UR5.MoveJ(Lift, blocking=True)
    UR5.MoveL(HL, blocking=True)
    UR5.MoveJ(int3, blocking=True)
    sleep(1)

def mazzerDosingLever():
    
    visual_program = RDK.Item("Show_Mazzer_Scale_Rancilio_Tool", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()
    #-----------------------------------------------------------------
    #                    Equipment side
    #----------------------------------------------------------------

    #Given coordinate points
    UR_Mazzer_orig = np.array([500.1, -423.7, 316.9])
    UR_Mazzer_point = np.array([554.5, -394.6, 316.9])

    local_Mazzer_orig = np.array([0, 0, 0])
    local_Mazzer_point = np.array([-61.73, 0, 0])

    UR_lidApex = np.array([555.5, -527.2, 581.5])
    local_lidApex = np.array([0, 290.83, 0])

    # Rotation bout x to intermediate frame M_dash
    theta_x = np.deg2rad(68.36284889)
    M_dash_R_M = getRx(theta_x)

    #Rotation bout z to local frame M
    diff_UR_z = UR_Mazzer_point - UR_Mazzer_orig
    diff_M_z = local_Mazzer_point - local_Mazzer_orig
    diff_UR_norm = diff_UR_z / np.linalg.norm(diff_UR_z)
    diff_M_norm = diff_M_z / np.linalg.norm(diff_M_z)

    cross = diff_M_norm[0]*diff_UR_norm[1] - diff_M_norm[1]*diff_UR_norm[0]
    dot   = diff_M_norm[0]*diff_UR_norm[0] + diff_M_norm[1]*diff_UR_norm[1]
    theta_z = np.arctan2(cross, dot)
    # theta_z = np.arctan2(-0.47, -0.882)
    print(f'theta z: {theta_z}')

    # UR T Mazzer local frame (no rotation)
    UR_T_M_np = np.array([[1, 0, 0, 500.1],
                          [0, 1, 0, -423.7],
                          [0, 0, 1, 316.9],
                          [0, 0, 0, 1]])

    # Rotation 
    UR_R_M_np = getRz(theta_z) @ M_dash_R_M
    UR_T_M_np[ :3, :3] = UR_R_M_np   #with rotation

    print(UR_T_M_np)


    #local Mazzer frame T Mazzer Center point
    M_t_MC = np.array([0, 0, 0])
    M_T_MC_np = getT(get1(), M_t_MC)
    M_R_MC = getRx(np.deg2rad(180 + 21.64))
    M_T_MC_np[ :3, :3] = M_R_MC


    # local Mazzer Frame T Mazzer Dose Lever
    M_t_MDL = np.array([84.1, -131.8, -84.2])
    M_T_MDL_np = getT(get1(), M_t_MDL)

    # Relative X-Y
    M_R_MDL_prime = getRx(np.deg2rad(180 + 21.64))
    MDL_prime_R_MDL = getRy(np.deg2rad(20))
    M_R_MDL = M_R_MDL_prime @ MDL_prime_R_MDL
    M_T_MDL_np[ :3, :3] = M_R_MDL


    # -------Add Mazzer center point frame----------

    #local Mazzer frame T Mazzer Center point
    # M_t_MC = np.array([5, 0, -5])
     #               (right, up, back)
    M_t_MC = np.array([0, 0, 1.5])
    M_R_MC = getRx(np.deg2rad( 21.64))
    M_T_MC_np = getT(M_R_MC, M_t_MC)

    # Mazzer Center Point T Mazzer Dose Lever
    MC_T_MDL_np = matrixInvert(M_T_MC_np) @ M_T_MDL_np

    #-----------------------------------------------------------------
    #                       Robot Side
    #----------------------------------------------------------------

    #Tool Connection Point T Mazza Tool Frame
    TCP_T_MTF_np = np.eye(4)
    TCP_T_MTF_np[ :3, :3] = getRz(np.deg2rad(-50))

    # Mazzer Tool Frame T mazzer Dose Bar Edge
    MTF_t_MDBE_np = np.array([-50, 0, 67.06])
    MTF_T_MDBE_np = getT(get1(), MTF_t_MDBE_np)

    # Mazzer Dose Bar edge T Mazzer Dose Lever
    # MDB_T_MDL_np = np.eye(4)
    MDB_T_MDL_np = np.array([[1, 0, 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])


    # UR_T_MDL_np = UR_T_M_np @ M_T_MDL_np

    UR_T_TCP_np = UR_T_M_np @ M_T_MC_np @ MC_T_MDL_np @ matrixInvert(MDB_T_MDL_np) @ matrixInvert(MTF_T_MDBE_np) @ matrixInvert(TCP_T_MTF_np)

    #-----------------------------------------------------------------
    #        Additional positions (around Mazzer Dose Lever Frame)
    #----------------------------------------------------------------
    # Starting point (right behind lever)
    MC_t_p1_np = np.array([[1, 0, 0, -10 ],
                            [0, 1, 0, 20], # <- Height on lever
                            [0, 0, 1, -10],
                            [0, 0, 0, 1]])
    #                        TODO understand / better naming convention
    # MC_t_p1_np = np.array([[1, 0, 0, -5.5],
    #                        [0, 1, 0, 0],
    #                        [0, 0, 1, -11],
    #                        [0, 0, 0, 1]])

    UR_T_TCP_1_np = UR_T_M_np @ M_T_MC_np @ MC_t_p1_np @ MC_T_MDL_np @ matrixInvert(MDB_T_MDL_np) @ matrixInvert(MTF_T_MDBE_np) @ matrixInvert(TCP_T_MTF_np)

    t_out_np = np.array([[1, 0, 0, 35],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

    # t_out_np = np.array([[1, 0, 0, -10],
    #                      [0, 1, 0, 0],
    #                      [0, 0, 1, 0],
    #                      [0, 0, 0, 1]])
    


    def leverRotation(degrees, step_size):
        client = modbus_scale_client.ModbusScaleClient(host = IP_MAZZER_2)
        client.tare()
        lever_theta = 0
        lever_theta2 = 0
        scale_value = 0
        while (scale_value < 19.8):
            if (RDK.RunMode() == RUNMODE_RUN_ROBOT):
                scale_value = client.read()
            

            if (lever_theta <= degrees):
                MC_R_np = np.eye(4)
                MC_R_np[ :3, :3] = getRy(np.deg2rad(lever_theta))
                
                if (RDK.RunMode() == RUNMODE_SIMULATE):
                    scale_value += 0.5

                UR_T_TCP_rot_np = UR_T_M_np @ M_T_MC_np  @ MC_R_np @ MC_t_p1_np@ MC_T_MDL_np @ matrixInvert(MDB_T_MDL_np) @ matrixInvert(MTF_T_MDBE_np) @ matrixInvert(TCP_T_MTF_np)
                UR_T_TCP_rot = rm.Mat(UR_T_TCP_rot_np.tolist())
                UR5.MoveL(UR_T_TCP_rot, blocking=True)
                lever_theta += step_size
                lever_theta2 = lever_theta
            else:
                MC_R_final_np = MC_R_np
                lever_theta2 = lever_theta
                UR_T_TCP_out_np = UR_T_M_np @ M_T_MC_np  @ MC_R_np @ MC_t_p1_np @ t_out_np @ MC_T_MDL_np @ matrixInvert(MDB_T_MDL_np) @ matrixInvert(MTF_T_MDBE_np) @ matrixInvert(TCP_T_MTF_np)
                UR_T_TCP_out = rm.Mat(UR_T_TCP_out_np.tolist())
                UR5.MoveL(UR_T_TCP_out, blocking=True)
                while (lever_theta2 >= 0):
                    MC_R_return_np = np.eye(4)
                    MC_R_return_np[ :3, :3] = getRy(np.deg2rad(lever_theta2))

                    # rotation BEFORE the offset, so the offset turns with the lever
                    UR_T_TCP_return_np = UR_T_M_np @ M_T_MC_np @ MC_R_return_np @ MC_t_p1_np @ t_out_np @ MC_T_MDL_np @ matrixInvert(MDB_T_MDL_np) @ matrixInvert(MTF_T_MDBE_np) @ matrixInvert(TCP_T_MTF_np)
                    UR_T_TCP_return = rm.Mat(UR_T_TCP_return_np.tolist())
                    UR5.MoveL(UR_T_TCP_return, blocking=True)
                    lever_theta2 -= step_size
                lever_theta = 0
            print(scale_value)
      
        # Return: same arc, offset behind the lever, counting back down to 0
        while (lever_theta2 >= 0):
            MC_R_return_np = np.eye(4)
            MC_R_return_np[ :3, :3] = getRy(np.deg2rad(lever_theta2))

            # rotation BEFORE the offset, so the offset turns with the lever
            UR_T_TCP_return_np = UR_T_M_np @ M_T_MC_np @ MC_R_return_np @ MC_t_p1_np @ t_out_np @ MC_T_MDL_np @ matrixInvert(MDB_T_MDL_np) @ matrixInvert(MTF_T_MDBE_np) @ matrixInvert(TCP_T_MTF_np)
            UR_T_TCP_return = rm.Mat(UR_T_TCP_return_np.tolist())
            UR5.MoveL(UR_T_TCP_return, blocking=True)
            lever_theta2 -= step_size
            lever_theta = 0


    #-----------------------------------------------------------------
    #                       Execute
    #----------------------------------------------------------------

    #Intermediate points:
    # i1 = [-28.740274, -85.658787, -129.601420, -149.868322, -56.739863, -87.930145]
    # i2 = [-36.903215, -110.476703, -102.504635, -151.593301, -64.939429, -125.890097]

    i1 = [118.590000, -104.650000, 78.030000, -3.380000, 90.000000, -206.410000]
    i2 = [108.510000, -68.600000, 102.450000, -34.160000, 81.360000, -128.990000]
    i3 =[114.391307, -67.177955, 101.499483, -34.525558, 80.445017, -129.028936]
    # Test points 
    UR_T_MC_np = UR_T_M_np @ M_T_MC_np

    #list
    UR_T_M = rm.Mat(UR_T_M_np.tolist())
    UR_T_TCP = rm.Mat(UR_T_TCP_np.tolist())
    UR_T_1_TCP = rm.Mat(UR_T_TCP_1_np.tolist())
    UR_T_MC = rm.Mat(UR_T_MC_np.tolist())

  
    UR_T_TCP_np = UR_T_M_np @ MC_T_MDL_np @ MC_T_MDL_np @ matrixInvert(MDB_T_MDL_np) @ matrixInvert(MTF_T_MDBE_np) @ matrixInvert(TCP_T_MTF_np)

    #tls.mazzer_tool_attach_l_ati()
    UR5.MoveL([87.542921, -86.978575, 138.426407, -106.972424, -11.087550, -309.589399], blocking=True)
    UR5.MoveJ([108.188836, -101.687537, 139.517213, -38.143704, 79.951438, -128.999970], blocking=True)

    UR5.MoveJ(i2, blocking=True)
    UR5.MoveL(i3, blocking=True)

    UR5.MoveL(UR_T_1_TCP, blocking=True)

    # leverPullsTest(1, 65, 2)

    leverRotation(66, 2)

    UR5.MoveL(i3, blocking=True)
    UR5.MoveL(i2, blocking=True)


    # UR5.MoveJ(UR_T_MC, blocking=True)
    # tls.mazzer_tool_detach_r_ati()
    # UR5.MoveC()

def mazzerDosingLever2():

    UR_Mazzer_orig = np.array([500.1, -423.7, 316.9])
    UR_Mazzer_point = np.array([554.5, -394.6, 316.9])

    local_Mazzer_orig = np.array([0, 0, 0])
    local_Mazzer_point = np.array([-61.73, 0, 0])

    UR_lidApex = np.array([555.5, -527.2, 581.5])
    local_lidApex = np.array([0, 290.83, 0])

    diff_UR_x = UR_Mazzer_point - UR_Mazzer_orig # local x direction
    diff_UR_y = UR_lidApex - UR_Mazzer_orig # local y directio

    x_hat = diff_UR_x / -np.linalg.norm(diff_UR_x) # magnitudes
    y_hat = diff_UR_y / np.linalg.norm(diff_UR_y)

    z_hat = np.cross(x_hat, y_hat)
    z_hat = z_hat / np.linalg.norm(z_hat)
    y_hat = np.cross(z_hat, x_hat)  

    UR_R_M_np = np.column_stack([x_hat, y_hat, z_hat])

    # UR T Mazzer local frame (no rotation)
    UR_T_M_np = np.array([[1, 0, 0, 500.1],
                          [0, 1, 0, -423.7],
                          [0, 0, 1, 316.9],
                          [0, 0, 0, 1]])

    # Rotation 
    UR_T_M_np[ :3, :3] = UR_R_M_np   #with rotation
    # UR_T_M_np[ , :3]

    print(UR_T_M_np)

    # local T Mazzer Dose Lever
    M_t_MDL = np.array([84.1, -131.8, -84.2])
    M_T_MDL_np = getT(get1(), M_t_MDL)



    #robot side

    #Tool Connection Point T Mazza Tool Frame
    TCP_T_MTF_np = np.eye(4)
    TCP_T_MTF_np[ :3, :3] = getRz(np.deg2rad(-50))

    # Mazzer Tool Frame T mazzer Dose Bar Edge
    MTF_t_MDB_np = np.array([-50, 0, 67.06])
    MTF_T_MDB_np = getT(get1(), MTF_t_MDB_np)

    # Mazzer Dose Bar edge T Mazzer Dose Lever
    # MDB_T_MDL_np = np.eye(4)
    MDB_T_MDL_np = np.array([[1, 0, 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])


    # UR_T_MDL_np = UR_T_M_np @ M_T_MDL_np

    UR_T_TCP_np = UR_T_M_np @ M_T_MDL_np @ matrixInvert(MDB_T_MDL_np) @ matrixInvert(MTF_T_MDB_np) @ matrixInvert(TCP_T_MTF_np)


    # Execute

    UR_T_M = rm.Mat(UR_T_M_np.tolist())

    UR_T_TCP = rm.Mat(UR_T_TCP_np.tolist())


    tls.mazzer_tool_attach_r_ati()
    UR5.MoveJ(UR_T_M, blocking=True)
    UR5.MoveJ(UR_T_TCP, blocking=True)

def placeInPUQ():
    #-----------------------------------------------------------------
    #                    Equipment side
    #----------------------------------------------------------------

    #Given coordinate points

    UR_PUQ_orig = np.array([386, 81.4, 270.4])
    UR_PUQ_point = np.array([346.1, 121.3, 270.4])

    local_PUQ_orig = np.array([0, 0, 0])
    local_PUQ_point = np.array([0, -56.38, 0])

    local_PUQ_clamp = np.array([11.5, -0.9, -135.3])


    diff_UR = UR_PUQ_point - UR_PUQ_orig
    diff_local = local_PUQ_point - local_PUQ_orig
    diff_UR_norm = diff_UR / np.linalg.norm(diff_UR)
    diff_M_norm = diff_local / np.linalg.norm(diff_local)

    cross = diff_local[0]*diff_UR[1] - diff_local[1]*diff_UR[0]
    dot   = diff_local[0]*diff_UR[0] + diff_local[1]*diff_UR[1]
    
    theta = np.arctan2(cross, dot)

    UR_t_PUQ = UR_PUQ_orig

    UR_R_PUQ = getRz(theta)
    print(np.rad2deg(theta))
    UR_T_PUQ_np = getT(UR_R_PUQ, UR_t_PUQ)

    # puq frame T puq Clamp
    PUQ_t_PC = local_PUQ_clamp
    PUQ_T_PC_np = getT(get1(), PUQ_t_PC)

    #-----------------------------------------------------------------
    #                       Robot Side
    #----------------------------------------------------------------

    #Tool connect point T Rancillio tool frame
    TCP_T_RTF_np = np.eye(4)
    TCP_T_RTF_np[ :3, :3] = getRz(np.deg2rad(-50))

    # Rancillio tool frame T basket center
    RTF_t_BC = np.array([29.1, 0, 148.2])
    # RTF_t_BC_1 = np.array([18, 0, 0])
    RTF_T_BC_np = getT(get1(), (RTF_t_BC ))
    BC_R_PC = getRy(np.deg2rad(-90 + 3.1))
    RTF_T_BC_np[ :3, :3] = BC_R_PC 

    # basket center T PUQ clamp
    BC_T_PC_np = np.array( [[1, 0, 0, 4], # <--- adjust how far inserted
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    #Relative rotation

    #-----------------------------------------------------------------
    #                       Execute
    #----------------------------------------------------------------

    UR_T_TCP_np = UR_T_PUQ_np @ PUQ_T_PC_np @ matrixInvert(BC_T_PC_np) @ matrixInvert (RTF_T_BC_np) @ matrixInvert(TCP_T_RTF_np)

    #Intermediate points:

    # Test points 
    i1 = [106.034054, -72.407972, 123.719160, -46.575162, -49.931532, -223.281179]
    i2 = [105.230626, -90.335986, 146.969540, -49.920214, -32.870616, -225.925366]
    #list
    UR_T_PUQ_ = rm.Mat(UR_T_PUQ_np.tolist())
    UR_T_TCP = rm.Mat(UR_T_TCP_np.tolist())

    # tls.rancilio_tool_attach_l_ati()
    # UR5.MoveJ(UR_T_PUQ_, blocking=True)
    UR5.MoveJ(i1, blocking=True)
    UR5.MoveL(i2, blocking=True)
    UR5.MoveL(rm.UR_2_Pose(rm.Pose_2_UR(UR_T_TCP)), blocking=True) 
    time.sleep(8)
    UR5.MoveJ(i2, blocking=True)
    UR5.MoveL(i1, blocking=True)
    # UR5.MoveL(UR_T_TCP, blocking=True)

def placeInPUQ2():
    #-----------------------------------------------------------------
    #                    Equipment side
    #----------------------------------------------------------------

    #Given coordinate points

    UR_PUQ_orig = np.array([386, 81.4, 270.4])
    UR_PUQ_point = np.array([346.1, 121.3, 270.4])

    local_PUQ_orig = np.array([0, 0, 0])
    local_PUQ_point = np.array([0, -56.38, 0])

    local_PUQ_clamp = np.array([11.5, -0.9, -135.3])


    diff_UR = UR_PUQ_point - UR_PUQ_orig
    diff_local = local_PUQ_point - local_PUQ_orig
    diff_UR_norm = diff_UR / np.linalg.norm(diff_UR)
    diff_M_norm = diff_local / np.linalg.norm(diff_local)

    cross = diff_local[0]*diff_UR[1] - diff_local[1]*diff_UR[0]
    dot   = diff_local[0]*diff_UR[0] + diff_local[1]*diff_UR[1]
    
    theta = np.arctan2(cross, dot)

    UR_t_PUQ = UR_PUQ_orig

    UR_R_PUQ = getRz(theta)
    print(np.rad2deg(theta))
    UR_T_PUQ_np = getT(UR_R_PUQ, UR_t_PUQ)

    # puq frame T puq Clamp
    PUQ_t_PC = local_PUQ_clamp
    PUQ_T_PC_np = getT(get1(), PUQ_t_PC)

    #-----------------------------------------------------------------
    #                       Robot Side
    #----------------------------------------------------------------

    #Tool connect point T Rancillio tool frame
    TCP_T_RTF_np = np.eye(4)
    TCP_T_RTF_np[ :3, :3] = getRz(np.deg2rad(-50))

    # Rancillio tool frame T basket center
    RTF_t_BC = np.array([29.1, 0, 148.2])
    # RTF_t_BC_1 = np.array([18, 0, 0])
    RTF_R_BC = getRz(np.deg2rad(0))
    RTF_T_BC_np = getT(RTF_R_BC, RTF_t_BC)
    RTF_R_BC_prime = getRy(np.deg2rad(-90 + 3.1))
    RTF_R_BC = getRx(np.deg2rad(3.5)) @ RTF_R_BC_prime
    RTF_T_BC_np[ :3, :3] = RTF_R_BC 

    # basket center T PUQ clamp
    BC_T_PC_np = np.array( [[1, 0, 0, 4], # <--- adjust how far inserted
                            [0, 1, 0, 0],
                            [0, 0, 1, 3],
                            [0, 0, 0, 1]])
    # BC_T_PC_np[ :3, :3] = getRz(np.deg2rad(-9))
    #Relative rotation

    #-----------------------------------------------------------------
    #                       Execute
    #----------------------------------------------------------------

    UR_T_TCP_np = UR_T_PUQ_np @ PUQ_T_PC_np @ matrixInvert(BC_T_PC_np) @ matrixInvert (RTF_T_BC_np) @ matrixInvert(TCP_T_RTF_np)

    #Intermediate points:

    # Test points 
    i0 = [-53.923137, -77.883196, -80.629530, -111.485527, 90.000457, -177.102559]
    i1 = [-34.957539, -85.695191, -152.814356, -102.350560, -165.294649, -201.717598]
    i2 = [-23.453885, -78.829758, -156.841447, -113.324857, -154.158325, -210.342644]
    #list
    UR_T_PUQ_ = rm.Mat(UR_T_PUQ_np.tolist())
    UR_T_TCP = rm.Mat(UR_T_TCP_np.tolist())

    # tls.rancilio_tool_attach_l_ati()
    # UR5.MoveJ(UR_T_PUQ_, blocking=True)
    # UR5.MoveL(i0, blocking=True)
    # UR5.MoveL([-65.543346, -94.791511, -126.493735, -134.827674, 171.257443, -216.656283], blocking=True)
    # UR5.MoveJ([-42.020524, -71.170625, -139.669621, -133.410265, -162.007779, -205.094994], blocking=True)
    UR5.MoveL(i1, blocking=True)
    UR5.MoveL(i2, blocking=True)
    UR5.MoveL(UR_T_TCP, blocking=True) 
    time.sleep(8)
    UR5.MoveL(i2, blocking=True)
    UR5.MoveL(i1, blocking=True)

def rotate_about(point, pivot, angle):
    """Rotate point (x, y) about pivot (x, y) by angle (rad, CCW +). Z ignored."""
    c, s = np.cos(angle), np.sin(angle)
    dx, dy = point[0] - pivot[0], point[1] - pivot[1]
    return np.array([pivot[0] + c*dx - s*dy,
                     pivot[1] + s*dx + c*dy])

def rancilio_intermediate():
    int1 = [-37.520000, -85.690000, -152.810000, -117.970000, -122.030000, 154.580000]
    int2 = [-23.099580, -77.848480, -146.429546, -133.836538, -73.321282, 154.641037]

    int3 = [-1.680000, -77.290000, -130.490000, -126.660000, -30.580000, 141.550000]
    int4 = [7.100000, -107.800000, 81.360000, -122.640000, -31.970000, 141.360000]
    int5 = [31.440000, -105.760000, 140.340000, -34.580000, -2.030000, 0]
    int6 = [118.650000, -105.760000, 140.340000, -32.540000, 89.490000, -223.730000]
    
    int7 = [89.162762, -113.552298, 153.376646, -43.415407, 106.870718, -218.766843]
    int8 = [14.576619, -139.770956, 141.114771, -10.537721, 31.843351, -210.101091]

    UR1 = np.array([439.4, -277.9, 41.9]) #Mazzer scale global
    UR2 = np.array([406.4, -296.9, 41.6]) # Second mazzer scale global
    MS1 = np.array([0,	0,	0]) #Mazzer scale local
    MS2 = np.array([0, -38,	0]) # Mazzer scale local

    diff_UR = UR2 - UR1
    diff_MS = MS2 - MS1

    cross = diff_MS[0]*diff_UR[1] - diff_MS[1]*diff_UR[0]
    dot   = diff_MS[0]*diff_UR[0] + diff_MS[1]*diff_UR[1]
    angle_MS = np.arctan2(cross, dot)

    MS_point = np.array([439.4, -277.9, 41.9])
    MS_ballbearing_hover_point = np.array([-31, -19, 14.5]) #Hover behind
    MS_ballbearing_lift_point = np.array([-12.1, -19, 25]) #lift
    MS_ballbearing_liftH_point = np.array([-26, -19, 20]) #hover and lift
    MS_ballbearing_point = np.array([-12.1, -19, 10.5]) #Point IRL from excel sheet

    RT_point = np.array([-32, 0, 28.07])
    TCP_point = np.array([0, 0, 0])

    UR_T_MS_np = getT(getRz(angle_MS), MS_point)
    MS_T_BB_np = getT(get1(), MS_ballbearing_point)
    Hover_T_BB_np = getT(get1(), MS_ballbearing_hover_point)
    Lift_T_BB_np = getT(get1(), MS_ballbearing_lift_point)
    HL_T_BB_np = getT(get1(), MS_ballbearing_liftH_point)
    RT_T_BB_np = getT(getRx(np.pi) @ getRy(-np.pi/2), RT_point)
    TCP_T_RT_np = getT(getRz(np.deg2rad(-50)), TCP_point)

    Lift_np = UR_T_MS_np @ Lift_T_BB_np @ matrixInvert(RT_T_BB_np) @ matrixInvert(TCP_T_RT_np)
    hover_np = UR_T_MS_np @ Hover_T_BB_np @ matrixInvert(RT_T_BB_np) @ matrixInvert(TCP_T_RT_np)
    HL_np = UR_T_MS_np @ HL_T_BB_np @ matrixInvert(RT_T_BB_np) @ matrixInvert(TCP_T_RT_np)
    Final_np = UR_T_MS_np @ MS_T_BB_np @ matrixInvert(RT_T_BB_np) @ matrixInvert(TCP_T_RT_np)

    Hover = rm.Mat(hover_np.tolist())
    Lift = rm.Mat(Lift_np.tolist())
    HL = rm.Mat(HL_np.tolist())
    UR_T_RT = rm.Mat(Final_np.tolist())

    tool = RDK.Item('Show_Mazzer_Scale_Rancilio_Tool', ITEM_TYPE_PROGRAM)
    tool.RunProgram()
    hidetool = RDK.Item('Hide_Mazzer_Scale_Rancilio_Tool', ITEM_TYPE_PROGRAM)
    showtool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)

    UR5.MoveJ(int1, blocking=True)
    UR5.MoveJ(int2, blocking=True)
    UR5.MoveL(HL, blocking=True)
    UR5.MoveL(Lift, blocking=True)
    UR5.MoveL(UR_T_RT, blocking=True)
    tls.student_tool_detach()
    UR5.MoveL(Hover)
    UR5.MoveJ(int3)
    UR5.MoveJ(int4)
    UR5.MoveJ(int5)
    UR5.MoveJ(int6)
    UR5.MoveJ(Hover)
    UR5.MoveL(UR_T_RT, blocking=True)
    tls.student_tool_attach()
    UR5.MoveL(Lift, blocking=True)
    UR5.MoveL(HL, blocking=True)
    UR5.MoveJ(int7, blocking=True)
    sleep(2)
    print('here')
    UR5.MoveJ(int8, blocking=True)


def ranicillioInsert():
    BASKET_ANGLE = 3.4
    INSERT_ANGLE = -45
    MAX_ROTATION = 30

    Rancilio_point1 = np.array([0,   0,      0])
    Rancilio_point2 = np.array([0,	220.42,	0])
    World_point1 = np.array([-414.1, -478.5,	350.3])
    World_point2 = np.array([-602.6, -364.9,	350.3])
    diff_UR = World_point2 - World_point1
    diff_MS = Rancilio_point2 - Rancilio_point1
    cross = diff_MS[0]*diff_UR[1] - diff_MS[1]*diff_UR[0]
    dot   = diff_MS[0]*diff_UR[0] + diff_MS[1]*diff_UR[1]
    angle_CM = np.arctan2(cross, dot)

    CM_point = [-414.1, -478.5,	350.3]

    UR_T_CM_np = getT(getRz(angle_CM), CM_point)

    gasket_centre = [-13.4, 68.9, -147.3]

    #45 deg turn
    CM_T_GC_np = getT(getRz(np.deg2rad(INSERT_ANGLE)), gasket_centre)

    #-----------------------------------------------------------------
    #                       Robot Side
    #----------------------------------------------------------------

    #Tool connect point T Rancillio tool frame
    TCP_T_RTF_np = np.eye(4)
    TCP_T_RTF_np[ :3, :3] = getRz(np.deg2rad(-50))

    # Rancillio tool frame T basket center
    RTF_t_BC = np.array([29.1, 0, 148.2])
    RTF_T_BC_np = getT(getRy(np.deg2rad(BASKET_ANGLE)), RTF_t_BC)

    R_lol = np.eye(4)
    R_lol[ :3, :3] = getRy(np.deg2rad(2.1))
    BC_T_GC_np = np.array( [[1, 0, 0, 0], 
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    BC_T_GC_np[ :3, :3] =   getRy(np.deg2rad(-90))

    #-----------------------------------------------------------------
    #                       Positions
    #----------------------------------------------------------------
    

    # Right under
    CM_t_under = getT(get1(), np.array([0, 0, -20]))
    UR_T_TCP_under_np = UR_T_CM_np  @ CM_T_GC_np @ CM_t_under  @ matrixInvert(BC_T_GC_np)  @ matrixInvert(RTF_T_BC_np) @ matrixInvert(TCP_T_RTF_np)

    #Insert
    UR_T_TCP_np = UR_T_CM_np @ CM_T_GC_np   @ matrixInvert(BC_T_GC_np) @ matrixInvert(RTF_T_BC_np) @ matrixInvert(TCP_T_RTF_np)

    UR_T_TCP = rm.Mat(UR_T_TCP_np.tolist())
    UR_T_TCP_under = rm.Mat(UR_T_TCP_under_np.tolist())

    #JAXON EDITS
    int1 = [63.890000, -124.070000, 105.760000, -16.270000, -26.440000, -231.070000]
    int2 = [45.630000, -124.070000, 138.310000, -36.610000, 34.580000, -231.070000]
    int3 = [29.410000, -99.660000, 140.340000, -30.510000, 95.590000, -231.070000]
    
    gasket_centre = [-13.4, 68.9, -147.3]
    gasket_centre_i = [-13.4, 68.9, -147.3]
    gasket_disconnect = [10, 68.9, -147.3]
    CM_T_GC_np = getT(get1(), gasket_centre)

    #insert INt
    CM_t_under_i = getT(get1(), np.array([110, 0, -20]))
    UR_T_TCPi_np = UR_T_CM_np @ CM_T_GC_np @ CM_t_under_i @ matrixInvert(BC_T_GC_np) @ matrixInvert(RTF_T_BC_np) @ matrixInvert(TCP_T_RTF_np)

    CM_T_GC3_np = getT(getRz(np.deg2rad(INSERT_ANGLE+MAX_ROTATION)), gasket_disconnect)
    UR_T_TCP3_np = UR_T_CM_np @ CM_T_GC3_np   @ matrixInvert(BC_T_GC_np) @ matrixInvert(RTF_T_BC_np) @ matrixInvert(TCP_T_RTF_np)
    UR_T_TCP3 = rm.Mat(UR_T_TCP3_np.tolist())
    UR_T_int = rm.Mat(UR_T_TCPi_np.tolist())
    
    # tls.rancilio_tool_attach_l_ati()
    # UR5.MoveJ(int1, blocking=True)
    # UR5.MoveJ(int2, blocking=True)
    UR5.MoveJ(UR_T_int, blocking=True)

    UR5.MoveL(rm.UR_2_Pose(rm.Pose_2_UR(UR_T_TCP_under)), blocking=True) 
    UR5.MoveL(rm.UR_2_Pose(rm.Pose_2_UR(UR_T_TCP)), blocking=True) 
    i=1
    while 1:
        CM_T_GC1_np = getT(getRz(np.deg2rad(INSERT_ANGLE+i)), gasket_centre)
        UR_T_TCP2_np = UR_T_CM_np @ CM_T_GC1_np   @ matrixInvert(BC_T_GC_np) @ matrixInvert(RTF_T_BC_np) @ matrixInvert(TCP_T_RTF_np)
        UR_T_TCP2 = rm.Mat(UR_T_TCP2_np.tolist())
        UR5.MoveJ(rm.UR_2_Pose(rm.Pose_2_UR(UR_T_TCP2)))
        print(i) 
        if i>=MAX_ROTATION:
            break
        i+=1
    tls.student_tool_detach
    UR5.MoveL(UR_T_TCP3, blocking=True)

def rancilioRemove():
    BASKET_ANGLE = 3.4
    INSERT_ANGLE = -45
    MAX_ROTATION = 30

    Rancilio_point1 = np.array([0,   0,      0])
    Rancilio_point2 = np.array([0,	220.42,	0])
    World_point1 = np.array([-414.1, -478.5,	350.3])
    World_point2 = np.array([-602.6, -364.9,	350.3])
    diff_UR = World_point2 - World_point1
    diff_MS = Rancilio_point2 - Rancilio_point1
    cross = diff_MS[0]*diff_UR[1] - diff_MS[1]*diff_UR[0]
    dot   = diff_MS[0]*diff_UR[0] + diff_MS[1]*diff_UR[1]
    angle_CM = np.arctan2(cross, dot)

    CM_point = [-414.1, -478.5,	350.3]

    UR_T_CM_np = getT(getRz(angle_CM), CM_point)

    gasket_centre = [-13.4, 68.9, -147.3]

    #45 deg turn
    CM_T_GC_np = getT(getRz(np.deg2rad(INSERT_ANGLE)), gasket_centre)

    #-----------------------------------------------------------------
    #                       Robot Side
    #----------------------------------------------------------------

    #Tool connect point T Rancillio tool frame
    TCP_T_RTF_np = np.eye(4)
    TCP_T_RTF_np[ :3, :3] = getRz(np.deg2rad(-50))

    # Rancillio tool frame T basket center
    RTF_t_BC = np.array([29.1, 0, 148.2])
    RTF_T_BC_np = getT(getRy(np.deg2rad(BASKET_ANGLE)), RTF_t_BC)

    R_lol = np.eye(4)
    R_lol[ :3, :3] = getRy(np.deg2rad(2.1))
    BC_T_GC_np = np.array( [[1, 0, 0, 0], 
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    BC_T_GC_np[ :3, :3] =   getRy(np.deg2rad(-90))

    #-----------------------------------------------------------------
    #                       Positions
    #----------------------------------------------------------------
    

    # Right under
    CM_t_under = getT(get1(), np.array([0, 0, -20]))
    UR_T_TCP_under_np = UR_T_CM_np  @ CM_T_GC_np @ CM_t_under  @ matrixInvert(BC_T_GC_np)  @ matrixInvert(RTF_T_BC_np) @ matrixInvert(TCP_T_RTF_np)

    #Insert
    UR_T_TCP_np = UR_T_CM_np @ CM_T_GC_np   @ matrixInvert(BC_T_GC_np) @ matrixInvert(RTF_T_BC_np) @ matrixInvert(TCP_T_RTF_np)

    UR_T_TCP = rm.Mat(UR_T_TCP_np.tolist())
    UR_T_TCP_under = rm.Mat(UR_T_TCP_under_np.tolist())

    #JAXON EDITS
    int1 = [63.890000, -124.070000, 105.760000, -16.270000, -26.440000, -231.070000]
    int2 = [45.630000, -124.070000, 138.310000, -36.610000, 34.580000, -231.070000]
    int3 = [29.410000, -99.660000, 140.340000, -30.510000, 95.590000, -231.070000]
    int4 = [72.000000, -93.560000, 122.030000, -45.170000, 105.760000, -58.980000]
    
    gasket_centre = [-13.4, 68.9, -147.3]
    gasket_disconnect = [10, 68.9, -147.3]
    CM_T_GC_np = getT(get1(), gasket_centre)

    CM_T_GC3_np = getT(getRz(np.deg2rad(INSERT_ANGLE+MAX_ROTATION)), gasket_disconnect)
    UR_T_TCP3_np = UR_T_CM_np @ CM_T_GC3_np   @ matrixInvert(BC_T_GC_np) @ matrixInvert(RTF_T_BC_np) @ matrixInvert(TCP_T_RTF_np)
    UR_T_TCP3 = rm.Mat(UR_T_TCP3_np.tolist())

    CM_T_GC4_np = getT(getRz(np.deg2rad(INSERT_ANGLE+MAX_ROTATION)), gasket_centre)
    UR_T_TCP4_np = UR_T_CM_np @ CM_T_GC4_np   @ matrixInvert(BC_T_GC_np) @ matrixInvert(RTF_T_BC_np) @ matrixInvert(TCP_T_RTF_np)
    UR_T_TCP4 = rm.Mat(UR_T_TCP4_np.tolist())
    
    # tls.rancilio_tool_attach_l_ati()
    # UR5.MoveJ(int1, blocking=True)
    # UR5.MoveJ(int2, blocking=True)
    UR5.MoveJ(int3, blocking=True)
    UR5.MoveL(UR_T_TCP3, blocking=True)
    UR5.MoveL(UR_T_TCP4, blocking=True)
    tls.student_tool_attach()
    i=1
    while 1:
        CM_T_GC1_np = getT(getRz(np.deg2rad(INSERT_ANGLE+MAX_ROTATION-i)), gasket_centre)
        UR_T_TCP2_np = UR_T_CM_np @ CM_T_GC1_np   @ matrixInvert(BC_T_GC_np) @ matrixInvert(RTF_T_BC_np) @ matrixInvert(TCP_T_RTF_np)
        UR_T_TCP2 = rm.Mat(UR_T_TCP2_np.tolist())
        UR5.MoveJ(rm.UR_2_Pose(rm.Pose_2_UR(UR_T_TCP2)))
        print(i) 
        if i>=MAX_ROTATION:
            break
        i+=1
    UR5.MoveL(rm.UR_2_Pose(rm.Pose_2_UR(UR_T_TCP_under)), blocking=True) 

    #new 
    UR5.MoveL([48.174389, -70.747180, 119.761035, -53.124894, 124.181124, -222.312342], blocking=True)
    UR5.MoveJ([34.230843, -81.266377, 134.619937, -57.275712, 64.845536, -217.973187], blocking=True)
    UR5.MoveJ([34.149454, -97.084026, 123.214109, -30.054854, 64.764338, -217.967032], blocking=True)
    UR5.MoveJ([39.933917, -89.988179, 116.733460, -32.734499, 39.268514, -214.868343], blocking=True)
    UR5.MoveJ([46.348099, -94.027278, 120.502942, -31.774761, 45.651776, -215.802326], blocking=True)
    UR5.MoveJ([46.554226, -96.133216, 107.799234, -6.086628, 45.993749, -46.173035], blocking=True)
    UR5.MoveJ([40.008010, -91.748067, 103.835623, -5.758887, 39.402943, -46.007463], blocking=True)
    # UR5.MoveJ(int4)


def cupSetup(): # (m & n - Jaxon)
    """Combines the 2 funcs into one seamless func"""
    getCup()
    placeCupRancilio()

def pullCoffee(): # (o, p & q - Jaxon)
    """Combines the 3 funcs into one seamless func"""
    unlockRancilioScale()
    pullHotWater()
    lockRancilioScale()

def grinderOn2():
    

    J_1 = [98.370000, -75.250000, 128.140000, -184.010000, -11.500000, -211.550000]
    #J_1 = [-59.830000, -113.900000, -124.070000, -99.660000, -172.880000, -164.750000]
    J_2 = [-53.750000, -105.760000, -124.070000, 7.040000, 118.440000, -183.950000]
    

    UR_T_M_np = np.array([[ -0.8818,    0.1913,    -0.4312,    500.1 ],
                        [ -0.4717,   -0.3576,     0.8060,   -423.7 ],
                        [ 0.0000,     0.9141,     0.4055,    316.9 ],
                        [ 0.000000, 0.000000, 0.000000, 1.000000 ]])

    R_I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    R_face     = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
    #R_approach = R_face @ getRx(np.radians(20)) @ getRy(np.radians(35))
    R_approach = R_face @ getRy(np.radians(25))


    M_T_MBOi_np = getT(R_approach, [90.9, -188.3, -144.2])
    M_T_MBO_np = getT(R_approach, [82.699, -182.63, -149.649])
      
    M_T_MBCi_np = getT(R_approach, [93.7, -182.6, -159.2-9]) 
    M_T_MBC_np = getT(R_approach, [86.617, -175.86, -164.812-9]) 

    MT_T_CGO_np = np.array([[ 1,     0,     0,      0],
                                [ 0,     1,     0,  0],
                                [ 0,     0,     1,  102.82],
                                [ 0,     0,     0,     1]])

    UR_T_MBOi_np = UR_T_M_np @ M_T_MBOi_np @ matrixInvert(MT_T_CGO_np)
    UR_T_MBCi_np = UR_T_M_np @ M_T_MBCi_np @ matrixInvert(MT_T_CGO_np)
    UR_T_MBO_np = UR_T_M_np @ M_T_MBO_np @ matrixInvert(MT_T_CGO_np)
    UR_T_MBC_np = UR_T_M_np @ M_T_MBC_np @ matrixInvert(MT_T_CGO_np)
    

    UR_T_MBOi = rm.Mat(UR_T_MBOi_np.tolist())
    UR_T_MBCi = rm.Mat(UR_T_MBCi_np.tolist())
    UR_T_MBO = rm.Mat(UR_T_MBO_np.tolist())
    UR_T_MBC = rm.Mat(UR_T_MBC_np.tolist())

    
    tls.mazzer_tool_attach_r_ati()
    sleep(1)
    UR5.MoveJ(J_1, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_MBOi, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_MBO, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_MBOi, blocking=True)
    sleep(3)
    UR5.MoveJ(UR_T_MBCi, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_MBC, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_MBCi, blocking=True)
    sleep(1)
    UR5.MoveJ(J_1, blocking=True)
    sleep(1)
    tls.mazzer_tool_detach_r_ati()

def task_i():
    UR_T_WDT_np = get_WDT_frame()

    WDT_rotor_t = np.array([28.4, 0.5, 83.2])
    WDT_rotor_i_t = np.array([28.4, 0.5, 83.2+20])

    poke_tool_np = np.array([[ 1,     0,     0,      0],
                            [ 0,     1,     0,  0],
                            [ 0,     0,     1,  102.82],
                            [ 0,     0,     0,     1]])

    R_rotor = np.eye(3) @ getRx(np.radians(180)) @ getRz(np.radians(180))
    WDT_T_Rotor_np = getT(R_rotor, WDT_rotor_t)
    WDT_T_Rotor_i_np = getT(R_rotor, WDT_rotor_i_t)

    rotor_centre = np.array([0, 0 , 83.2])

    rotor_20 = calc_rotation(rotor_centre, WDT_rotor_t, 20)
    rotor_90  = calc_rotation(rotor_centre, WDT_rotor_t, 90)
    rotor_180 = calc_rotation(rotor_centre, WDT_rotor_t, 180)
    rotor_270 = calc_rotation(rotor_centre, WDT_rotor_t, 270)

    WDT_T_20_np = getT(R_rotor, rotor_20)
    WDT_T_90_np = getT(R_rotor, rotor_90)
    WDT_T_180_np = getT(R_rotor, rotor_180)
    WDT_T_270_np = getT(R_rotor, rotor_270)

    UR_T_Rotor_np = UR_T_WDT_np @ WDT_T_Rotor_np @ matrixInvert(poke_tool_np)
    UR_T_Rotor_i_np = UR_T_WDT_np @ WDT_T_Rotor_i_np @ matrixInvert(poke_tool_np)
    UR_T_20_np = UR_T_WDT_np @ WDT_T_20_np @ matrixInvert(poke_tool_np)
    UR_T_90_np = UR_T_WDT_np @ WDT_T_90_np @ matrixInvert(poke_tool_np)
    UR_T_180_np = UR_T_WDT_np @ WDT_T_180_np @ matrixInvert(poke_tool_np)
    UR_T_270_np = UR_T_WDT_np @ WDT_T_270_np @ matrixInvert(poke_tool_np)

    UR_T_WDT = rm.Mat(UR_T_WDT_np.tolist())
    UR_T_Rotor = rm.Mat(UR_T_Rotor_np.tolist())
    UR_T_Rotor_i = rm.Mat(UR_T_Rotor_i_np.tolist())
    UR_T_20 = rm.Mat(UR_T_20_np.tolist())
    UR_T_90 = rm.Mat(UR_T_90_np.tolist())
    UR_T_180 = rm.Mat(UR_T_180_np.tolist())
    UR_T_270 = rm.Mat(UR_T_270_np.tolist())


    tls.mazzer_tool_attach_r_ati()
    sleep(1)
    UR5.MoveJ(UR_T_Rotor_i, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_Rotor, blocking=True)
    sleep(1)
    for _ in range(5):
        UR5.MoveC(UR_T_90,  UR_T_180, blocking=True)
        UR5.MoveC(UR_T_270, UR_T_20, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_Rotor_i, blocking=True)
    sleep(1)
    #tls.mazzer_tool_detach_r_ati()

def task_g():

    J1 = [7.510656, -85.891467, -155.422341, -117.276602, -79.101731, 135.464688]
    J2 = [7.201365, -71.304973, -153.172026, -134.162615, -79.653188, 135.783014]
    int3 = [18.418590, -94.997167, -141.203403, -127.989607, -10.465157, 144.267303]

    UR_T_WDT_np = get_WDT_frame()

    portafilter_t = np.array([29.1, 0, 148.2])
    portafilter_depth = np.array([29.3, 0, 0])
    portafilter_tilt = 3.3
    portafilter_aligned_t = portafilter_t - getRy(np.radians(portafilter_tilt)) @ getRx(np.radians(2)) @ ( portafilter_depth)

    portafilter_np = getT(np.eye(3) @ getRy(np.radians(portafilter_tilt)), portafilter_aligned_t)

    WDT_local_t =  np.array([0+4.5, 0+1.5, 0+1.5])
    R_aligned = np.eye(3) @ getRy(np.radians(90)) @ getRz(np.radians(50))
    WDT_T_portafilter_np = getT(R_aligned, WDT_local_t)

    WDT_P_t = np.array([108, 0, -42.3])
    WDT_T_P_np = getT(R_aligned, WDT_P_t)

    WDT_i2_t_np = np.array([0+4.5, 0+1.5, 45])
    WDT_T_i2_np = getT(R_aligned, WDT_i2_t_np)

    WDT_i1_t_np = np.array([150, 0, 45])
    WDT_T_i1_np = getT(R_aligned, WDT_i1_t_np)

    UR_T_WDT_portafilter_np = UR_T_WDT_np @ WDT_T_portafilter_np @ matrixInvert(portafilter_np)
    UR_T_P_np = UR_T_WDT_np @ WDT_T_P_np @ matrixInvert(portafilter_np)
    UR_T_i2_np = UR_T_WDT_np @ WDT_T_i2_np @ matrixInvert(portafilter_np)
    UR_T_i1_np = UR_T_WDT_np @ WDT_T_i1_np @ matrixInvert(portafilter_np)

    UR_T_WDT_portafilter = rm.Mat(UR_T_WDT_portafilter_np.tolist())
    UR_T_P = rm.Mat(UR_T_P_np.tolist())
    UR_T_i2 = rm.Mat(UR_T_i2_np.tolist())
    UR_T_i1 = rm.Mat(UR_T_i1_np.tolist())
    

    # tls.rancilio_tool_attach_r_ati()
    RDK.Item("Hide_WDT_Rancilio_Tool", ITEM_TYPE_PROGRAM).RunCode()
    RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL).setVisible(True, False)
    tls.wdt_open()
    UR5.MoveJ(int3, blocking=True)
    UR5.MoveJ(J2, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_i2, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_WDT_portafilter, blocking=True)
    sleep(1)
    tls.student_tool_detach()
    RDK.Item("Show_WDT_Rancilio_Tool", ITEM_TYPE_PROGRAM).RunCode()
    RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL).setVisible(False, False)
    sleep(1)
    UR5.MoveJ(UR_T_i1, blocking=True)
    sleep(1)
    tls.wdt_shut()
    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)

def task_j():

    J1 = [7.510656, -85.891467, -155.422341, -117.276602, -79.101731, 135.464688]

    UR_T_WDT_np = get_WDT_frame()

    portafilter_t = np.array([29.1, 0, 148.2])
    portafilter_depth = np.array([29.3, 0, 0])
    portafilter_tilt = 3.3
    portafilter_aligned_t = portafilter_t - getRy(np.radians(portafilter_tilt)) @ portafilter_depth
    portafilter_np = getT(np.eye(3) @ getRy(np.radians(portafilter_tilt)), portafilter_aligned_t)

    WDT_local_t =  np.array([0+3, 0+2, 0+1])
    R_aligned = np.eye(3) @ getRy(np.radians(90)) @getRz(np.radians(50))
    WDT_T_portafilter_np = getT(R_aligned, WDT_local_t)

    WDT_P_t = np.array([108, 0, -42.3])
    WDT_T_P_np = getT(R_aligned, WDT_P_t)

    WDT_i2_t_np = np.array([0+3, 0+2, 45])
    WDT_T_i2_np = getT(R_aligned, WDT_i2_t_np)

    WDT_i1_t_np = np.array([150, 0, 45])
    WDT_T_i1_np = getT(R_aligned, WDT_i1_t_np)

    UR_T_WDT_portafilter_np = UR_T_WDT_np @ WDT_T_portafilter_np @ matrixInvert(portafilter_np)
    UR_T_P_np = UR_T_WDT_np @ WDT_T_P_np @ matrixInvert(portafilter_np)
    UR_T_i2_np = UR_T_WDT_np @ WDT_T_i2_np @ matrixInvert(portafilter_np)
    UR_T_i1_np = UR_T_WDT_np @ WDT_T_i1_np @ matrixInvert(portafilter_np)

    UR_T_WDT_portafilter = rm.Mat(UR_T_WDT_portafilter_np.tolist())
    UR_T_P = rm.Mat(UR_T_P_np.tolist())
    UR_T_i2 = rm.Mat(UR_T_i2_np.tolist())
    UR_T_i1 = rm.Mat(UR_T_i1_np.tolist())


    tls.wdt_open()
    UR5.MoveJ(J1, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_WDT_portafilter, blocking=True)
    sleep(1)
    tls.student_tool_attach()
    RDK.Item("Hide_WDT_Rancilio_Tool", ITEM_TYPE_PROGRAM).RunCode()
    RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL).setVisible(True, False)
    sleep(1)
    UR5.MoveJ(UR_T_i2, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_i1, blocking=True)
    sleep(1)
    tls.wdt_shut()
    # tls.rancilio_tool_detach_r_ati()

def task_t():
    J1 = [-123.809247, -85.380338, -139.314560, 45.295451, 125.011710, -178.922254]
    SBi = [-108.678421, -100.996924, -122.391450, 43.686552, 109.919543, -179.057261]

    CP_t = np.array([-154.5, -541, 180.7])
    UR_T_CP_np = getT(np.eye(3), CP_t)

    PORTAFILTER_BASKET_DEPTH = 18
    portafilter_t = np.array([29.1, 0, 148.2])
    portafilter_depth = np.array([PORTAFILTER_BASKET_DEPTH, 0, 0])
    portafilter_tilt = 2.1
    portafilter_aligned_t = portafilter_t - getRy(np.radians(portafilter_tilt)) @ ( portafilter_depth)
    portafilter_np = getT(np.eye(3) @ getRy(np.radians(portafilter_tilt)), portafilter_aligned_t)

    spin_frame = getT(getRz(np.radians(-50)), [0, 0, 0])

    PC_SB_t = np.array([-45+5, -51.2-7, 8.2-11])
    R = np.eye(3) @ getRx(np.radians(90)) @ getRz(np.radians(-90))
    CP_T_SB = getT(R, PC_SB_t)

    PC_SBi_t = np.array([-45+5, -51.2-7, 8.2+30])
    CP_T_SBi = getT(R, PC_SBi_t)

    PC_BB_t = np.array([45+5, -51.2-7, 8.2-7])
    CP_T_BB = getT(R, PC_BB_t)

    PC_BBi_t = np.array([45+5, -51.2-7, 8.2+30])
    CP_T_BBi = getT(R, PC_BBi_t)


    UR_T_SB_np = UR_T_CP_np @ CP_T_SB @ matrixInvert(portafilter_np) @ matrixInvert(spin_frame)
    UR_T_SBi_np = UR_T_CP_np @ CP_T_SBi @ matrixInvert(portafilter_np) @ matrixInvert(spin_frame)
    UR_T_BB_np = UR_T_CP_np @ CP_T_BB @ matrixInvert(portafilter_np) @ matrixInvert(spin_frame)
    UR_T_BBi_np = UR_T_CP_np @ CP_T_BBi @ matrixInvert(portafilter_np) @ matrixInvert(spin_frame)


    UR_T_CP = rm.Mat(UR_T_CP_np.tolist())
    UR_T_SB = rm.Mat(UR_T_SB_np.tolist())
    UR_T_SBi = rm.Mat(UR_T_SBi_np.tolist())
    UR_T_BB = rm.Mat(UR_T_BB_np.tolist())
    UR_T_BBi = rm.Mat(UR_T_BBi_np.tolist())

    UR5.MoveJ(UR_T_SBi, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_SB, blocking=True)
    sleep(2)
    UR5.MoveJ(UR_T_SBi, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_BBi, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_BB, blocking=True)
    sleep(1)
    UR5.MoveJ(UR_T_BBi, blocking=True)
    tls.rancilio_tool_detach_r_ati()

def main():
    resetSim()

    weighRancilioTool()
    cupSetup()

    unlockMazzerScale()
    grinderOn()
    mazzerDosingLever()
    lockMazzerScale()

    removeRancilioToolMazzer()
    placeInPUQ2()
    ranicillioInsert()

    # task_g()
    task_i()

    # task_j()

    #placeInPUQ2() # Sticking with 2
    # rancilio_intermediate()
    # ranicillioInsert()

    pullCoffee()

    rancilioRemove()
    task_t() #cleaning

    serveCoffee()

    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)
main()