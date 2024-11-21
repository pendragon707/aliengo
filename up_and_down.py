#!/usr/bin/python

import sys
import time
import math

sys.path.append('../lib/python/amd64')
import robot_interface_aliengo as sdk

TARGET_PORT = 8007
LOCAL_PORT = 8082
TARGET_IP = "192.168.123.10"   # target IP address

LOW_CMD_LENGTH = 610
LOW_STATE_LENGTH = 771

def jointLinearInterpolation(initPos, targetPos, rate):

    #rate = np.fmin(np.fmax(rate, 0.0), 1.0)
    if rate > 1.0:
        rate = 1.0
    elif rate < 0.0:
        rate = 0.0

    p = initPos*(1-rate) + targetPos*rate
    return p


if __name__ == '__main__':

    d = {'FR_0':0, 'FR_1':1, 'FR_2':2,
         'FL_0':3, 'FL_1':4, 'FL_2':5, 
         'RR_0':6, 'RR_1':7, 'RR_2':8, 
         'RL_0':9, 'RL_1':10, 'RL_2':11 }
    PosStopF  = math.pow(10,9)
    VelStopF  = 16000.0
    HIGHLEVEL = 0x00
    LOWLEVEL  = 0xff
    start_q = [-0.15, 1.18, -2.8] * 4
    end_q = [0.0, 0.3, -1, 0.0, 1, -1]
    dt = 0.002
    qInit = [0] * 12
    qDes = [0] * 12
    sin_count = 0
    rate_count = 0
    Kp = [0] * 12
    Kd = [0] * 12

    udp = sdk.UDP(LOCAL_PORT, TARGET_IP, TARGET_PORT, LOW_CMD_LENGTH, LOW_STATE_LENGTH, -1)
    #udp = sdk.UDP(8082, "192.168.123.10", 8007, 610, 771)
    safe = sdk.Safety(sdk.LeggedType.Aliengo)
    
    cmd = sdk.LowCmd()
    state = sdk.LowState()
    udp.InitCmdData(cmd)
    cmd.levelFlag = LOWLEVEL

    count = 0
    count2 = 0
    Tpi = 0
    motiontime = 0
    while True:
        time.sleep(0.002)
        motiontime += 1
        
        udp.Recv()
        udp.GetRecv(state)
        
        if( motiontime >= 0):

            # first, get record initial position
            if( motiontime >= 0 and motiontime < 10):

                for num, value in enumerate(d.values()):
                    qInit[num] = state.motorState[value].q

                print("Front: ", qInit[0], qInit[1], qInit[2])
                print("Rear: ", qInit[6], qInit[7], qInit[8])
            # second, move to the origin point of a sine movement with Kp Kd
            if( motiontime >= 10 and motiontime < 400):
                rate_count += 1
                rate = rate_count/200.0                       # needs count to 200
                # Kp = [5, 5, 5]
                # Kd = [1, 1, 1]
                # Kp = [20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20]
                # Kd = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
                Kp = [90] * 12
                Kd = [1] * 12

                for num, value in enumerate(d.values()):
                    qDes[num] = jointLinearInterpolation(qInit[num], start_q[num], rate)
            
            # last, do sine wave
            freq_Hz = 1
            # freq_Hz = 5
            freq_rad = freq_Hz * 2* math.pi
            # t = dt*sin_count
            t = 0
            if( motiontime >= 400):
                sin_count += 1
                jointf = 0
                jointr = 0

                
                alpha = min(count /  1000, 1)
                alpha2 = min(count2 / 1000, 1)
                if motiontime <= 2500:
                    
                    for i in range( 0, len(d) - 6 , 3):
                        qDes[i] = start_q[0]
                        qDes[i + 1] =  alpha * end_q[1] + (1-alpha) * start_q[1]
                        qDes[i + 2] = alpha * end_q[2] + (1-alpha) * start_q[2]
                        qDes[i + 6] = start_q[0]
                        qDes[i + 7] =  alpha * end_q[4] + (1-alpha) * start_q[1]
                        qDes[i + 8] = alpha * end_q[5] + (1-alpha) * start_q[2]
                    count += 1
                else:
                    for i in range( 0, len(d) - 6 , 3):
                        qDes[i] = start_q[0]
                        qDes[i + 1] =  (1-alpha2) * end_q[1] + alpha2 * start_q[1]
                        qDes[i + 2] = (1-alpha2) * end_q[2] + alpha2 * start_q[2]
                        qDes[i + 6] = start_q[0]
                        qDes[i + 7] =  (1-alpha2) * end_q[4] + alpha2 * start_q[1]
                        qDes[i + 8] = (1-alpha2) * end_q[5] + alpha2 * start_q[2]
                    count2 += 1


            cmd.motorCmd[d['FR_0']].q = qDes[0]
            cmd.motorCmd[d['FR_0']].dq = 0
            cmd.motorCmd[d['FR_0']].Kp = Kp[0]
            cmd.motorCmd[d['FR_0']].Kd = Kd[0]
            cmd.motorCmd[d['FR_0']].tau = 0.0

            cmd.motorCmd[d['FR_1']].q = qDes[1]
            cmd.motorCmd[d['FR_1']].dq = 0
            cmd.motorCmd[d['FR_1']].Kp = Kp[1]
            cmd.motorCmd[d['FR_1']].Kd = Kd[1]
            cmd.motorCmd[d['FR_1']].tau = 0.0

            cmd.motorCmd[d['FR_2']].q =  qDes[2]
            cmd.motorCmd[d['FR_2']].dq = 0
            cmd.motorCmd[d['FR_2']].Kp = Kp[2]
            cmd.motorCmd[d['FR_2']].Kd = Kd[2]
            cmd.motorCmd[d['FR_2']].tau = 0.0

            

            cmd.motorCmd[d['FL_0']].q = qDes[3]
            cmd.motorCmd[d['FL_0']].dq = 0
            cmd.motorCmd[d['FL_0']].Kp = Kp[3]
            cmd.motorCmd[d['FL_0']].Kd = Kd[3]
            cmd.motorCmd[d['FL_0']].tau = 0.0

            cmd.motorCmd[d['FL_1']].q = qDes[4]
            cmd.motorCmd[d['FL_1']].dq = 0
            cmd.motorCmd[d['FL_1']].Kp = Kp[4]
            cmd.motorCmd[d['FL_1']].Kd = Kd[4]
            cmd.motorCmd[d['FL_1']].tau = 0.0

            cmd.motorCmd[d['FL_2']].q =  qDes[5]
            cmd.motorCmd[d['FL_2']].dq = 0
            cmd.motorCmd[d['FL_2']].Kp = Kp[5]
            cmd.motorCmd[d['FL_2']].Kd = Kd[5]
            cmd.motorCmd[d['FL_2']].tau = 0.0


            cmd.motorCmd[d['RR_0']].q = qDes[6]
            cmd.motorCmd[d['RR_0']].dq = 0
            cmd.motorCmd[d['RR_0']].Kp = Kp[6]
            cmd.motorCmd[d['RR_0']].Kd = Kd[6]
            cmd.motorCmd[d['RR_0']].tau = 0.0

            cmd.motorCmd[d['RR_1']].q = qDes[7]
            cmd.motorCmd[d['RR_1']].dq = 0
            cmd.motorCmd[d['RR_1']].Kp = Kp[7]
            cmd.motorCmd[d['RR_1']].Kd = Kd[7]
            cmd.motorCmd[d['RR_1']].tau = 0.0

            cmd.motorCmd[d['RR_2']].q =  qDes[8]
            cmd.motorCmd[d['RR_2']].dq = 0
            cmd.motorCmd[d['RR_2']].Kp = Kp[8]
            cmd.motorCmd[d['RR_2']].Kd = Kd[8]
            cmd.motorCmd[d['RR_2']].tau = 0.0

            

            cmd.motorCmd[d['RL_0']].q = qDes[9]
            cmd.motorCmd[d['RL_0']].dq = 0
            cmd.motorCmd[d['RL_0']].Kp = Kp[9]
            cmd.motorCmd[d['RL_0']].Kd = Kd[9]
            cmd.motorCmd[d['RL_0']].tau = 0.0

            cmd.motorCmd[d['RL_1']].q = qDes[10]
            cmd.motorCmd[d['RL_1']].dq = 0
            cmd.motorCmd[d['RL_1']].Kp = Kp[10]
            cmd.motorCmd[d['RL_1']].Kd = Kd[10]
            cmd.motorCmd[d['RL_1']].tau = 0.0

            cmd.motorCmd[d['RL_2']].q =  qDes[11]
            cmd.motorCmd[d['RL_2']].dq = 0
            cmd.motorCmd[d['RL_2']].Kp = Kp[11]
            cmd.motorCmd[d['RL_2']].Kd = Kd[11]
            cmd.motorCmd[d['RL_2']].tau = 0.0
            # cmd.motorCmd[d['FR_2']].tau = 2 * sin(t*freq_rad)
            if motiontime % 10 == 0:
                # print(dir(state.motorState[d['FR_0']]))
                print("Torque :   ", end = "")
                print(int(state.motorState[d['FR_0']].tauEst * 100), end = "  ")
                print(int(state.motorState[d['FR_1']].tauEst * 100), end = "  ")
                print(int(state.motorState[d['FR_2']].tauEst * 100), end = "  ")

                print(int(state.motorState[d['FL_0']].tauEst * 100), end = "  ")
                print(int(state.motorState[d['FL_1']].tauEst * 100), end = "  ")
                print(int(state.motorState[d['FL_2']].tauEst * 100), end = "  ")

                print(int(state.motorState[d['RR_0']].tauEst * 100), end = "  ")
                print(int(state.motorState[d['RR_1']].tauEst * 100), end = "  ")
                print(int(state.motorState[d['RR_2']].tauEst * 100), end = "  ")

                print(int(state.motorState[d['RL_0']].tauEst * 100), end = "  ")
                print(int(state.motorState[d['RL_1']].tauEst * 100), end = "  ")
                print(int(state.motorState[d['RL_2']].tauEst * 100))      


        if(motiontime > 10):
             safe.PowerProtect(cmd, state, 1)




        udp.SetSend(cmd)
        udp.Send()
