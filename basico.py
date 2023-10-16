# coding=utf-8
# Insert in a script in Coppelia
# simRemoteApi.start(19999)
try:
    import sim
    import matplotlib.pyplot as plt
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import math
import time

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)
robotname = 'PionnerP3DX'
targetname = 'PionnerP3DX'
if clientID != -1:
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    print ('Connected to remote API server')
    sim.simxAddStatusbarMessage(clientID,'Funcionando...',sim.simx_opmode_oneshot_wait)
    time.sleep(0.02)
    # Coletar handles
    erro, robot = sim.simxGetObjectHandle(clientID, robotname, sim.simx_opmode_oneshot_wait)
    [erro, target] = sim.simxGetObjectHandle(clientID, targetname, sim.simx_opmode_oneshot_wait)
    [erro, robotLeftMotor] = sim.simxGetObjectHandle(clientID, robotname +'/leftMotor',sim.simx_opmode_oneshot_wait)
    [erro, robotRightMotor] = sim.simxGetObjectHandle(clientID, robotname + '/rightMotor', sim.simx_opmode_oneshot_wait)

    # Criar stream de dados
    [erro, positionrobot] = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
    [erro, positiontarget] = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_streaming)
    [erro, orientationrobot] = sim.simxGetObjectOrientation(clientID,robot,-1,sim.simx_opmode_streaming)
    time.sleep(2)

    cont = 0
    while(cont < 100):
        cont+=1
        # Coletar dados robôs e alvos
        [erro, [xr,yr,zr]] = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_buffer)
        [erro, [xt,yt,zt]] = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_buffer)
        [erro, [alpha, beta, gamma]] = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)

        #vref = 10.0
        sim.simxAddStatusbarMessage(clientID, 'A posição em x é '+str(xr) + ' e a posição em y é '+str(yr), sim.simx_opmode_oneshot_wait)

        # Comandos motores
        sim.simxPauseCommunication(clientID, True)
        sim.simxSetJointTargetVelocity(clientID, robotRightMotor, 50, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, 50, sim.simx_opmode_oneshot)
        sim.simxPauseCommunication(clientID, False)

        [erro, positionrobot] = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
        [erro, positiontarget] = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_streaming)
        [erro, orientationrobot] = sim.simxGetObjectOrientation(clientID,robot,-1,sim.simx_opmode_streaming)

    # Stop simulation:
    # sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)

    # Pause simulation
    sim.simxPauseSimulation(clientID,sim.simx_opmode_oneshot_wait)

    # Now close the connection to V-REP:
    sim.simxAddStatusbarMessage(clientID, 'Programa pausado', sim.simx_opmode_blocking )
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')