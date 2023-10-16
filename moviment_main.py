import sim 
import sys
import time

sim.simxFinish(-1)

clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID != -1:
    print ("Conectado ao servidro API")

else: 
    print ("Não conectado")
    sys.exit("Não foi possivel conectar")

erro, robot = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_oneshot_wait)
errorCode, left_motor_handle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_oneshot_wait)

if errorCode != sim.simx_return_ok:
        print("Erro ao obter os identificadores dos motores:", errorCode)
        exit(1)


# Coletar dados robôs e alvos
[erro, [xr,yr,zr]] = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_buffer)
[erro, [alpha, beta, gamma]] = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)

sim.simxAddStatusbarMessage(clientID, 'A posição em x é '+str(xr) + ' e a posição em y é '+str(yr), sim.simx_opmode_oneshot_wait)

errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 1, sim.simx_opmode_oneshot_wait)
errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 1.3, sim.simx_opmode_oneshot_wait)

if errorCode != sim.simx_return_ok:
    print("Erro ao definir a velocidade dos motores:", errorCode)
    exit(1)

# Pause simulation
time.sleep(10)
sim.simxPauseSimulation(clientID,sim.simx_opmode_oneshot_wait)