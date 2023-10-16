import sim
import time
import sys 
import matplotlib.pyplot as plt
import numpy as np
import math as mt

#--------------------------------------------------------CONEXAO COM API--------------------------------------------------------#
print ("Start")
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)

if (clientID != -1):
    print('Conectado')
else:
    print('Não conectou')
    sys.exit(1)

time.sleep(1)

def Poli3 (qi, qf, step) :
    dx = qf[0] - qi[0]
    dy = qf[1] - qi[1]
    teta_init = (qi[2]*mt.pi) / 180
    teta_final = (qf[2]*mt.pi) / 180
    di = mt.tan(teta_init)
    df = mt.tan(teta_final)

    teta_func = []

    if (((teta_init > 89) and (teta_init < 91)) & ((teta_final > 89) & (teta_final < 91))):
        a0 = qi[0]
        a1 = 0
        a2 = 3 * dx
        a3 = (-2) * dx
        b0 = qi[1]
        b1 = dy
        b2 = 0
        b3 = dy - b1 - b2
    elif ((teta_init > 89) and (teta_init < 91)):
        a0 = qi[0]
        a1 = 0
        a3 = (-1) * dx / 2
        a2 = dx (-1) * a3
        b0 = qi[1]
        b2 = 1
        b1 = 2 * (dy - df * dx) - df * a3 + b2
        b3 = (2 * df * dx - dy) + df * a3 - 2 * b2   
    elif ((teta_final > 89) and (teta_final < 91)): 
        a0 = qi[0]
        a1 = 3*dx/2
        a2 = 3*dx - 2*a1
        a3 = a1 - 2*dx
        b0 = qi[1]
        b1 = di*a1
        b2 = 1
        b3 = dy - di*a1 - b2
    else:
        a0 = qi[0]
        a1 = dx
        a2 = 0
        a3 = dx - a1 - a2
        b0 = qi[1]
        b1 = di*a1
        b2 = 3*(dy - df*dx) + 2*(df - di)*a1 + df*a2
        b3 = 3*df*dx - 2*dy - (2*df - di)*a1 - df*a2

    a = np.array([a0, a1, a2, a3])
    b = np.array([b0, b1, b2, b3])
    lambda_poli = np.arange(0, 1+step, step)

    X = a0 + a1*lambda_poli + a2*pow(lambda_poli, 2) + a3*pow(lambda_poli, 3)
    Y = b0 + b1*lambda_poli + b2*pow(lambda_poli, 2) + b3*pow(lambda_poli, 3)

    for i in range (len(lambda_poli)):
        teta_func.append(mt.atan((b1 + b2*lambda_poli[i] + b3*pow(lambda_poli[i], 2))/(a1 + a2*lambda_poli[i] + b3*pow(lambda_poli[i],2)))*180/mt.pi)
        #teta_func.append(mt.atan((a1 + a2*lambda_poli[i] + b3*pow(lambda_poli[i],2))/(b1 + b2*lambda_poli[i] + b3*pow(lambda_poli[i], 2)))*180/mt.pi)

    X = np.append(X, qf[0])
    Y = np.append(Y, qf[1])
    teta_func.append(qf[2])

    #Retorno dos dados
    return X, Y, teta_func, a, b

def interpoGamma (qi, qf, step):

    teta_interp = np.linspace(qi, qf, step)
    return teta_interp

#Configuração inicial e final do robô
qi = np.array([0, 0, 0])
qf = np.array([2,3, 60])
step = 0.1

errorcode, frame = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_oneshot_wait)
errorcode = sim.simxSetObjectPosition(clientID, frame,-1, [qi[0],qi[1], 0], sim.simx_opmode_oneshot_wait)
errorcode = sim.simxGetObjectPosition(clientID, frame, -1, sim.simx_opmode_oneshot_wait)
errorcode = sim.simxGetObjectOrientation(clientID, frame, -1, sim.simx_opmode_oneshot_wait)

#returnCode, goalFrame = sim.simxGetObjectHandle(clientID, 'Goal', sim.simx_opmode_oneshot_wait)     
#returnCode = sim.simxSetObjectPosition(clientID, goalFrame, -1, [qf[0],qf[1], 0], sim.simx_opmode_oneshot_wait)
#returnCode = sim.simxSetObjectOrientation(clientID, goalFrame, -1, [0, 0, qf[2]], sim.simx_opmode_oneshot_wait)    

qgoal_x, qgoal_y, qgoal_teta, a, b = Poli3(qi, qf, step)
qgoal = [qgoal_x, qgoal_y, qgoal_teta]

tetainter = interpoGamma(qi[2], qf[2], len(qgoal_x))

for i in range (len(qgoal_x)):

    errorcode = sim.simxSetObjectPosition(clientID, frame, -1, [qgoal_x[i], qgoal_y[i], 0], sim.simx_opmode_oneshot_wait)
    errorcode = sim.simxSetObjectOrientation(clientID, frame, -1, [0, 0, tetainter[i]], sim.simx_opmode_oneshot_wait)

    time.sleep(1)

sim.simxPauseSimulation(clientID,sim.simx_opmode_oneshot_wait)

# Now close the connection to CoppeliaSim:
sim.simxFinish(clientID)


print('x = ' , qgoal_x)
print('y = ', qgoal_y)
print('gamma = ', tetainter)

plt.plot(qgoal_x,qgoal_y, linestyle='solid', color='black', marker='o',markerfacecolor='red')
plt.title("Posição do robô (x(t),y(t))")
plt.xlabel("Coordenada x (m)")
plt.ylabel("Coordenada y (m)")
plt.grid()
plt.show()