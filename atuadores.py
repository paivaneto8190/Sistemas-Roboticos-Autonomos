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

if (clientID != 1):
    print('Conectado')
else:
    print('Não conectou')
    sys.exit(1)

time.sleep(1)

#--------------------------------------------------------FUNCOES DO PROGRAMA--------------------------------------------------------#
#Funcao geradora de caminhos com polinomio de 3 grau
def Poli3 (qi, qf, step) :
    dx = qf[0] - qi[0]
    dy = qf[1] - qi[1]
    teta_init = (qi[2]*mt.pi) / 180
    teta_final = (qf[2]*mt.pi) / 180
    di = mt.tan(teta_init)
    df = mt.tan(teta_final)

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
    lambda_poli = np.arange(0, 1, step)

    teta_func = []

    X = a0 + a1*lambda_poli + a2*pow(lambda_poli, 2) + a3*pow(lambda_poli, 3)
    Y = b0 + b1*lambda_poli + b2*pow(lambda_poli, 2) + b3*pow(lambda_poli, 3)

    for i in range (len(lambda_poli)):
        #teta_func.append(mt.atan((a1 + a2*lambda_poli[i] + b3*pow(lambda_poli[i],2))/(b1 + b2*lambda_poli[i] + b3*pow(lambda_poli[i], 2)))*180/mt.pi)
        teta_func.append(mt.atan((b1 + b2*lambda_poli[i] + b3*pow(lambda_poli[i], 2))/(a1 + a2*lambda_poli[i] + a3*pow(lambda_poli[i],2)))*180/mt.pi)

    #Retorno dos dados
    return X, Y, teta_func, a, b

#Matriz para resolver equacao cinemativa (ACHO QUE PODE TIRAR)
def iRr(teta):
    return np.array([[ mt.cos(teta), -mt.sin(teta), 0 ],
                      [ mt.sin(teta), mt.cos(teta) , 0 ],
                      [ 0            , 0             , 1 ]])

#Funcoes para o controlador de Samson
def getDistance(a,b) :
    x = a[0] - b[0]
    y = a[1] - b[1]

    res = np.sqrt(pow(x,2) + pow(y,2))

    return res

def samson(v, ks, delta_teta, dL) :
    kT = 0.3 #Constante angular
    kL = 0.1 #Constante L
    u = (-1)*(kT*delta_teta + (kL*dL*v*mt.sin(delta_teta))/delta_teta)
    w = u + (ks*v*mt.cos(delta_teta))/(1 - ks*dL)

    return w

def segueCaminho(q_ref, ks, deltaTeta_res, deltaL) :
    pos = distMin(q_ref)
    errorcode = sim.simxSetObjectPosition(clientID, robot,-1, [pos[0], pos[1]], sim.simx_opmode_oneshot_wait)
    v = 0.1
    print(deltaTeta(q_ref,pos))
    W = samson(v, ks, deltaTeta_res, deltaL)

    vd = v - (b/2 * W)
    ve = v + (b/2 * W)

    wd = vd/rd
    we = ve/re

    return wd, we


def distMin (ponto_ref, a, ref, camTam) :
    deltaL = 50000
    d = 0
    i = 0
    x = 0
    y = 0
    tamCaminho = camTam

    while i < tamCaminho :
        d = getDistance(a, ref)
        if (d < deltaL) :
            deltaL = d
            x = a[0]
            y = a[1]
        i += 1
    
    return x,y, deltaL

def pos_ref (ref_pos):
    errorcode, ref = sim.simxGetObjectPosition(clientID, robot,-1, sim.simx_opmode_oneshot_wait)
    deltaX = ref[0] - ref_pos[0]
    deltaY = ref[1] - ref_pos[1]

    phi = mt.atan2(deltaY, deltaX)

    delta_La = mt.sqrt(pow(deltaX,2) + pow(deltaY,2))
    delta_teta = phi - ref_pos[2]
    delta_L = delta_La * mt.cos(delta_teta)

    v = 0.05 
    w = 0.5 * delta_teta 

    vd = v - (b/2 * w)
    ve = v + (b/2 * w)

    wd = vd/rd 
    we = ve/re 

    if (delta_La < 0.1) :
        sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot_wait)
    
    return wd, we

def segueCaminho (q,v_q, q_ref,vq_ref):
    t = 0.05 #colocar como variavel global
    xa = q_ref[0]
    ya = q_ref[1]

    xa_vel = vq_ref[0]
    ya_vel = vq_ref[1]

    xa_acel = (vq_ref[0] - aux_v_ref[0]) / t
    ya_acel = (vq_ref[1] - aux_v_ref[1]) / t #declarar aux_v_ref como vetor nulo

    x = q[0]
    y = q[1]

    x_vel = v_q[0]
    y_vel = v_q[1]

    teta = q[2]

    v = mt.sqrt(pow(x_vel,2) + pow(y_vel,2))

    #onde Kpx, Kpy > 0 e Kdx, Kdy > 0 (LEMBRAR DE TIRAR ESSA COMENTARIO)
    Kpx = 0.05
    Kpy = Kpx

    Kdx = 1.0
    Kdy = Kdx

    #Realimentacao PD - Aceleracoes de Comando
    xc_acel = xa_acel + Kdx*(xa_vel - x_vel) + Kpx*(xa - x) #aceleracao em x
    yc_acel = ya_acel + Kdy*(ya_vel - y_vel) + Kpy*(ya - y) #aceleracao em y

    #Compensasao do Modelo Nao Linear
    vc_l = (mt.cos(teta)*xc_acel) + (mt.sin(teta)*yc_acel) #aceleracao de comando 
    wc = -((mt.sin(teta)/v)*xc_acel) + ((mt.cos(teta)/v)*yc_acel) #velocidade angular de comando

    vc = (t*vc_l) + v_aux

    v_aux = vc #colocar uma zerada como variavel global
 
    aux_v_ref = vq_ref


    vd = vc - (b/2 * wc)
    ve = vc + (b/2 * wc)

    wd = vd/rd 
    we = ve/re 

    return wd, we

def roboPose_atual ():
    errorcode, posicao = sim.simxGetObjectPosition(clientID, robot,-1, sim.simx_opmode_oneshot_wait)
    errorcode, [alpha, beta, gamma] = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)
    q = np.array([posicao[0] + 0.0445, posicao[1], gamma])
    return q

def Pose_atual(ref_):
    errorcode, refPosicao = sim.simxGetObjectPosition(clientID, ref_,-1, sim.simx_opmode_oneshot_wait)
    errorcode, [alpha, beta, gamma] = sim.simxGetObjectOrientation(clientID, ref_, -1, sim.simx_opmode_buffer)
    qPose_atual = np.array([refPosicao[0], refPosicao[1], gamma])
    return qPose_atual

def deltaTeta (pos, min_dist) :
    y_coef = (-1)*(3 * pow(min_dist[0],2)) + 1
    ang_coef = mt.sqrt(pow(y_coef,2) + 1)

    coef = np.array([1/ang_coef, y_coef/ang_coef])

    deltaP = np.array([pos[0] - min_dist[0], pos[1] - min_dist[1]])

    tetaP = mt.deg(mt.atan(coef[1]/coef[0]))
    tetaR = mt.deg(pos[2])
    deltaTeta_res = tetaR - tetaP

    return deltaTeta_res

def curve(X, Y):

    coordinates = np.array([X,Y])
    coordinates = coordinates.transpose()
    print(coordinates.shape)

    #plt.plot(coordinates[:,0], coordinates[:,1], label = 'X e Y da coordinate')
    #plt.plot(X,Y, label = 'X e Y da funcao Poli3')
    #plt.legend()
    #plt.grid()
    #plt.show()
    #------------------------------------------------------------------------------------------------------------#
    x_t = np.gradient(coordinates[:, 0])
    y_t = np.gradient(coordinates[:, 1])

    #vel = np.array([ [x_t[i], y_t[i]] for i in range(x_t.size)])

    #print(vel)

    #Modulo da velocidade
    #speed = np.sqrt(x_t * x_t + y_t * y_t)

    #print(speed)
    #------------------------------------------------------------------------------------------------------------#
    #ss_t = np.gradient(speed)
    xx_t = np.gradient(x_t)
    yy_t = np.gradient(y_t)

    curvature_val = np.abs(xx_t * y_t - x_t * yy_t) / pow((x_t * x_t + y_t * y_t), 1.5)

    return curvature_val

#--------------------------------------------------------MOVIMENTO DO ROBO--------------------------------------------------------#
#Captura identificados do Pionner P3DX e dos motores esquerdo e direito
errorCode, robot = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_oneshot_wait)
errorCode, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

# Cinemática Direta
b_robo = 0.1655
rd = 0.1955/2       
re = 0.1955/2       
Mdir = np.array([[1, 0, b_robo/2],[1, 0, -b_robo/2], [0, 1, 0]])

#Configuração inicial e final do robô
qi = np.array([0, 0, 70])
qf = np.array([2,3, 80])
step = 0.01

qgoal_x, qgoal_y, qgoal_teta, a, b = Poli3(qi, qf, step)
#Goal configuration q
qgoal = np.array([qgoal_x, qgoal_y, qgoal_teta])
qgoal = np.transpose(qgoal)

gain = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])

cont = 0
x_pos = []
y_pos = []
teta = []
error = np.array([[0], 
                  [0], 
                  [0]])


#Captura configuração do robô iniciando stream de dados
errorCode, posRobot = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
errorCode, orientationrobot = sim.simxGetObjectOrientation(clientID,robot,-1,sim.simx_opmode_streaming)

for i in range(len(qgoal_teta)):
    #Captura dados da configuração dos robôs
    erro, [x, y, z] = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_buffer)
    erro, [alpha, beta, gamma] = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)
    x_pos.append(x)
    y_pos.append(y)
    teta.append((gamma*180)/mt.pi)

    #Imprime os dados da configuração do robô no CoppeliaSim
    sim.simxAddStatusbarMessage(clientID, "Posição em x " + str(x) + " e a Posição em y " +str(y), sim.simx_opmode_oneshot_wait)
    sim.simxAddStatusbarMessage(clientID, "Ângulo teta: " + str(gamma), sim.simx_opmode_oneshot_wait)

    #-------------------------------------------------------VARIAVEIS ROBO--------------------------------------------------------#
    q = np.array([x, y, gamma])

    error[0, 0] = qgoal[i,0] - q[0]
    error[1, 0] = qgoal[i,1] - q[1]
    error[2, 0] = qgoal[i,2] - q[2]
    qdot = np.dot(gain, error)

    Minv = np.linalg.inv(Mdir) #Calcula inversa
    print(Minv)
    Minv_aux = np.dot(iRr(q[2]), Minv)
    print(Minv_aux)
    u = np.dot(Minv_aux, qdot)
    #--------------------------------------------------------MOVIMENTO DAS RODAS--------------------------------------------------------#
    #Define velocidade dos motores e objetos
    #Motores
    errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, u[0, 0], sim.simx_opmode_oneshot_wait)
    errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, u[1, 0], sim.simx_opmode_oneshot_wait)

    if ((x > qf[0]) and (y > qf[1])):
        errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_oneshot_wait)
        errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_oneshot_wait)
        break

sim.simxPauseSimulation(clientID,sim.simx_opmode_oneshot_wait)

# Now close the connection to CoppeliaSim:
sim.simxFinish(clientID)

#Plotar (x(t), y(t))
plt.plot(x_pos, y_pos, linestyle = 'dashdot', color = 'g')
plt.xlabel('Coordenada x (m)')
plt.ylabel('Coordenada y (m)')
plt.title('Posição (x(t), y(t))')
plt.grid()
plt.show() 

#Plotar (x(λ), y(λ))
plt.plot(qgoal_x, qgoal_y, color = 'g')
plt.xlabel('Coordenada x (m)')
plt.ylabel('Coordenada y (m)')
plt.title('Posição (x(λ), y(λ))')
plt.grid()
plt.show()

#Plotar configuracao do robo
plt.plot(x_pos, label = 'Posição no eixo x', linestyle = 'dotted')
plt.plot(y_pos, label = 'Posição no eixo y', linestyle = '--')
plt.plot(teta, label = 'Angulo teta', linestyle = 'solid')
plt.ylabel('Posição (m)')
plt.xlabel('Tempo (s)')
plt.title('Configuração do robô (x, y, θ)')
plt.grid()
plt.legend()
plt.show()