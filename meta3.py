import numpy as np
import math as mt

a = np.array([1,2])
b = np.array([5,7])

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

def segueCaminho(q_ref) :
    pos = distMin(q_ref)
    errorcode = sim.simxSetObjectPosition(clientID, robot,-1, [pos[1], pos[2]], sim.simx_opmode_oneshot_wait)
    v = 0.1
    print(deltaTeta(q_ref,pos))
    W = samson(v, ks, deltaTeta_res, deltaL)

    vd = v - (b/2 * W)
    ve = v + (b/2 * W)

    wd = vd / rd
    we = ve/return

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
    deltaX = ref[1] - ref_pos[1]
    deltaY = ref[2] - ref_pos[2]

    phi = mt.atan2(deltaY, deltaX)

    delta_La = mt.sqrt(pow(deltaX,2) + pow(deltaY,2))
    delta_teta = phi - ref_pos[3]
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



def segueCaminho (q,v_q, q_ref,vq_ref)
    t = 0.05 #colocar como variavel global
    xa = q_ref[1]
    ya = q_ref[2]

    xa_vel = vq_ref[1]
    ya_vel = vq_ref[2]

    xa_acel = (vq_ref[1] - aux_v_ref[1]) / t
    ya_acel = (vq_ref[2] - aux_v_ref[2]) / t #declarar aux_v_ref como vetor nulo

    x = q[1]
    y = q[2]

    x_vel = v_q[1]
    y_vel = v_q[2]

    teta = q[3]

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
    q = np.array([posicao[1] + 0.0445, posicao[2], gamma])
    return q

def Pose_atual(ref_):
    errorcode, refPosicao = sim.simxGetObjectPosition(clientID, ref_,-1, sim.simx_opmode_oneshot_wait)
    errorcode, [alpha, beta, gamma] = sim.simxGetObjectOrientation(clientID, ref_, -1, sim.simx_opmode_buffer)
    qPose_atual = np.array([refPosicao[1], refPosicao[2], gamma])
    return qPose_atual

def deltaTeta (pos, min_dist) :
    y_coef = (-1)*(3 * pow(min_dist[1],2)) + 1
    ang_coef = mt.sqrt(pow(y_coef,2) + 1)

    coef = np.array([1/ang_coef, y_coef/ang_coef])

    deltaP = np.array([pos[1] - min_dist[1], pos[2] - min_dist[2]])

    tetaP = mt.deg(mt.atan(coef[2]/T[1]))
    tetaR = mt.deg(pos[3])
    deltaTeta_res = tetaR - tetaP

    return deltaTeta_res

def curve(X, Y)

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

    curvature_val = np.abs(xx_t * y_t - x_t * yy_t) / (x_t * x_t + y_t * y_t)**1.5

    return curvature_val