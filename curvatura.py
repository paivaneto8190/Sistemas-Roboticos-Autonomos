import numpy as np
import math as mt
import matplotlib.pyplot as plt

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

qi = np.array([1, 0, 10])
qf = np.array([3,5, 80])
step = 0.01

X, Y, teta_func, a, b = Poli3(qi, qf, step)

#-------------------------------------------PROGRAMA DE TESTE PARA CURVATURA-----------------------------------------------------------------#

coordinates = np.array([X,Y])
coordinates = coordinates.transpose()
print(coordinates.shape)

plt.plot(coordinates[:,0], coordinates[:,1], label = 'X e Y da coordinate')
plt.plot(X,Y, label = 'X e Y da funcao Poli3')
plt.legend()
plt.grid()
plt.show()
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

print("Curvatura: ",curvature_val)



