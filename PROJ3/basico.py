import sim
from time import sleep as delay
import numpy as np
import cv2
import sys
import matplotlib.pyplot as plt

print('Program started')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

lSpeed = 0
rSpeed = 0

if (clientID != -1):
    print('Connected to remote API server')

else:
    sys.exit('Failed connecting to remote API server')

delay(1)

robotname = 'Pioneer_p3dx'

errorCode, handlerobo = sim.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx', sim.simx_opmode_oneshot_wait)

errorCode, left_motor_handle = sim.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = sim.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

returnCode, sonar_front = sim.simxGetObjectHandle(clientID, robotname + '_ultrasonicSensor4', sim.simx_opmode_oneshot_wait)
returnCode, sonar_right = sim.simxGetObjectHandle(clientID, robotname + '_ultrasonicSensor7', sim.simx_opmode_oneshot_wait)



errorCode, camera_handle = sim.simxGetObjectHandle(
    clientID, 'cam1', sim.simx_opmode_oneshot_wait)
delay(1)

returnCode, resolution, image = sim.simxGetVisionSensorImage(
    clientID, camera_handle, 0, sim.simx_opmode_streaming)
delay(1)

ultrassonic_data = []
ultrassonic_ang = []
t = 0
try:
    while (t < 20):
        returnCode, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera_handle, 0, sim.simx_opmode_buffer)
        im = np.array(image, dtype=np.uint8)
        im.resize([resolution[0], resolution[1], 3])

        im = cv2.flip(im, 0)
        im = cv2.rotate(im, cv2.ROTATE_90_COUNTERCLOCKWISE)
        im = cv2.resize(im, (512, 512))
        im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)

        errorCode = sim.simxSetJointTargetVelocity(
            clientID, left_motor_handle, lSpeed, sim.simx_opmode_streaming)
        errorCode = sim.simxSetJointTargetVelocity(
            clientID, right_motor_handle, rSpeed, sim.simx_opmode_streaming)

        cv2.imshow("data", im)
        com = cv2.waitKey(1)
        if (com == ord('q')):
            break
        elif (com == ord('w')):
            lSpeed = 1
            rSpeed = 1
        elif (com == ord('a')):
            lSpeed = -0.5
            rSpeed = 1
        elif (com == ord('d')):
            lSpeed = 1
            rSpeed = -0.5
        elif (com == ord('s')):
            lSpeed = -1
            rSpeed = -1
        else:
            lSpeed = 0
            rSpeed = 0
        com = 'o'

# Fazendo leitura dos sensores
        returnCode, detected_front, point_front, *_ = sim.simxReadProximitySensor(clientID, sonar_front, sim.simx_opmode_oneshot_wait)
        returnCode, detected_right, point_right, *_ = sim.simxReadProximitySensor(clientID, sonar_right, sim.simx_opmode_oneshot_wait)        

        #print("DetF: ", detected_front, " PoiF:" , point_front)
        #print("DetR: ", detected_right, " PoiR:" , point_right)


        returnCode, [x,y,z] = sim.simxGetObjectPosition(clientID, handlerobo, -1, sim.simx_opmode_streaming)
        returnCode, [e, t, gamma] = sim.simxGetObjectOrientation(clientID, handlerobo, -1, sim.simx_opmode_streaming)

        distObs = point_front[2] 
        ultrassonic_data.append(distObs)
        ultrassonic_ang.append(gamma)

        print('distObs: ' ,distObs)
        print('x: ', x , '\ny: ', y)
        print('detecF: ', detected_front )
        t +=1
        delay(1)


    cv2.destroyAllWindows()
except:
    cv2.destroyAllWindows()


dist = np.copy(ultrassonic_data)
ang = np.copy(ultrassonic_ang)
        
for i in range(len(dist)):
        
    # Quando o feixe não acerta nada, retorna o valor máximo (definido na simulação)
    # Logo, usar um pequeno limiar do máximo para considerar a leitura
    if (2 - dist[i]) > 0.1:
        x = dist * np.cos(ang[i])
        y = dist * np.sin(ang[i])
        c = 'r'
        if ang[i] < 0:    
            c = 'b'
        plt.plot(x, y, 'o', color=c)

plt.plot(0, 0, 'k>', markersize=10)
print(dist)
print(ang)
plt.grid()
plt.plot()
