import sim 
import sys
import time
import math as mt
import numpy as np

sim.simxFinish(-1)

clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID != -1:
    print ("Conectado ao servidro API")

else: 
    print ("Não conectado")
    sys.exit("Não foi possivel conectar")

