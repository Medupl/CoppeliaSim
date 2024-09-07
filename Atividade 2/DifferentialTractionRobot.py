from coppeliasim_zmqremoteapi_client import *
import numpy as np
import time

# Conexão com o CoppeliaSim
client = RemoteAPIClient()
sim = client.getObject("sim")

# Handles do robô
handle_robo = sim.getObject("/DifferentialTractionRobot")
rightEngine = sim.getObject("/DifferentialTractionRobot/RightEngine")
leftEngine = sim.getObject("/DifferentialTractionRobot/LeftEngine")

# Iniciamndo simulação
sim.startSimulation()

# Dados do robô
L = 0.2
r = 0.0325

# Calculando a velocidade angular e linear do robô
w = np.deg2rad(50)
v = 1

# Calculando as velocidades angulares dos motores
wr = ((2 * v) + (w * L)) / (2 * r)
wl = ((2 * v) - (w * L)) / (2 * r) + 5.3702438523
print(wr,wl)
wr = np.deg2rad(wr)
wl = np.deg2rad(wl)

# Atribuindo as velocidades ao robô DifferentialTractionRobot
speed_right = sim.setJointTargetVelocity(rightEngine,wr)
speed_left = sim.setJointTargetVelocity(leftEngine,wl)
x2 = 0

# Aplicando a função time

time.sleep(5)  

  

# Parando a simulação
sim.stopSimulation()