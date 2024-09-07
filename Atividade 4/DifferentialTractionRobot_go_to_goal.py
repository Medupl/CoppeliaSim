from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time

client = RemoteAPIClient()
sim = client.getObject("sim")

# Função de normalização dos ângulos ao intervalo [-pi, pi)
def normalizeAngle(angle):
    return np.mod(angle+np.pi, 2*np.pi) - np.pi

# Handle do P3DX
handle_robo = sim.getObject("/DifferentialTractionRobot")

# Handle dos motores
roda_direita = sim.getObject('/DifferentialTractionRobot/RightEngine')
roda_esquerda = sim.getObject('/DifferentialTractionRobot/LeftEngine')

# Dados do robo DifferentialTractionRobot
L = 0.2
r = 0.0325
maxv = 1
maxw = np.deg2rad(45)

# Definir o meu Goal -> array (x, y, theta)
handle_goal = sim.getObject("/Ponto1")
pos_goal = sim.getObjectPosition(handle_goal,-1)
theta_goal = sim.getObjectOrientation(handle_goal,-1)[2]

#rho inicia muito grande
rho = np.inf

sim.startSimulation()

# criando variavel para controlar o percurso do robô.
chegada = 0

# Enquanto meu robô estiver a uma distância maior que 5cm do goal, execute: 
while chegada < 3:
    # Obter posição e orientação atual do robô
    pos_robo = sim.getObjectPosition(handle_robo,-1)
    theta_robo = sim.getObjectOrientation(handle_robo,-1)[2]
    
    # calculo do erro em x, y e theta
    dt_x = pos_goal[0] - pos_robo[0]
    dt_y = pos_goal[1] - pos_robo[1]
    theta = theta_goal - theta_robo
    theta = normalizeAngle(theta)
    
    # variaveis de estado atual (alpha, beta e rho) -> usar equações do slide
    rho = np.sqrt((dt_x**2) + (dt_y**2))
    alpha = (-theta_robo) + np.arctan2(dt_y,dt_x)
    beta = theta_goal - np.arctan2(dt_y,dt_x)
    alpha = normalizeAngle(alpha)
    beta = normalizeAngle(beta)
   
    # definir valores de constante proporcial
    # tem que sair testando a melhor opção para o modelo de seu carro.
    kr = 0.07
    ka = 0.5
    kb = -0.3

    # checar se o goal está na frente do robô -> | alpha | > 90° ? 
    # se sim, a velocidade linear deve ser negativa
    # se sim, alpha e beta devem ser normalizados para o intervalo
    if np.abs(alpha) > np.deg2rad(90):
        alpha = normalizeAngle(alpha-np.pi)
        beta = normalizeAngle(beta-np.pi)
    # calculo da velocidade linear e angular
        v = -(kr * rho)         # velocidade linear
    else:
        v = kr * rho            # velocidade linear
    w = ka * alpha + kb * beta  # velocidade angular

    # lembre-se de limitar as velocidades ao valor maximo do robo
    v = np.clip(v, -maxv, maxv)     # limitando a velocidade linear
    w = np.clip(w, -maxw, maxw)     # limitando a velocidade angular

    # calculo de WL e WR a partir de v e w
    wl = ((2 * v) - (w * L)) / (2 * r)  # Calculo da velocidade angular esquerda
    wr = ((2 * v) + (w * L)) / (2 * r)  # Calculo da velocidade angular direita
    #wl = np.deg2rad(wl)                 # Ajuste para velocidade ficar em radianos
    #wr = np.deg2rad(wr)                 # Ajuste para velocidade ficar em radianos

    # definir a velocidade dos motores do P3DX
    vel_esq = sim.setJointTargetVelocity(roda_esquerda, wl)
    vel_dir = sim.setJointTargetVelocity(roda_direita, wr)
    if (rho < 0.07):
        # ao alcançar esta proximidade com o alvo, incrementa a variavél chegada.
        chegada = chegada + 1 

        if chegada == 1:
            # caso o robô alcance o primeiro alvo, o goal irá mudar.
            handle_goal = sim.getObject("/Ponto2")
            pos_goal = sim.getObjectPosition(handle_goal,-1)
            theta_goal = sim.getObjectOrientation(handle_goal,-1)[2]

        if chegada == 2:
            # caso o robô alcance o segundo alvo, o goal irá mudar.
            handle_goal = sim.getObject("/Center")
            pos_goal = sim.getObjectPosition(handle_goal,-1)
            theta_goal = sim.getObjectOrientation(handle_goal,-1)[2]

# Parar o robô
vel_esq = sim.setJointTargetVelocity(roda_esquerda, 0)
vel_dir = sim.setJointTargetVelocity(roda_direita, 0)

# Finalizar simulação
sim.stopSimulation()
