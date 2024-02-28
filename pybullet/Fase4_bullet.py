import pybullet as p
import time
import csv
import pybullet_data

#0.5359841334206371 --probar.csv
#0.5326216873831918 --buenpid1.csv


def write_to_csv(data):
    with open('probar.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Tiempo', 'Posición_Y', 'Velocidad_Y', 'Velocidad_Rueda_1', 'velocidad_Rueda_2', 'velocidad_Rueda_3', 'velocidad_Rueda_4', 'Fuerza_Rueda_1', 'Fuerza_Rueda_2', 'Fuerza_Rueda_3', 'Fuerza_Rueda_4'])
        writer.writerows(data)
rampa = "urdf/escenario1.urdf"
barra = "urdf/barrita.urdf"
meta = "urdf/final.urdf"
coche = "husky/husky.urdf"

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

startPos = [0,0,0]
finalPos = [0.0, 20.0, 0.05]
posBarra = [-1.5,17,0.55]

####PID####
Kp = 75
Ki = 33
Kd = 24

velocity = 11
force=25
error_anterior=0


startOrientation = p.getQuaternionFromEuler([0,0,3.15/2])
huskyOrientation = p.getQuaternionFromEuler([0,0,3.15/2])

terreneitor = p.loadURDF(coche, startPos, huskyOrientation)
rampas = p.loadURDF(rampa, startPos, startOrientation)
barrita = p.loadURDF(barra,posBarra, startOrientation)
final = p.loadURDF(meta, finalPos, startOrientation)

currenent_time = time.time()
#sacar la posicion del husky

numJoints = p.getNumJoints(rampas)
print("NumJoints: {}".format(numJoints))
for j in range(numJoints):
     print("{} - {}".format(p.getJointInfo(rampas,j)[0], p.getJointInfo(rampas,j)[1].decode("utf-8")))

numJointsROBOT = p.getNumJoints(terreneitor)

print("NumJoints ROBOT: {}".format(numJointsROBOT))
for j in range(numJointsROBOT):
     print("{} - {}".format(p.getJointInfo(terreneitor,j)[0], p.getJointInfo(terreneitor,j)[1].decode("utf-8")))

p.setRealTimeSimulation(1)
try:
     distance_threshold = 0.01  # Distance threshold for recording information
     data = []  # Lista para almacenar los datos
     current_pos = -1
     p.changeDynamics(barrita, 0, localInertiaDiagonal=[6.6, 0, 6.6])
     lateralFriction = 0.93
     spinningFriction =0.005
     rollingFriction = 0.003
     for i in range(2, 6):
          p.changeDynamics(terreneitor, i, lateralFriction=lateralFriction ,spinningFriction=spinningFriction, rollingFriction=rollingFriction)
     while True:
          posRobot, OriRobot = p.getBasePositionAndOrientation(terreneitor)
          velRobot, _ = p.getBaseVelocity(terreneitor)
          pos_y = posRobot[1]
          Euler = p.getEulerFromQuaternion(OriRobot)
          angleengrados = Euler[1]*180/3.1416
          # Calcular la distancia recorrida desde el último registro
          distance = pos_y - current_pos
          
          error_vel= 2.1 - velRobot[1]
          derivativo = abs(error_vel - error_anterior)
          integral = +error_vel
          velocity = Kp * error_vel + Ki * integral + Kd * derivativo
          if velocity > 33:
               velocity = 33
          error_anterior = error_vel
          if distance >= 0.01:  # Si se ha movido al menos 0.01 metros
               if (angleengrados > -39 and angleengrados < -2):
                    p.setJointMotorControlArray(terreneitor, [4, 5], p.VELOCITY_CONTROL, targetVelocities=[velocity*8.6] * 2, forces=[force] * 2)
               elif (angleengrados > 2):     
                    p.setJointMotorControlArray(terreneitor, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=[velocity/4] * 4, forces=[force] * 4)
                    print("bajada")
               else:
                    p.setJointMotorControlArray(terreneitor, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=[velocity] * 4, forces=[force] * 4)
                    print("plano")
               
               # print("angle", angleengrados)
               # print("error_vel", error_vel)
               print(velocity)
               current_pos = pos_y
               # Obtener velocidad de las ruedas
               wheel_velocities = [p.getJointState(terreneitor, i)[1] for i in [2, 3, 4, 5]]
               # Obtener fuerza de las ruedas
               wheel_forces = [p.getJointState(terreneitor, i)[3] for i in [2, 3, 4, 5]]
               # Agregar los datos a la lista
               data.append([round(time.time() - currenent_time, 4), round(pos_y, 4), round(velRobot[1], 4), round(wheel_velocities[0], 4), round(wheel_velocities[1], 4), round(wheel_velocities[2], 4), round(wheel_velocities[3], 4), round(wheel_forces[0], 4), round(wheel_forces[1], 4), round(wheel_forces[2], 4), round(wheel_forces[3], 4)])
          
          if pos_y > 20:  # Si el robot alcanza el final del escenario
               write_to_csv(data)  # Escribir datos en el archivo CSV
               break
except KeyboardInterrupt:
   pass
p.disconnect()

