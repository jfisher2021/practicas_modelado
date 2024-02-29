import pybullet as p
import time
import csv
import pybullet_data



def write_to_csv(data):
    with open('robot_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Tiempo', 'Posición_Y', 'Velocidad_Y', 'Velocidad_Ruedas', 'Fuerza_Ruedas'])
        writer.writerows(data)


rampa = "urdf/ramp.urdf"
barra = "urdf/barrita.urdf"
meta = "urdf/final.urdf"
coche = "husky/husky.urdf"

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

startPos = [0,0,0]
finalPos = [0.0, 20.0, 0.05]
posBarra = [-1.5,17,0.5]

velocity = 11
force=25

startOrientation = p.getQuaternionFromEuler([0,0,3.15/2])
huskyOrientation = p.getQuaternionFromEuler([0,0,3.15/2])

terreneitor = p.loadURDF(coche, startPos, huskyOrientation)
rampas = p.loadURDF(rampa, startPos, startOrientation)
barraita = p.loadURDF(barra,posBarra, startOrientation)
final = p.loadURDF(meta, finalPos, startOrientation)

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
     current_pos = 0
     while True:
          posRobot, _ = p.getBasePositionAndOrientation(terreneitor)
          velRobot, _ = p.getBaseVelocity(terreneitor)
          pos_y = posRobot[1]
          
          p.setJointMotorControlArray(terreneitor, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=[velocity, velocity, velocity, velocity], forces=[force, force, force, force])
          # Calcular la distancia recorrida desde el último registro
          distance = pos_y - current_pos
          
          if distance >= 0.01:  # Si se ha movido al menos 0.01 metros
               current_pos = pos_y
               # Obtener velocidad de las ruedas
               wheel_velocities = [p.getJointState(terreneitor, i)[1] for i in [2, 3, 4, 5]]
               # Obtener fuerza de las ruedas
               wheel_forces = [p.getJointState(terreneitor, i)[3] for i in [2, 3, 4, 5]]
               # Agregar los datos a la lista
               data.append([time.time(), pos_y, velRobot[1], wheel_velocities[1], wheel_forces[1]])
          
          if pos_y > 20:  # Si el robot alcanza el final del escenario
               write_to_csv(data)  # Escribir datos en el archivo CSV
               break
except KeyboardInterrupt:
   pass
p.disconnect()

