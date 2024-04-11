import pybullet as p
import pybullet_data
import time
import csv
import matplotlib.pyplot as plt
import numpy as np

# Function to write data to a CSV file
def write_to_csv(data):
     with open('csvs/Fase3.csv', mode='w', newline='') as file:
          writer = csv.writer(file)
          writer.writerow(['Time', 'Position_Y', 'Velocity_Y', 'Wheel_1_Velocity', 'Wheel_2_Velocity', 'Wheel_3_Velocity', 'Wheel_4_Velocity', 'Wheel_1_Force', 'Wheel_2_Force', 'Wheel_3_Force', 'Wheel_4_Force'])
          writer.writerows(data)
          
car = "urdf/probando.urdf"
caja = "urdf/caja.urdf"

# Connect to the physics server
physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

current_time = time.time()
# Load the plane
planeId = p.loadURDF("plane.urdf")

#### STATE MACHINE ####
ACERCRSE = 1
ARRIBA_CAJA = 2
COGER_CAJA = 3
CLOSE_GRIPPER = 4
ARRIBA_ROBOT = 5
DEJAR_CAJA = 6
RETURN_UP = 7
RETURN = 8
END = 9
ACTUAL = ACERCRSE

########### POSITIONS ############
position_arm = [0,0,0]
startPos = [0,0,2]
UpBox = [0,4,2]
boxPos = [0,3.9,0]
UpCar = [0,4,3.6]
InCar = [0,0.5,3]
UpReturn = [0,2,3.5]

# Constants for controlling the robot
velocity = 20
force = 20

# Variables para controlar el muestreo y el tiempo de simulación
time_step = 0.005  # Paso de tiempo para la simulación
sample_time = 0.01  # Tiempo para muestrear los datos
total_time = 15.0  # Tiempo total de simulación
current_sample_time = 0
current_simulation_time = 0

# Load URDF models
carOrientation = p.getQuaternionFromEuler([0,0,3.15])

carModel = p.loadURDF(car, startPos, carOrientation)
cajaModel = p.loadURDF(caja, boxPos, carOrientation)


# Get the number of joints in the ramp model
numJoints = p.getNumJoints(carModel)
print("NumJoints: {}".format(numJoints))
for j in range(numJoints):
      print("{} - {}".format(p.getJointInfo(carModel,j)[0], p.getJointInfo(carModel,j)[1].decode("utf-8")))
armJoints = [0, 1, 3]
endEfector = [5, 6]


# Solo queremos que la inemática inversa actue sobre los primeros 5 JOINTS
robotEndEffectorIndex=5
# p.setRealTimeSimulation(1)
p.setJointMotorControl2(carModel, 0, p.VELOCITY_CONTROL, targetVelocity=50, force=2000)
try:
      data = []  # List to store the data
      current_pos = -1
      # part of 3.2
      lateralFriction = 0.93
      spinningFriction = 1
      for i in range(7, 11):
            p.changeDynamics(carModel, i, lateralFriction=lateralFriction, spinningFriction=spinningFriction)
      for i in armJoints:
            p.enableJointForceTorqueSensor(carModel, i)
      while True:
            # No es necesario, solo para ver más despacio el movimiento.
            p.stepSimulation()
            time.sleep(0.005)
            
            current_simulation_time = time.time() - current_time
                  
        
            if (ACTUAL == ACERCRSE):
                  # Get the position and orientation of the car
                  carPos, carOri = p.getBasePositionAndOrientation(carModel)
                  pos_y = carPos[1]
                  # Calcular la distancia restante hasta el punto de detención (1 metro antes de la caja)
                  distancia_restante = 1 - pos_y
                  # Ajustar la velocidad proporcionalmente a la distancia restante, con un mínimo de 0
                  actual_force = force * (distancia_restante)
                  # Aplicar la velocidad actual al robot
                  p.setJointMotorControlArray(carModel, [7, 8, 9, 10], p.VELOCITY_CONTROL, targetVelocities=[velocity] * 4, forces=[actual_force] * 4)
                  
                  if pos_y >= 0.8:
                        ACTUAL = COGER_CAJA      
                        print("COGER_CAJA")
            else : 
                  if current_simulation_time >= current_sample_time:
                        current_sample_time += sample_time
                        print("current_simulation_time: {}".format(current_simulation_time))
                        print("current_sample_time: {}".format(current_sample_time))
                        G_parcial = 0
                        # for i in armJoints:
                        #       joint_force0 = p.getJointState(carModel, )[2]
                              
                        joint_force0 = p.getJointState(carModel, 0)[2]
                        joint_force1 = p.getJointState(carModel, 1)[2]
                        joint_force2 = p.getJointState(carModel, 3)[2]
                        abs_value0 = abs(joint_force0[0])
                        abs_value1 = abs(joint_force0[1])
                        abs_value2 = abs(joint_force0[2])
                        abs_value10 = abs(joint_force1[0])
                        abs_value11 = abs(joint_force1[1])
                        abs_value12 = abs(joint_force1[2])
                        abs_value20 = abs(joint_force2[0])
                        abs_value21 = abs(joint_force2[1])
                        abs_value22 = abs(joint_force2[2])
                        abs_value_array = [abs_value0, abs_value1, abs_value2, abs_value10, abs_value11, abs_value12, abs_value20, abs_value21, abs_value22]
                        # print("joint_force: {}".format(joint_force))
                        G_parcial += sum(abs_value_array) # Sumamos los valores absolutos de las fuerzas
                        data.append([current_simulation_time, robotEndEffectorIndex, G_parcial])

                  # sacar posicion del brazo
                  p.setJointMotorControlArray(carModel, [7,8,9,10], p.VELOCITY_CONTROL, targetVelocities=[0] * 4, forces=[100] * 4)
                  ls = p.getLinkState(carModel, robotEndEffectorIndex)
                  position = ls[0] 
                  print("ls1: {}".format(position[1]))
                  print("ls2: {}".format(position[2]))
                  if (ACTUAL == COGER_CAJA):
                        position_arm = boxPos
                        ForceJoint = 100
                        if (position[1] <= 4.1 and position[2] <= 0.38):
                              ACTUAL = CLOSE_GRIPPER
                              print("CLOSE_GRIPPER")
                  elif (ACTUAL == CLOSE_GRIPPER):
                        p.setJointMotorControlArray(carModel, [5,6], p.POSITION_CONTROL, targetPositions=[-0.20,-0.20],forces=[800]*2)
                        if (p.getJointState(carModel, 5)[0] <= -0.15 and p.getJointState(carModel, 6)[0] <= -0.16):
                              ACTUAL = ARRIBA_ROBOT
                              print("ARRIBA_ROBOT")
                  elif (ACTUAL == ARRIBA_ROBOT):
                        position_arm = UpCar
                        ForceJoint = 1500
                        if (position[2] >= 2.8):
                              ACTUAL = DEJAR_CAJA
                              print("DEJAR_CAJA")
                  elif (ACTUAL == DEJAR_CAJA):
                        position_arm = InCar
                        ForceJoint = 1500
                        if (position[1] <= 0.433 and position[2] <= 3.1):
                              p.setJointMotorControlArray(carModel, [5,6], p.POSITION_CONTROL, targetPositions=[0.0,0.0],forces=[50]*2)
                              ACTUAL = RETURN_UP
                              
                              print("RETURN_UP")
                  elif (ACTUAL == RETURN_UP):
                        ForceJoint = 2000
                        position_arm = UpReturn
                        if (position[2] >= 3.2):
                              ACTUAL = RETURN
                              print("RETURN")
                              
                  elif (ACTUAL == RETURN):
                        position_arm = UpCar
                        ForceJoint = 1500
                        if (position[1] >= 4 and position[2] >= 3):
                              ACTUAL = END
                              print("END")
                              # if (position[1] <= 4.2 and position[2] <= 0.358):
                              #       print("END2")
                                               
                              # Calculamos G-total como la suma de todos los G-parciales
                              G_total = sum([row[2] for row in data])


                              # Guardamos los datos en un archivo CSV
                              with open('g_parcial_data.csv', 'w', newline='') as file:
                                    writer = csv.writer(file)
                                    writer.writerow(["Tiempo", "NúmeroJoints", "G_parcial"])
                                    writer.writerows(data)

                              # Generamos el plot
                              times = [row[0] for row in data]
                              G_parciales = [row[2] for row in data]
                              plt.plot(times, G_parciales)
                              plt.title(f'G-Total: {G_total:.2f}, Desviación Estándar G-parcial: {np.std(G_parciales):.2f}')
                              plt.xlabel('Tiempo (s)')
                              plt.ylabel('G-parcial')
                              plt.show()                  
                  #             break
                  # elif (ACTUAL == END):
                  #       ForceJoint = 100
                  #       position_arm = boxPos
                        
                        
                  actual_position = position_arm
                  jointPoses = p.calculateInverseKinematics(carModel, robotEndEffectorIndex, position_arm)
                  p.addUserDebugText("X", position_arm  , [1,0,0], 1)
            
                  p.setJointMotorControlArray(carModel, armJoints, p.POSITION_CONTROL,targetPositions=jointPoses[0:len(armJoints)],forces=[ForceJoint]*len(armJoints),
                                              positionGains=[0.01]*len(armJoints),velocityGains=[0.5]*len(armJoints))
 
                  
                  

                        
                        

                  
            
                  
                   

except KeyboardInterrupt:
   pass

# Disconnect from the physics server
p.disconnect()
