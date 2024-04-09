import pybullet as p
import time
import pybullet_data

car = "urdf/probando.urdf"
caja = "urdf/caja.urdf"

# Connect to the physics server
physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

# Load the plane
planeId = p.loadURDF("plane.urdf")

#### STATE MACHINE ####
ACERCRSE = 1
ARRIBA_CAJA = 2
COGER_CAJA = 3
CLOSE_GRIPPER = 4
ARRIBA_ROBOT = 5
DEJAR_CAJA = 6
ACTUAL = ACERCRSE

########### POSITIONS ############
startPos = [0,0,2]
UpBox = [0,4,2]
boxPos = [0,3.9,0]
UpCar = [0,4,3.5]
InCar = [0,2,3]

# Constants for controlling the robot
velocity = 20
force = 20

current_time = time.time()
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
# Enable real-time simulation
p.setRealTimeSimulation(1)

prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 1

# Solo queremos que la cinemática inversa actue sobre los primeros 3 JOINTS
robotEndEffectorIndex=4
going = True

# Vamos a posicionar el brazo por una diagonal X,Y usando la misma altura (Z)
# El rango máximo/minimo definido
range_m=[0.3,2.0]

c=range_m[0]

trailDuration = 5

# Solo queremos que la inemática inversa actue sobre los primeros 5 JOINTS
robotEndEffectorIndex=5

try:
      current_pos = -1
      # part of 3.2
      lateralFriction = 0.93
      spinningFriction = 1
      for i in range(2, 6):
            p.changeDynamics(carModel, i, lateralFriction=lateralFriction, spinningFriction=spinningFriction)

      while True:
           
                  
            # Get the position and orientation of the car
            carPos, carOri = p.getBasePositionAndOrientation(carModel)
            pos_y = carPos[1]
            
            if (ACTUAL == ACERCRSE):
                  # Calcular la distancia restante hasta el punto de detención (1 metro antes de la caja)
                  distancia_restante = 1 - pos_y
                  # Ajustar la velocidad proporcionalmente a la distancia restante, con un mínimo de 0
                  actual_force = force * (distancia_restante)
                  # Aplicar la velocidad actual al robot
                  p.setJointMotorControlArray(carModel, [7, 8, 9, 10], p.VELOCITY_CONTROL, targetVelocities=[velocity] * 4, forces=[actual_force] * 4)
                  
                  if pos_y >= 0.8:
                        ACTUAL = ARRIBA_CAJA      
                        print("ARRIBA_CAJA")
                        
            elif (ACTUAL == ARRIBA_CAJA):
                  p.setJointMotorControlArray(carModel, [7,8,9,10], p.VELOCITY_CONTROL, targetVelocities=[0] * 4, forces=[100] * 4)
                  
                  jointPoses = p.calculateInverseKinematics(carModel, robotEndEffectorIndex, UpBox)
                  p.addUserDebugText("X", UpBox  , [1,0,0], 1)
            
                  p.setJointMotorControlArray(carModel, armJoints, p.POSITION_CONTROL,targetPositions=jointPoses[0:len(armJoints)],forces=[2000]*len(armJoints),positionGains=[0.01]*len(armJoints),velocityGains=[2]*len(armJoints))
                  # sacar posicion del brazo
                  ls = p.getLinkState(carModel, robotEndEffectorIndex)
                  position = ls[0] 
                  print("ls1: {}".format(position[1]))
                  print("ls2: {}".format(position[2]))
                  if (position[1] >= 4 and position[2] >= 1.99):
                        ACTUAL = COGER_CAJA
                        print("COGER_CAJA")
                  
            elif (ACTUAL == COGER_CAJA):
                  jointPoses = p.calculateInverseKinematics(carModel, robotEndEffectorIndex, boxPos)
                  p.addUserDebugText("X", boxPos  , [1,0,0], 1)
            
                  p.setJointMotorControlArray(carModel, armJoints, p.POSITION_CONTROL,targetPositions=jointPoses[0:len(armJoints)],forces=[2000]*len(armJoints),positionGains=[0.01]*len(armJoints),velocityGains=[2]*len(armJoints))
                  # sacar posicion del brazo
                  ls = p.getLinkState(carModel, robotEndEffectorIndex)
                  
                  
                  position = ls[0] 
                  
                  print("ls1: {}".format(position[1]))
                  print("ls2: {}".format(position[2]))
                  if (position[1] <= 4.1 and position[2] <= 0.38):
                        ACTUAL = CLOSE_GRIPPER
                        print("CLOSE_GRIPPER")
            elif (ACTUAL == CLOSE_GRIPPER):
                  p.setJointMotorControlArray(carModel, [5,6], p.POSITION_CONTROL, targetPositions=[-0.20,-0.20],forces=[300]*2)
                  time.sleep(1)
                  ACTUAL = ARRIBA_ROBOT
                  print("ARRIBA_ROBOT")
            elif (ACTUAL == ARRIBA_ROBOT):
                  jointPoses = p.calculateInverseKinematics(carModel, robotEndEffectorIndex, UpCar)
                  p.addUserDebugText("X", UpCar  , [1,0,0], 1)
            
                  p.setJointMotorControlArray(carModel, armJoints, p.POSITION_CONTROL,targetPositions=jointPoses[0:len(armJoints)],forces=[2000]*len(armJoints),positionGains=[0.01]*len(armJoints),velocityGains=[2]*len(armJoints))
                  # sacar posicion del brazo
                  ls = p.getLinkState(carModel, robotEndEffectorIndex)
                  
                  
                  position = ls[0] 
                  
                  print("ls1: {}".format(position[1]))
                  print("ls2: {}".format(position[2]))
                  if (position[2] >= 3.4):
                        ACTUAL = DEJAR_CAJA
                        print("DEJAR_CAJA")
            elif (ACTUAL == DEJAR_CAJA):
                  jointPoses = p.calculateInverseKinematics(carModel, robotEndEffectorIndex, InCar)
                  p.addUserDebugText("X", InCar  , [1,0,0], 1)
            
                  p.setJointMotorControlArray(carModel, armJoints, p.POSITION_CONTROL,targetPositions=jointPoses[0:len(armJoints)],forces=[2000]*len(armJoints),positionGains=[0.01]*len(armJoints),velocityGains=[2]*len(armJoints))
                  # sacar posicion del brazo
                  ls = p.getLinkState(carModel, robotEndEffectorIndex)
                  
                  
                  position = ls[0] 
                  
                  print("ls1: {}".format(position[1]))
                  print("ls2: {}".format(position[2]))
                  if (position[1] <= 0.33 and position[2] <= 3):
                        p.setJointMotorControlArray(carModel, [5,6], p.POSITION_CONTROL, targetPositions=[0.0,0.0],forces=[10]*2)

                  
            
                  
                   

except KeyboardInterrupt:
   pass

# Disconnect from the physics server
p.disconnect()
