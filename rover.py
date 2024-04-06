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

# Define initial positions and orientations
startPos = [0,0,3]
cajaPos = [0,3,0]
pos_dejar = [0,-0.5,1.5]

# Constants for controlling the robot
velocity = 11
force = 25

current_time = time.time()
# Load URDF models
carOrientation = p.getQuaternionFromEuler([0,0,3.15])

carModel = p.loadURDF(car, startPos, carOrientation)
cajaModel = p.loadURDF(caja, cajaPos, carOrientation)


# Get the number of joints in the ramp model
numJoints = p.getNumJoints(carModel)
print("NumJoints: {}".format(numJoints))
for j in range(numJoints):
      print("{} - {}".format(p.getJointInfo(carModel,j)[0], p.getJointInfo(carModel,j)[1].decode("utf-8")))

# Enable real-time simulation
p.setRealTimeSimulation(1)



# Solo queremos que la cinemática inversa actue sobre los primeros 3 JOINTS
robotEndEffectorIndex=3

try:
      
      # part of 3.2
      lateralFriction = 0.93
      spinningFriction = 0.005
      rollingFriction = 0.003
      for i in range(2, 6):
            p.changeDynamics(carModel, i, lateralFriction=lateralFriction, spinningFriction=spinningFriction, rollingFriction=rollingFriction)

    
      
      while True:
                  
            jointPoses = p.calculateInverseKinematics(carModel, robotEndEffectorIndex, cajaPos  )
            print(jointPoses)
            p.addUserDebugText("X", cajaPos  , [1,0,0], 1)


            for i in range(len(jointPoses)):
                  p.setJointMotorControl2(bodyIndex=carModel,
                                                      jointIndex=i,
                                                      controlMode=p.POSITION_CONTROL,
                                                      targetPosition=jointPoses[i],
                                                      targetVelocity=0,
                                                      force=500,
                                                      positionGain=0.03,
                                                      velocityGain=1)
            # Set the new camera target position
            #p.resetDebugVisualizerCamera(cam_distance, cam_yaw, cam_pitch, cam_target_pos)

    
            # p.setJointMotorControlArray(carModel, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=[velocity] * 4, forces=[force] * 4)
            # # Get wheel velocities
            # wheel_velocities = [p.getJointState(carModel, i)[1] for i in [2, 3, 4, 5]]
            # # Get wheel forces
            # wheel_forces = [p.getJointState(carModel, i)[3] for i in [2, 3, 4, 5]]
            # Add the data to the list

except KeyboardInterrupt:
   pass

# Disconnect from the physics server
p.disconnect()
