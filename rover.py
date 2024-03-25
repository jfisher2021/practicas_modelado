import pybullet as p
import time
import pybullet_data


car = "urdf/robot_nuevo.urdf"

# Connect to the physics server
physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

# Load the plane
planeId = p.loadURDF("plane.urdf")

# Define initial positions and orientations
startPos = [0,0,2]
finalPos = [0.0, 20.0, 0.05]
barPos = [-1.5,17,0.55]

# Configure camera settings
cam_target_pos = [0, 0, 0]  # Position for camera to look at (initial position of the robot)
cam_distance = 5 # Distance from camera to target (adjust as needed)
cam_yaw = 45  # Horizontal rotation angle of the camera
cam_pitch = -45  # Vertical rotation angle of the camera

# Constants for controlling the robot
velocity = 11
force = 25

current_time = time.time()
# Load URDF models
startOrientation = p.getQuaternionFromEuler([0,0,3.15/2])
carOrientation = p.getQuaternionFromEuler([0,0,3.15/2])

carModel = p.loadURDF(car, startPos, carOrientation)


# Get the number of joints in the ramp model
numJoints = p.getNumJoints(carModel)
print("NumJoints: {}".format(numJoints))
for j in range(numJoints):
      print("{} - {}".format(p.getJointInfo(carModel,j)[0], p.getJointInfo(carModel,j)[1].decode("utf-8")))

# Enable real-time simulation
p.setRealTimeSimulation(1)

try:
      
      # part of 3.2
      lateralFriction = 0.93
      spinningFriction = 0.005
      rollingFriction = 0.003
      for i in range(2, 6):
            p.changeDynamics(carModel, i, lateralFriction=lateralFriction, spinningFriction=spinningFriction, rollingFriction=rollingFriction)

    
      
      while True:
            print("a")
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
