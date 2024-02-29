import pybullet as p
import time
import csv
import pybullet_data

# Function to write data to a CSV file
def write_to_csv(data):
     with open('csvs/probar.csv', mode='w', newline='') as file:
          writer = csv.writer(file)
          writer.writerow(['Time', 'Position_Y', 'Velocity_Y', 'Wheel_1_Velocity', 'Wheel_2_Velocity', 'Wheel_3_Velocity', 'Wheel_4_Velocity', 'Wheel_1_Force', 'Wheel_2_Force', 'Wheel_3_Force', 'Wheel_4_Force'])
          writer.writerows(data)

# File paths for URDF models
ramp = "urdf/escenario1.urdf"
bar = "urdf/barrita.urdf"
goal = "urdf/final.urdf"
car = "husky/husky.urdf"

# Connect to the physics server
physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

# Load the plane
planeId = p.loadURDF("plane.urdf")

# Define initial positions and orientations
startPos = [0,0,0]
finalPos = [0.0, 20.0, 0.05]
barPos = [-1.5,17,0.55]

# Configure camera settings
cam_target_pos = [0, 0, 0]  # Position for camera to look at (initial position of the robot)
cam_distance = 5 # Distance from camera to target (adjust as needed)
cam_yaw = 45  # Horizontal rotation angle of the camera
cam_pitch = -45  # Vertical rotation angle of the camera

# Constants for controlling the robot
best_velocity = 11.2
velDown = 8.9
force = 25
num = 0

current_time = time.time()
# Load URDF models
startOrientation = p.getQuaternionFromEuler([0,0,3.15/2])
carOrientation = p.getQuaternionFromEuler([0,0,3.15/2])

carModel = p.loadURDF(car, startPos, carOrientation)
rampModel = p.loadURDF(ramp, startPos, startOrientation)
barModel = p.loadURDF(bar, barPos, startOrientation)
goalModel = p.loadURDF(goal, finalPos, startOrientation)

# Get the number of joints in the ramp model
numJoints = p.getNumJoints(rampModel)
print("NumJoints: {}".format(numJoints))
for j in range(numJoints):
      print("{} - {}".format(p.getJointInfo(rampModel,j)[0], p.getJointInfo(rampModel,j)[1].decode("utf-8")))

# Get the number of joints in the car model
numJointsCar = p.getNumJoints(carModel)
print("NumJoints Car: {}".format(numJointsCar))
for j in range(numJointsCar):
      print("{} - {}".format(p.getJointInfo(carModel,j)[0], p.getJointInfo(carModel,j)[1].decode("utf-8")))

# Enable real-time simulation
p.setRealTimeSimulation(1)

try:
      distance_threshold = 0.01  # Distance threshold for recording information
      data = []  # List to store the data
      current_pos = -1
      p.changeDynamics(barModel, 0, localInertiaDiagonal=[6.6, 0, 6.6])
      lateralFriction = 0.93
      spinningFriction = 0.005
      rollingFriction = 0.003
      for i in range(2, 6):
            p.changeDynamics(carModel, i, lateralFriction=lateralFriction, spinningFriction=spinningFriction, rollingFriction=rollingFriction)
      
      while True:
            # Get the position and orientation of the car
            carPos, carOri = p.getBasePositionAndOrientation(carModel)
            carVel, _ = p.getBaseVelocity(carModel)
            pos_y = carPos[1]
            Euler = p.getEulerFromQuaternion(carOri)
            angle_degrees = Euler[1] * 180 / 3.1416

            # Calculate the distance traveled since the last recording
            distance = pos_y - current_pos

            # Set the new camera target position
            cam_target_pos = carPos
            p.resetDebugVisualizerCamera(cam_distance, cam_yaw, cam_pitch, cam_target_pos)

            if distance >= 0.01:  # If the car has moved at least 0.01 meters
                  if (angle_degrees > -39 and angle_degrees < -10):
                         num += 1  
                         p.setJointMotorControlArray(carModel, [4, 5], p.VELOCITY_CONTROL, targetVelocities=[i * 6.5] * 2, forces=[force] * 2)
                  elif (angle_degrees > 2):
                         p.setJointMotorControlArray(carModel, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=[velDown] * 4, forces=[force * 3] * 4)
                  else:
                         p.setJointMotorControlArray(carModel, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=[best_velocity] * 4, forces=[force] * 4)

                  current_pos = pos_y
                  # Get wheel velocities
                  wheel_velocities = [p.getJointState(carModel, i)[1] for i in [2, 3, 4, 5]]
                  # Get wheel forces
                  wheel_forces = [p.getJointState(carModel, i)[3] for i in [2, 3, 4, 5]]
                  # Add the data to the list
                  data.append([round(time.time() - current_time, 4), round(pos_y, 4), round(carVel[1], 4), round(wheel_velocities[0], 4), round(wheel_velocities[1], 4), round(wheel_velocities[2], 4), round(wheel_velocities[3], 4), round(wheel_forces[0], 4), round(wheel_forces[1], 4), round(wheel_forces[2], 4), round(wheel_forces[3], 4)])

            if pos_y > 20:  # If the car reaches the end of the scenario
                  write_to_csv(data)  # Write data to the CSV file
                  break
except KeyboardInterrupt:
   pass

# Disconnect from the physics server
p.disconnect()
