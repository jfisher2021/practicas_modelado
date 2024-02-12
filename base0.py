import pybullet as p
import time
import pybullet_data

urdf_path = "urdf/example_continuous.urdf"

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane_transparent.urdf")

startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,-3.15])

robotId = p.loadURDF(urdf_path,startPos, startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: {}".format(numJoints))
for j in range(numJoints):
     print("{} - {}".format(p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))


right_id = p.addUserDebugParameter("rightMotor", -1, 1, 0)
left_id = p.addUserDebugParameter("leftMotor", -1, 1, 0)

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)

        right_motor_value = p.readUserDebugParameter(right_id)
        left_motor_value = p.readUserDebugParameter(left_id)
        
        p.setJointMotorControl2(robotId,1, p.VELOCITY_CONTROL, targetVelocity=right_motor_value)
        p.setJointMotorControl2(robotId,0, p.VELOCITY_CONTROL, targetVelocity=left_motor_value)
        
except KeyboardInterrupt:
      pass
	
p.disconnect()    