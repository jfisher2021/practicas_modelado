import pybullet as p
import time
import pybullet_data

urdf_path = "urdf/practica0.urdf"

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

frictionId = p.addUserDebugParameter("jointFriction", 0, 20, 10)
torqueId = p.addUserDebugParameter("torque force", 0, 20, 0)
velocityId = p.addUserDebugParameter("jointVelocity", -20, 20, 0)

torqueId0 = p.addUserDebugParameter("long_arm", -40, 40, 0)
try:
    while True:
        
        jointTorque = p.readUserDebugParameter(torqueId)
        frictionForce = p.readUserDebugParameter(frictionId)
        velocity = p.readUserDebugParameter(velocityId)
        jointTorque0 = p.readUserDebugParameter(torqueId0)
          
        p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
        p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=jointTorque)
        p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=velocity, force=frictionForce)
        p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=jointTorque0)
        p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, targetVelocity=20, force=jointTorque0)
        p.stepSimulation()
        time.sleep(1./240.)

except KeyboardInterrupt:
      pass
	
p.disconnect()    