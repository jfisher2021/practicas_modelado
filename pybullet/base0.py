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

frictionId = p.addUserDebugParameter("DOOR_jointFriction", 0, 100, 10)
torqueId = p.addUserDebugParameter("DOOR_joint torque", -20, 20, -9)
numJoints = p.getNumJoints(robotId)
print("NumJoints: {}".format(numJoints))
for j in range(numJoints):
     print("{} - {}".format(p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

speedId = p.addUserDebugParameter("R2D2_speed", 0, 40, 5)
torqueId = p.addUserDebugParameter("R2D2_force", 0, 40, 5)
try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
        speed = p.readUserDebugParameter(speedId)
        torque = p.readUserDebugParameter(torqueId)

                
        frictionForce = p.readUserDebugParameter(frictionId)
        jointTorque = p.readUserDebugParameter(torqueId)
        #set the joint friction
        p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
        #apply a joint torque
except KeyboardInterrupt:
      pass
	
p.disconnect()    