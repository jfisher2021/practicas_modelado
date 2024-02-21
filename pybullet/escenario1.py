import pybullet as p
import time
import pybullet_data

rampa = "urdf/escenario1.urdf"
barra = "urdf/barrita.urdf"
meta = "urdf/final.urdf"

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")
terreneitor = p.loadURDF("husky/husky.urdf")

startPos = [0,0,1]

startOrientation = p.getQuaternionFromEuler([0,0,0])

rampas = p.loadURDF(rampa, startPos, startOrientation)
barraita = p.loadURDF(barra,startPos, startOrientation)
final = p.loadURDF(meta, startPos, startOrientation)

numJoints = p.getNumJoints(rampas)
print("NumJoints: {}".format(numJoints))
for j in range(numJoints):
     print("{} - {}".format(p.getJointInfo(rampas,j)[0], p.getJointInfo(rampas,j)[1].decode("utf-8")))

numJointsROBOT = p.getNumJoints(terreneitor)

print("NumJoints ROBOT: {}".format(numJointsROBOT))
for j in range(numJointsROBOT):
     print("{} - {}".format(p.getJointInfo(terreneitor,j)[0], p.getJointInfo(terreneitor,j)[1].decode("utf-8")))
try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
        
        p.setJointMotorControlArray(terreneitor, [2,3,4,5], p.VELOCITY_CONTROL, targetVelocities=[20,20,20,20])
        
        
except KeyboardInterrupt:
      pass
	
p.disconnect()    