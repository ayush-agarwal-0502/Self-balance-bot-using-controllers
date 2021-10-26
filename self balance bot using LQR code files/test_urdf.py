import numpy as np
import pybullet as p
import pybullet_data
import math
import time

id = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)

robot = p.loadURDF("../urdf/self_balance.urdf" , [0,0,0.2])

num = p.getNumJoints(robot)
for i in range(num):
    info = p.getJointInfo(robot, i)
    print(info)

print("----------------------------------------------------------------------------------------------------------------")
com = p.getDynamicsInfo(robot, -1)[3][2]
com += p.getBasePositionAndOrientation(robot)[0][2] 
print("Centre of mass - ", com)
while(True):
    p.setJointMotorControl2(robot, 0, p.VELOCITY_CONTROL, targetVelocity = 100)
    p.setJointMotorControl2(robot, 1, p.VELOCITY_CONTROL, targetVelocity = -100)
    p.stepSimulation()
    time.sleep(0.01)