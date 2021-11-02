import gym
from gym import error, spaces, utils
import pybullet as p
import pybullet_data
import math
import os
import time
basetime = time.time()
class RoBots(gym.Env):

  def __init__(self):
    self.physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    self.reward = 0
    self.angle = 0
    self.done = 0

  def reset(self):
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(0.01)
    planeId = p.loadURDF("plane.urdf")
    cubeStartPos = [0, 0, 0.001]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    path = os.path.abspath(os.path.dirname(__file__))
    self.botId = p.loadURDF(os.path.join(path, "robot.urdf"), cubeStartPos, cubeStartOrientation)
    self.angle = 0
    self.reward = 0
    self.done = 0

  def step(self, action):
    p.setJointMotorControl2(self.botId,jointIndex=0,controlMode = p.TORQUE_CONTROL , force = action)
    p.setJointMotorControl2(self.botId,jointIndex=1,controlMode = p.TORQUE_CONTROL , force = action)
    p.stepSimulation()
    angle = math.degrees(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.botId)[1])[0])
    # reward = math.cos(angle)
    present_time = time.time()
    passed_time = present_time - basetime
    if(passed_time<=20):
      self.done = 0
    else:
      self.done = 1
    #return observation , reward , done
      return angle , -math.cos(angle) , self.done



  def render(self, mode='human'):
    p.stepSimulation()
    time.sleep(0.01)


# env1 = RoBots()
# env1.render()
# env1.reset()
# while(env1.done!=1):
#   env1.step(action=0.1)
