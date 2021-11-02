import gym
#import gym_robot
import math
import os
import time
import pybullet as p
import tensorflow
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.deepq.policies import MlpPolicy
from stable_baselines import DQN

env = gym.make('packagename:packagename-v0')

model = DQN(MlpPolicy, env, verbose=1)
model.learn(total_timesteps=250)
model.save("deepq_RL_W3")

del model # remove to demonstrate saving and loading

model = DQN.load("deepq_RL_W3")

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()
