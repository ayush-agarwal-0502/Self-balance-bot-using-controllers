#########################################################
#SELF BALANCE BOT USING LQR CONTOLLER
#MADE BY AYUSH AGARWAL , ELECTRONICS ENGINEERING
#project made as a solution to problem statment in week 2 of controls dynamics camp , robotics summer camp
##########################################################
# # Introduction

# Description the implementation of Control Systems namely [LQR](#lqr_sec) on our robot.

# #### Importing Libraries

import sys
#import pidcontrol as pid
import numpy as np
import pybullet as p
import math
import time
import pybullet_data
import controlpy


# # LQR Controller

# Here, we are working on LQR controller. We have used control library [[1](#PythonControl)] of python to work on lqr.
# For the lqr controllers, the state-space equations of the system are required.
# The above equation is derived from [[2](#SystemEquation)] .
# Here,
#(M)       mass of the cart                         2 kg
#(m)       mass of the pendulum                     5 kg
#(b)       coefficient of friction for cart         0.5 N/m/sec
#(l)       length to pendulum center of mass        0.1 m
#(I)       mass moment of inertia of the pendulum   0.10500391309238813 kg.m^2
#(F)       force applied to the cart
#(x)       cart position coordinate
#(theta)   pendulum angle from vertical (down)

#We can calculate
#x(t+1)=A*x(t)+B*u
#x(t+1)=k*x(t)

M = 2;
m = 5;
b = 0.5;
I = 0.10500391309238813;
g = 9.8;
l = 0.1;

####################### write your code here #####################################################################################
#Refer to "test_lqr.py" for example code
#Variable : "A" : 2D numpy array
#Variable : "A" : 2D numpy array
#write here
A = (np.array([[0,1,0,0],[0,0,-(m*g)/M,0],[0,0,0,1],[0,0,((m+M)*g)/(M*l),0]]))
B = np.array([[0],[1/M],[0],[-1/(M*l)]])

###################################################################################################################################

# The class for LQR

# Q = np.array([[ 100,   0],[  0, 1000]])
# R = 0.0001
# K,S,e = lqr(A,B,Q,R)
# print(K)
# print(S)
# print(e)

# From [[2](#SystemEquation)] , we get the cost function
# J = int (x^T Q x +u^T R u)dt
# Here, we need to tune Q matrix and R such that the value of "J" is minimum. The lqp function automatically does that for you

class SelfBalanceLQR:
    '''
    Purpose:
    ---
    Class Describing the gains of LQR and various functions
    Functions:
    ---
        __init__ : Called by default to initilize the variables in LQR
        callback : It takes the data from sensors/bots and accordingly predicts the next state and respecive action
        callback_q : It can be used to change the value of gains during execution
        callback_r : It can be used to change the value of gains during execution
    Example initialization and function call:
    ---
    balance=SelfBalanceLQR()
    vel=balance.callback(data)
    '''
    def __init__(self):
        self.xvelMin=-.01# x velocity
        self.xvelMax =0
        self.yMin = -0.01#yaw
        self.yMax = 0
        self.y_ = 0
        #these are the parameters to be tuned
        self.Q = np.array([[ 10, 0 ,0 ,0],[0,0.00000011,0,0],[0,0,2.5,0],[0,0,0,1]])
        self.R = [[1]]
        # self.Q = np.array([[ 10, 0 ,0 ,0],[0,10,0,0],[0,0,100,0],[0,0,0,10]])
        # self.R = [[1]]
        self.K,self.S,self.e = controlpy.synthesis.controller_lqr(A,B,self.Q,self.R)
    def callback(self,data):
        ######################################## write yor code here ######################################################
        # calculate the state using data calulated from bot
        # Variable np_x : 2D array of state to be multiplie with A
        # Variable y : integer representing the state value that is used to calculate error/other states
        #write here
        np_x = data
        y = p.getBasePositionAndOrientation(robot)[1][2]
        u_t=-np.matmul(self.K,np_x)
        #print(np_x,self.K,u_t)
        #print(np_x.shape,self.K.shape,u_t.shape,A.shape,B.shape)
        xvel = (np.matmul(A,np_x)+np.matmul(B,u_t))[1] ##### might have to change this based on your state equation
        #print(xvel)
        ####################################################################################################################

        #storing previous values for evaluation
        if y>self.yMax:
            self.yMax = y
        elif y<self.yMin:
            self.yMin =y
        if xvel>self.xvelMax:
            self.xvelMax=xvel
        elif xvel<self.xvelMin:
            self.xvelMin = xvel

        linear_vel = [xvel,0,0]
        angular_vel=[0,0,0]
        #print("Max vel " + str(self.xvelMax) + " & Min vel " + str(self.xvelMin) + " Max y " + str(self.yMax*180/3.1416) +" & Min y" + str(self.yMin*180/3.1416))
        #print("Velocity "+ str(xvel)+ " & yaw " + str(y))
        self.y_ = y # storing th previous value of state

        return xvel-np_x[1] # returning the difference of current state and next state to compensate error

    def callback_q(self,data):
        q = data.data
        self.Q = np.array([[ q,   0],[  0, 10*q]])
        self.K,self.S,self.e = controlpy.synthesis.controller_lqr(A,B,self.Q,self.R)

    def callback_r(self,data):
        r = data.data
        self.R = r
        self.K,self.S,self.e = controlpy.synthesis.controller_lqr(A,B,self.Q,self.R)

#def synthesizeData(p,robot):
def synthesizeData(robot):
    '''
    Purpose:
    ---
    Calculate the current state(position , velocity , orienation etc.)
    Input Arguments:
    ---
    `robot` :  integer
        object id of bot spawned in pybullet
    Returns:
    ---
    `data` :  1D array
        list of information required for calculation
    Example call:
    ---
    data=synthesizeData(robot)
    '''
    # print("----------------------------------------------------------------------------------------------------------------")
    # print("Dynamic Info of Base : ",p.getDynamicsInfo(robot, -1),end="\n")
    # #0->mass , 3->local inertial pos
    # print("Base position and Orientation : " , p.getBasePositionAndOrientation(robot),end="\n")
    # #1->orientation

    # com = p.getDynamicsInfo(robot, -1)
    # com += p.getBasePositionAndOrientation(robot)[0][2]
    # print("Centre of mass - ", com)

    #information required yaw
    #imu sensor , kp ,ki ,kd
    #set cmd_vel
    ################################################## write our code here ######################################################
    #Varible : data ,1D array that contains all the state variable required to implement LQR
    # For hints refer to statements above and choose an apt state variable
    #write here
    #state matrix will be [x x. theta theta.](and the x is along the x direction)
    data = np.zeros([4,1],dtype=np.int64)
    data[0][0] = p.getBasePositionAndOrientation(robot)[0][0]
    data[1][0] = p.getBaseVelocity(robot)[0][0]
    data[2][0] = (p.getEulerFromQuaternion(p.getBasePositionAndOrientation(robot)[1])[0])
    data[3][0] = p.getBaseVelocity(robot)[1][1]
    #############################################################################################################################
    return data


if __name__ == "__main__":
    '''
    Purpose:
    ---
        Setup the pybullet environment and calculation of state variables and respective action to balance the bot
    '''
    # connecting to pybullet
    id = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)

    robot = p.loadURDF("../urdf/self_balance.urdf" , [0,0,0.2])
    # num = p.getNumJoints(robot)
    # for i in range(num):
    #     info = p.getJointInfo(robot, i)
    #     print(info,end="\n")
    #     link_name = info[12].decode("ascii")
    #     if link_name == "left_wheel": left_wheel = j
    #     if link_name == "right_wheel": wheel_foot = j
    left_joint=0
    right_joint=1
    maxForce = 0
    mode = p.VELOCITY_CONTROL
    p.setJointMotorControl2(robot, left_joint,controlMode=mode, force=maxForce)
    p.setJointMotorControl2(robot, right_joint,controlMode=mode, force=maxForce)


    # p.changeDynamics(robot,left_joint,lateralFriction=0.7,spinningFriction=0.5,rollingFriction=0.5)
    # p.changeDynamics(robot,right_joint,lateralFriction=0.7,spinningFriction=0.5,rollingFriction=0.5)
    balance=SelfBalanceLQR()


    while(True):
        data=synthesizeData(robot)#get data from simulation and bot
        trq=balance.callback(data)#call to function to implement algorithm
        p.setJointMotorControl2(robot, left_joint , p.TORQUE_CONTROL, force = 4*trq)
        p.setJointMotorControl2(robot, right_joint , p.TORQUE_CONTROL, force = -4*trq)
        p.stepSimulation()
        time.sleep(0.01)
