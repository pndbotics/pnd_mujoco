import mujoco
import numpy as np
import pygame
import sys
import struct

from pndbotics_sdk_py.core.channel import ChannelSubscriber, ChannelPublisher

from pndbotics_sdk_py.utils.thread import RecurrentThread

from pndbotics_sdk_py.idl.adam_u.msg.dds_ import *
from pndbotics_sdk_py.idl.default import adam_u_msg_dds__LowState_
import config

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HAND_POSE = "rt/handcmd"

NUM_MOTOR_IDL_ADAM_U = 19
NUM_MOTOR_IDL_ADAM_LITE = 23
NUM_MOTOR_IDL_ADAM_SP = 29

class pndSdkBridge:

    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data

        if(config.ROBOT != "adam_lite"):
            self.num_motor = self.mj_model.nu - 24
        else:
            self.num_motor = self.mj_model.nu
        self.dt = self.mj_model.opt.timestep
        self.joystick = None

        # PNDBotics SDK message
        self.low_state = adam_u_msg_dds__LowState_()
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self.low_state_puber.Init()
        self.lowStateThread = RecurrentThread(
            interval=self.dt, target=self.PublishLowState, name="sim_lowstate"
        )
        self.lowStateThread.Start()

        # subscriber hand cmd_
        self.hand_cmd_suber = ChannelSubscriber(TOPIC_HAND_POSE, HandCmd_)
        self.hand_cmd_suber.Init(self.HandCmdHandler, 10)

        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 10)

    def LowCmdHandler(self, msg: LowCmd_):
        if self.mj_data != None:
            for i in range(self.num_motor):
                self.mj_data.ctrl[i] = (
                    msg.motor_cmd[i].tau
                    + msg.motor_cmd[i].kp * 1.5
                    * (msg.motor_cmd[i].q - self.mj_data.sensordata[i])
                    + msg.motor_cmd[i].kd * 2.5
                    * ( 
                        msg.motor_cmd[i].dq
                        - self.mj_data.sensordata[i + self.num_motor]
                    )
                )

    def PublishLowState(self):
        if self.mj_data != None:
            for i in range(self.num_motor):
                self.low_state.motor_state[i].q = self.mj_data.sensordata[i]
                self.low_state.motor_state[i].dq = self.mj_data.sensordata[
                    i + self.num_motor
                ]
                self.low_state.motor_state[i].tau_est = self.mj_data.sensordata[
                    i + 2 * self.num_motor
                ]
            self.low_state_puber.Write(self.low_state)

    def HandCmdHandler(self, msg: HandCmd_):
        if self.mj_data != None:
            fingers_pos = msg.position[0:12]
            
            # 创建 fingers 列表，每个 fingers_pos 的值重复两次
            fingers = [finger for finger in fingers_pos for _ in range(2)]
            
            if config.HANDPOSE_SRC == 0:
                for i in range(self.num_motor, self.num_motor + 24):
                    self.mj_data.ctrl[i] = fingers[i - self.num_motor]
            else:
                # 修改 fingers 数组中的特定值
                fingers[10] = fingers[8] * 0.5
                fingers[8] = 2 * fingers[10]
                fingers[9] = 2 * fingers[10]
                fingers[22] = fingers[20] * 0.5
                fingers[20] = 2 * fingers[22]
                fingers[21] = 2 * fingers[22]
                
                for i in range(self.num_motor, self.num_motor + 24):
                    self.mj_data.ctrl[i] = 1.6 - fingers[i - self.num_motor] * 0.0016
                    
                    if i == self.num_motor + 10:
                        self.mj_data.ctrl[i] = 0.5 - fingers[i - self.num_motor] * 0.001
                    
                    if i in (self.num_motor + 11, self.num_motor + 9, self.num_motor + 8):
                        self.mj_data.ctrl[i] = 1.0 - fingers[i - self.num_motor] * 0.001
                    
                    if i == self.num_motor + 22:
                        self.mj_data.ctrl[i] = 0.5 - fingers[i - self.num_motor] * 0.001
                    
                    if i in (self.num_motor + 23, self.num_motor + 21, self.num_motor + 20):
                        self.mj_data.ctrl[i] = 1.0 - fingers[i - self.num_motor] * 0.001


    def PrintSceneInformation(self):
        print(" ")

        print("<<------------- Link ------------->> ")
        for i in range(self.mj_model.nbody):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, i)
            if name:
                print("link_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Joint ------------->> ")
        for i in range(self.mj_model.njnt):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_JOINT, i)
            if name:
                print("joint_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Actuator ------------->>")
        for i in range(self.mj_model.nu):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_ACTUATOR, i
            )
            if name:
                print("actuator_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Sensor ------------->>")
        index = 0
        for i in range(self.mj_model.nsensor):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            if name:
                print(
                    "sensor_index:",
                    index,
                    ", name:",
                    name,
                    ", dim:",
                    self.mj_model.sensor_dim[i],
                )
            index = index + self.mj_model.sensor_dim[i]
        print(" ")


class ElasticBand:

    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 3.5])
        self.length = 0
        self.enable = True

    def Advance(self, x, dx):
        """
        Args:
          δx: desired position - current position
          dx: current velocity
        """
        δx = self.point - x
        distance = np.linalg.norm(δx)
        direction = δx / distance
        v = np.dot(dx, direction)
        f = (self.stiffness * (distance - self.length) - self.damping * v) * direction
        return f

    def MujuocoKeyCallback(self, key):
        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable