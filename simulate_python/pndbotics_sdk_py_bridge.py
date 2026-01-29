import mujoco
import numpy as np
import pygame
import sys
import struct

from pndbotics_sdk_py.core.channel import ChannelSubscriber, ChannelPublisher
from pndbotics_sdk_py.utils.thread import RecurrentThread

import config
# Use pnd_adam IDL for all robot types
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import LowCmd_
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import LowState_
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import HandCmd_
from pndbotics_sdk_py.idl.default import pnd_adam_msg_dds__LowCmd_ as LowCmd_default
from pndbotics_sdk_py.idl.default import pnd_adam_msg_dds__LowState_ as LowState_default

TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_HAND_POSE = "rt/handcmd"

MOTOR_SENSOR_NUM = 3

class pndSdkBridge:

    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data

        if(config.ROBOT != "adam_lite"):
            self.num_motor = self.mj_model.nu - 24
        else:
            self.num_motor = self.mj_model.nu
        # print(self.num_motor, " motors detected.")
        # exit()
        self.dim_motor_sensor = MOTOR_SENSOR_NUM * self.num_motor
        self.have_imu_ = False
        self.have_frame_sensor_ = False
        self.dt = self.mj_model.opt.timestep

        self.joystick = None

        # Check sensor
        for i in range(self.dim_motor_sensor, self.mj_model.nsensor):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            if name == "imu_quat":
                self.have_imu_ = True
            if name == "frame_pos":
                self.have_frame_sensor_ = True

        # publisher state
        self.low_state = LowState_default(self.num_motor)
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self.low_state_puber.Init()
        self.lowStateThread = RecurrentThread(
            interval=self.dt, target=self.PublishLowState, name="sim_lowstate"
        )
        self.lowStateThread.Start()

        if config.ROBOT == "adam_u":
            # subscriber hand cmd_
            self.hand_cmd_suber = ChannelSubscriber(TOPIC_HAND_POSE, HandCmd_)
            self.hand_cmd_suber.Init(self.HandCmdHandler, 10)

        # subscriber cmd
        LowCmd_default(self.num_motor)
        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 10)

    def LowCmdHandler(self, msg: LowCmd_):
        if self.mj_data != None:
            for i in range(self.num_motor):
                self.mj_data.ctrl[i] = (
                    msg.motor_cmd[i].tau
                    + msg.motor_cmd[i].kp
                    * (msg.motor_cmd[i].q - self.mj_data.sensordata[i])
                    + msg.motor_cmd[i].kd
                    * (
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
            
            if self.have_frame_sensor_:
                self.low_state.imu_state.quaternion[0] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 0
                ]
                self.low_state.imu_state.quaternion[1] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 1
                ]
                self.low_state.imu_state.quaternion[2] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 2
                ]
                self.low_state.imu_state.quaternion[3] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 3
                ]

                self.low_state.imu_state.gyroscope[0] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 4
                ]
                self.low_state.imu_state.gyroscope[1] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 5
                ]
                self.low_state.imu_state.gyroscope[2] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 6
                ]

                self.low_state.imu_state.accelerometer[0] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 7
                ]
                self.low_state.imu_state.accelerometer[1] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 8
                ]
                self.low_state.imu_state.accelerometer[2] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 9
                ]

            if self.joystick != None:
                pygame.event.get()
                # Buttons
                self.low_state.wireless_remote[0] = self.joystick.get_axis(self.axis_id["LX"])
                self.low_state.wireless_remote[1] = self.joystick.get_axis(self.axis_id["LY"])
                self.low_state.wireless_remote[2] = self.joystick.get_axis(self.axis_id["RX"])
                self.low_state.wireless_remote[3] = self.joystick.get_axis(self.axis_id["RY"])
                self.low_state.wireless_remote[4] = self.joystick.get_axis(self.axis_id["LT"])
                self.low_state.wireless_remote[5] = self.joystick.get_axis(self.axis_id["RT"])

                self.low_state.wireless_remote[6] = self.joystick.get_hat(0)[0]
                self.low_state.wireless_remote[7] = self.joystick.get_hat(0)[1]
                
                self.low_state.wireless_remote[8] = self.joystick.get_button(self.button_id["A"])
                self.low_state.wireless_remote[9] = self.joystick.get_button(self.button_id["B"])
                self.low_state.wireless_remote[10] = self.joystick.get_button(self.button_id["X"])
                self.low_state.wireless_remote[11] = self.joystick.get_button(self.button_id["Y"])
                self.low_state.wireless_remote[12] = self.joystick.get_button(self.button_id["LB"])
                self.low_state.wireless_remote[13] = self.joystick.get_button(self.button_id["RB"])
                self.low_state.wireless_remote[14] = self.joystick.get_button(self.button_id["SELECT"])
                self.low_state.wireless_remote[15] = self.joystick.get_button(self.button_id["START"])
                self.low_state.wireless_remote[16] = self.joystick.get_button(self.button_id["HOME"])
                self.low_state.wireless_remote[17] = self.joystick.get_button(self.button_id["LO"])
                self.low_state.wireless_remote[18] = self.joystick.get_button(self.button_id["RO"])
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


    def SetupJoystick(self, device_id=0, js_type="xbox"):
        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            self.joystick = pygame.joystick.Joystick(device_id)
            self.joystick.init()
        else:
            print("No gamepad detected.")

        if js_type == "xbox":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 3,  # Right stick axis x
                "RY": 4,  # Right stick axis y
                "LT": 2,  # Left trigger
                "RT": 5,  # Right trigger
                "XX": 6,  # Directional pad x
                "YY": 7,  # Directional pad y
            }

            self.button_id = {
                "A": 0,
                "B": 1,
                "X": 2,
                "Y": 3,
                "LB": 4,
                "RB": 5,
                "SELECT": 6,
                "START": 7,
                "HOME": 8,
                "LO": 9,
                "RO": 10,
            }

        elif js_type == "switch":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 2,  # Right stick axis x
                "RY": 3,  # Right stick axis y
                "LT": 5,  # Left trigger
                "RT": 4,  # Right trigger
                "XX": 6,  # Directional pad x
                "YY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 3,
                "Y": 4,
                "B": 1,
                "A": 0,
                "LB": 6,
                "RB": 7,
                "SELECT": 10,
                "START": 11,
                "HOME": 12,
                "LO": 13,
                "RO": 14,
            }
        else:
            print("Unsupported gamepad. ")

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
        self.point = np.array([0, 0, 3.8])
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
