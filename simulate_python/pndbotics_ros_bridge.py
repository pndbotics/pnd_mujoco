from typing import Any
import traceback

import mujoco
import numpy as np

from pndbotics_sdk_py.idl.adam_u.msg.dds_ import LowCmd_
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.core import Listener

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"

NUM_MOTOR_IDL_ADAM_U = 19
NUM_MOTOR_IDL_ADAM_LITE = 23
NUM_MOTOR_IDL_ADAM_SP = 29

class pndRos2Bridge:

    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data

        self.num_motor = NUM_MOTOR_IDL_ADAM_U
        # ROS2/CycloneDDS subscriber setup
        self.sub_participant = DomainParticipant(2)
        self.sub_topic = Topic(self.sub_participant, TOPIC_LOWCMD, LowCmd_)
        
        print("等待ROS 2消息...")
        self.sub_reader = DataReader(
            self.sub_participant, 
            self.sub_topic,
            None,  # qos 参数，使用默认值
            Listener(on_data_available=self.LowCmdHandler)
        )

    def LowCmdHandler(self, msg: LowCmd_):
        msgs = self.sub_reader.read()
        for msg in msgs:
            print("[Reader] msg:", msg.motor_cmd[6].q)
            if self.mj_data != None:
                for i in range(self.num_motor):
                    self.mj_data.ctrl[i] = (
                        msg.motor_cmd[i].tau
                        + msg.motor_cmd[i].kp
                        * (msg.motor_cmd[i].q - self.mj_data.sensordata[i])
                        + msg.motor_cmd[i].kd
                        * (
                            msg.motor_cmd[i].dq
                            - self.mj_data.sensordata[i + self.num_motor]
                        )
                    )

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
        self.point = np.array([0, 0, 3])
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