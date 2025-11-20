import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from pndbotics_sdk_py.core.channel import ChannelFactoryInitialize
from pndbotics_sdk_py_bridge import pndSdkBridge, ElasticBand
from pndbotics_ros_bridge import pndRos2Bridge

import config


locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)
azimuth = mj_model.vis.global_.azimuth
elevation = mj_model.vis.global_.elevation

if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()

    band_attached_link = mj_model.body("torso").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

viewer.cam.azimuth = azimuth
viewer.cam.elevation = elevation
viewer.cam.distance = 3.0
viewer.cam.lookat[:] = [0, 0, 1]

mj_model.opt.timestep = config.SIMULATE_DT

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model

    
    if config.SDK_TYPE == "ROS2":
        # pnd = pndSdkBridge(mj_model, mj_data)
        ChannelFactoryInitialize(config.DOMAIN_ID)
        pnd = pndRos2Bridge(mj_model, mj_data)
    elif config.SDK_TYPE == "DDS":
        # use python sdk example 
        # ChannelFactoryInitialize(config.DOMAIN_ID)
        
        # use pnd_mujoco open_arm.py example
        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
        pnd = pndSdkBridge(mj_model, mj_data)

    # print(pnd.low_cmd_suber)

    # if config.USE_JOYSTICK:
    #     pnd.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        pnd.PrintSceneInformation()

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
