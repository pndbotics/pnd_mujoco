# Introduction

## PND mujoco

`pnd_mujoco` is a simulator developed based on `pnd_sdk_python` and `mujoco`. Users can easily integrate the control programs developed with `pnd_sdk_python` and `pnd_ros2` into this simulator, enabling a seamless transition from simulation to physical development. The repository with a structure as follows:

![PND_API](./PND_API.png)

<div align="center">

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![Mujoco](https://img.shields.io/badge/Mujoco-3.2.0-005BBB?logo=google&logoColor=white)](https://mujoco.org/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![CycloneDDS](https://img.shields.io/badge/CycloneDDS-latest-6D28D9)](https://cyclonedds.io/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)

![Updated At](https://img.shields.io/badge/Updated_At-December-64748B?style=flat-square)
![Version](https://img.shields.io/badge/Version-1.0.4-2563EB?style=flat-square)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-059669?style=flat-square)](https://opensource.org/licenses/BSD-3-Clause)
[![Issues](https://img.shields.io/badge/issues-open-EF4444?style=flat-square)](https://github.com/pndbotics/pnd_mujoco/issues)

</div>

## Directory Structure
- `simulate_python`: Simulator implemented based on `pnd_sdk_python` and mujoco 
- `pnd_sdk_python`: MJCF description files for robots supported by `pnd_sdk_python`
- `example`: Example programs

## Supported `pnd_sdk_python` Messages:
**Current version only supports low-level development, mainly used for sim to real verification of controller**
- `LowCmd`: Motor control commands
- `LowState`: Motor state information
- `SportModeState`: Robot position and velocity data
- `IMUState`: Torso IMU state at `rt/secondary_imu` topic

## C++ Simulator Installation (coming soon)

### Dependencies
```bash
sudo apt install libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev
```

## Python Simulator Installation (`simulate_python`)

#### 1. Install pnd_sdk_python
```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/pndbotics/pnd_sdk_python.git
cd pnd_sdk_python
pip3 install -e .
```

#### 2. Install MuJoCo Python
```bash
pip3 install mujoco==3.2.0
```

#### 3. Install joystick support
```bash
pip3 install pygame
```

#### 4. Test Simulation
```bash
cd simulate_python
python3 pnd_mujoco.py
```

Open another terminal:
```bash
python3 example/python/open_arm.py
```

The Adam‑U robot in simulation will lift and lower its arm.


## 🚀 Quick Start

### Python Simulator Configuration  
Configuration file: `simulate_python/config.py`

```python
ROBOT = "adam_u"

# Robot simulation scene file
ROBOT_SCENE = "../pnd_robots/" + ROBOT + "/scene.xml" # Robot scene


# dds domain id. It is recommended to use a different one from the real robot (real robot defaults to 0)
Separate startup for ROS2 or DDS and their corresponding IDs
# For ROS2
SDK_TYPE="ROS2" # "ROS2" or "DDS"
DOMAIN_ID = 2 # Domain id

# For DDS
SDK_TYPE="DDS" # "ROS2" or "DDS"
DOMAIN_ID = 1 # Domain id

# Network interface name. For simulation, it is recommended to use the local loopback "lo"
INTERFACE = "lo" # Interface

# Whether to print robot link, joint, sensor and other information. True means print enabled
PRINT_SCENE_INFORMATION = True

USE_JOYSTICK = 1 # Simulate PND WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

# Whether to use the virtual elastic band. 1 means enabled
# Mainly used to simulate the hanging state during adam robot initialization
ENABLE_ELASTIC_BAND = False

# Simulation timestep (s)
# To ensure simulation stability, the timestep must be larger than the rendering time of one viewer.sync() call
SIMULATE_DT = 0.003

# Visualization timestep. 0.02 corresponds to 50fps
VIEWER_DT = 0.02

```

## 📖 Usage Examples

### Humanoid Virtual Hoist  
To simulate the suspension & release process of humanoid robots:
- Enable in config:
```python
ENABLE_ELASTIC_BAND = 1
```
- Controls:
    - `9` — engage/release hoist  
    - `7` — lower robot  
    - `8` — lift robot  

### Sim to Real

Examples located in `example/`:

| Folder   | Description                            |
| -------- | -------------------------------------- |
| `cpp`    | C++ examples using `pnd_sdk`          |
| `python` | Python examples using `pnd_sdk_python` |
| `ros2`   | ROS2 examples using `pnd_ros2`         |

### Python Example: Sim vs Real

```bash
python3 ./open_arm.py           # simulation
python3 ./open_arm.py enp3s0    # real robot (network interface)

```

Program logic:
```python
if len(sys.argv) < 2:
    ChannelFactoryInitialize(1, "lo")   # simulation
else:
    ChannelFactoryInitialize(1, sys.argv[1])   # real robot
```

## 🔧 API Reference

### ROS2 Example

#### 1. Source pnd_ros2
```bash
cd pnd_ros2
source setup.sh
```

#### 2. Run in simulation or real robot
```bash
source ~/pnd_ros2/setup.sh
export ROS_DOMAIN_ID=2
python3 example/ros2/src/open_arm_ros2.py
```

### Supported PND SDK Messages

- `LowCmd` — motor control command
- `LowState` — motor state feedback

### Message (DDS IDL) Type
The PND Adam-U robot model uses the `adam_u idl` for low-level communication.

## 📄 License

[BSD-3 Clause © PNDbotics](./LICENSE)

## 📖 Reference

- [pnd_sdk_python](https://github.com/pndbotics/pnd_sdk_python)
  - [pnd_ros2](https://github.com/pndbotics/pnd_ros2)
  - [PND wiki](https://wiki.pndbotics.com/half_robot/pnd_adam_u_sdk/)
- [mujoco doc](https://mujoco.readthedocs.io/en/stable/overview.html)

## 🙏 Acknowledgement
- MuJoCo physics engine
- ROS2 community
- DDS community
- PNDbotics SDK ecosystem

## 📞 Contact

- Email: info@pndbotics.com
- Wiki: https://wiki.pndbotics.com  
- SDK: https://github.com/pndbotics/pnd_sdk_python  
- Issues: https://github.com/pndbotics/pnd_mujoco/issues

## 📜 Version Log

| Version | Date       | Updates                                                                              |
| ------- | ---------- | ------------------------------------------------------------------------------------ |
| v1.2.0  | 2026-01-22 | DDS refactoring version |
| v1.0.4  | 2025-12-09 | Update xbox axis & Add lo ro home in switch|
| v1.0.3  | 2025-11-20 | Get state in ros2 |
| v1.0.2  | 2025-11-17 | Change the meshes and add Columns mass to 88 kg |
| v1.0.1  | 2025-11-11 | Add ros2 example & Support hands |
| v1.0.0  | 2025-11-10 | Initial release|

---

<div align="center">

[![Website](https://img.shields.io/badge/Website-PNDbotics-black?)](https://www.pndbotics.com)
[![Twitter](https://img.shields.io/badge/Twitter-@PNDbotics-1DA1F2?logo=twitter&logoColor=white)](https://x.com/PNDbotics)
[![YouTube](https://img.shields.io/badge/YouTube-ff0000?style=flat&logo=youtube&logoColor=white)](https://www.youtube.com/@PNDbotics)
[![Bilibili](https://img.shields.io/badge/-bilibili-ff69b4?style=flat&labelColor=ff69b4&logo=bilibili&logoColor=white)](https://space.bilibili.com/303744535)

**⭐ Star us on GitHub — it helps!**

</div>
