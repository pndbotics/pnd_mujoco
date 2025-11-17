# 介绍

## PND mujoco

`pnd_mujoco` 是基于 `pnd sdk` 和 `mujoco` 开发的仿真器。用户使用 `pnd_ros2` 和 `pnd_sdk_python` 开发的控制程序可以方便地接入该仿真器，实现仿真到实物的开发流程。仓库别基于 python cyclonedds 以及 python ros2 humble实现了两个版本的仿真器

## 目录结构

- `simulate_python`: 基于 pnd_sdk_py 和 mujoco (python) 实现的仿真器
- `pnd_robots`: pnd sdk 支持的机器人 mjcf 描述文件
- `example`: 例程

## 支持的 PND sdk 消息：

**当前版本仅支持底层开发，主要用于控制器的 sim to real 验证**

- `LowCmd`: 电机控制指令
- `LowState`：电机状态

## 消息(DDS idl)类型说明

- PND Adam_u 型号的机器人使用 adam_u idl 实现底层通信

## 相关链接

- [pnd_sdk_python](https://github.com/pndbotics/pnd_sdk_python)
  - [pnd_ros2](https://github.com/pndbotics/pnd_ros2)
  - [PND wiki](https://wiki.pndbotics.com/half_robot/pnd_adam_u_sdk/)
- [mujoco doc](https://mujoco.readthedocs.io/en/stable/overview.html)

# 安装

## c++ 仿真器 (simulate)

### 1. 依赖

```bash
sudo apt install libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev
```

## Python 仿真器 (simulate_python)

### 1. 依赖

#### pnd_sdk_python

```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/pndbotics/pnd_sdk_python.git
cd pnd_sdk_python
pip3 install -e .
```

#### mujoco-python

```bash
pip3 install mujoco
```

#### joystick

```bash
pip3 install pygame
```

### 2. 测试

```bash
cd ./simulate_python
python3 ./pnd_mujoco.py
```

在新终端运行

```bash
python3 example/python/open_arm.py
```

adam_u机器人会打开手臂然后放下。

# 使用

### python 仿真器

python 仿真器的配置文件位于 `/simulate_python/config.py` 中：

```python

ROBOT = "adam_u"

# 机器人仿真仿真场景文件
ROBOT_SCENE = "../pnd_robots/" + ROBOT + "/scene.xml" # Robot scene


# dds domain id，最好与实物(实物上默认为 0)区分开
单独打开ROS2或者DDS以及其对应的ID
# For ROS2
SDK_TYPE="ROS2" # "ROS2" or "DDS"
DOMAIN_ID = 2 # Domain id

# For DDS
SDK_TYPE="DDS" # "ROS2" or "DDS"
DOMAIN_ID = 1 # Domain id

# 网卡名称, 对于仿真建议使用本地回环 "lo"
INTERFACE = "lo" # Interface

# 是否输出机器人连杆、关节、传感器等信息，True 为输出
PRINT_SCENE_INFORMATION = True

USE_JOYSTICK = 1 # Simulate PND WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

# 是否使用虚拟挂带, 1 为启用
# 主要用于模拟 adam 机器人初始化挂起的过程
ENABLE_ELASTIC_BAND = False

# 仿真步长 单位(s)
# 为保证仿真的可靠性，需要大于 viewer.sync() 渲染一次所需要的时间
SIMULATE_DT = 0.003

# 可视化界面的运行步长，0.02 对应 50fps/s
VIEWER_DT = 0.02
```

### 人形机器人虚拟挂带

考虑到人形机器人不便于从平地上启动并进行调试，在仿真中设计了一个虚拟挂带，用于模拟人形机器人的吊起和放下。设置 `enable_elastic_band/ENABLE_ELASTIC_BAND = 1` 可以启用虚拟挂带。加载机器人后，按 `9` 启用或松开挂带，按 `7` 放下机器人，按 `8` 吊起机器人。

## sim to real

`example` 文件夹下提供了使用不同接口实现, 这些例子简演示了如何使用 PND 提供的接口实现仿真到实物的实现。下面是每个文件夹名称的解释：

- `cpp`: 基于 `C++`, 使用 `pnd_sdk2` 接口
- `python`: 基于 `python`，使用 `pnd_sdk_python` 接口
- `ros2`: 基于`ros2`，使用 `pnd_ros2` 接口

### pnd_sdk_python

1. 运行：

```bash
python3 ./open_arm.py # 控制仿真中的机器人
python3 ./open_arm.py enp3s0 # 控制机器人实物，其中 enp3s0 为机器人所连接的网卡名称
```

2. sim to real

```python
if len(sys.argv) <2:
    // 如果没有输入网卡，使用仿真的 domian id 和 网卡(本地)
    ChannelFactoryInitialize(1, "lo")
else:
    // 否则使用指定的网卡
    ChannelFactoryInitialize(0, sys.argv[1])
```

### pnd_ros2

1. 编译安装
   首先确保已经正确配置好 pnd_ros2 环境，见 [pnd_ros2](https://github.com/pndrobotics/pnd_ros2)。

```bash
source ~/pnd_ros2/setup.sh
cd example/ros2
colcon build
```

2. 运行仿真

```bash
source ~/pnd_ros2/setup_local.sh # 使用本地网卡
export ROS_DOMAIN_ID=1 # 修改domain id 与仿真一致
./install/open_arm/bin/open_arm # 运行
```

3. 运行实物

```bash
source ~/pnd_ros2/setup.sh # 使用机器人连接的网卡
export ROS_DOMAIN_ID=2 # 使用默认的 domain id
./install/open_arm/bin/open_arm # 运行
```

