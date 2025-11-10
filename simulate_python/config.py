ROBOT = "adam_u" # Robot name, "adam_u", "adam_lite", "adam_sp" 
ROBOT_SCENE = "../pnd_robots/" + ROBOT + "/scene.xml" # Robot scene

# For ROS2
# SDK_TYPE="ROS2" # "ROS2" or "DDS"
# DOMAIN_ID = 2 # Domain id

# For DDS
SDK_TYPE="DDS" # "ROS2" or "DDS"
DOMAIN_ID = 1 # Domain id

INTERFACE = "lo" # Interface 

USE_JOYSTICK = 1 # Simulate PND WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = False # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = False # Virtual spring band, used for lifting adam

SIMULATE_DT = 0.001  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.01  # 100 fps for viewer
