import yaml


# List of paths
DOBOTICS_MAIN_PATH      = 'src/dobotics_main'
ROBOT_CONFIG_YAML_PATH  = 'src/dobotics_main/config/robot/robot_config.yaml'
SIM_CONFIG_YAML_PATH    = 'src/dobotics_main/config/simulation/simulation_config.yaml'
ROBOT_DESCRIPTION_PATH  = 'src/dobotics_main/robot_description'
ROBOTIS_OP3_DESC_PATH   = 'src/dobotics_main/robot_description/robotis_op3/urdf/robotis_op3.urdf'


# List of available robot options
ROBOTIS_OP3_ENUM            = 0
ROBOTIS_TURTLEBOT3_ENUM     = 1
BOSTON_DYNAMICS_SPOT_ENUM   = 2


# List of loaded .yaml
ROBOT_CONFIG_YAML   = yaml.load(open(ROBOT_CONFIG_YAML_PATH, 'r'), Loader=yaml.FullLoader)
SIM_CONFIG_YAML     = yaml.load(open(SIM_CONFIG_YAML_PATH, 'r'), Loader=yaml.FullLoader)


# Joint enumerator
def GET_JOINT_ENUM(robot_enum:int, joint_name:str):
    return ROBOT_CONFIG_YAML['robot_info'][robot_enum]['joints_enumerator'][joint_name]