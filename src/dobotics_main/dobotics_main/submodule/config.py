import yaml

# List of paths
DOBOTICS_MAIN_PATH      = 'src/dobotics_main'
ROBOT_CONFIG_YAML_PATH  = 'src/dobotics_main/config/robot/robot_config.yaml'
SCENE_SELECT_YAML_PATH  = 'src/dobotics_main/config/simulation/scene_select.yaml'
ROBOT_DESCRIPTION_PATH  = 'src/dobotics_main/robot_description'
ROBOTIS_OP3_DESC_PATH   = 'src/dobotics_main/robot_description/robotis_op3/urdf/robotis_op3.urdf'

# List of loaded .yaml
ROBOT_CONFIG_YAML   = yaml.load(open(ROBOT_CONFIG_YAML_PATH, 'r'), Loader=yaml.FullLoader)
SCENE_SELECT_YAML   = yaml.load(open(SCENE_SELECT_YAML_PATH, 'r'), Loader=yaml.FullLoader)