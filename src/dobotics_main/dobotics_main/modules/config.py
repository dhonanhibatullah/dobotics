import os
import yaml
from typing import Dict

DOBOTICS_MAIN_CONFIG_PATH = os.path.join(os.getcwd(), 'src', 'dobotics_main', 'config')
ROBOTIS_OP3_YAML_PATH = os.path.join(DOBOTICS_MAIN_CONFIG_PATH, 'robotis_op3.yaml')

robotis_op3_config: Dict = yaml.safe_load(open(ROBOTIS_OP3_YAML_PATH, 'r'))