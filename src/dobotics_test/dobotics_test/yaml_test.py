import yaml

def main(args=None):
    # Define the path
    YAML_FILE_PATH = 'src/dobotics_main/config/robot/robot_config.yaml'
    
    # Get the .yaml file
    with open(YAML_FILE_PATH, 'r') as file:
        yaml_file = yaml.load(file, Loader=yaml.FullLoader)
    for robot in yaml_file['robot']:
        print(robot['name'])

if __name__ == '__main__':
    main()