import rclpy
import pybullet as pb
import pybullet_data
import time

from .submodule.dobotics_pybullet_robot_node import DoboticsPybulletOP3
from .submodule.config import *



def main(args=None):
    # Initiate rclpy
    rclpy.init(args=args)

    # Initiate pybullet physics
    physics_client = pb.connect(pb.GUI)
    pb.setGravity(0, 0, -9.8)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    simulation_fps = SIM_CONFIG_YAML['pybullet_simulation_fps']


    # Creating the world scene. Change the scene_number on config/simulation/simulation_config.yaml
    scene_number = SIM_CONFIG_YAML['pybullet_scene_number']

    # SCENE 1: A Robotis OP3 robot on a samurai temple
    if scene_number == 1:
        plane_id    = pb.loadURDF('samurai.urdf')
        start_pos   = [0, 0, 0.27]
        start_orn   = pb.getQuaternionFromEuler([0, 0, 0])
        op3_id      = pb.loadURDF(ROBOTIS_OP3_DESC_PATH, start_pos, start_orn)


    # Simulate
    while rclpy.ok():
        # Simulate the physics per step
        pb.stepSimulation()
        time.sleep(simulation_fps)

        # rclpy spin
        rclpy.spin_once()
        

    # Disconnect from bullet physics server and shutdown rclpy
    pb.disconnect()
    rclpy.shutdown()



if __name__ == '__main__':
    main()