import pybullet as pb
import pybullet_data
import time

from .submodule.dobotics_pybullet_robot_node import DoboticsPybulletOP3
from .submodule.config import *



def main(args=None):
    # Initiate pybullet physics
    physics_client = pb.connect(pb.GUI)
    pb.setGravity(0, 0, -9.8)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())


    # Creating the world scene. Change the scene_number on config/simulation/scene_select.yaml
    scene_number = SCENE_SELECT_YAML['scene_number']

    # SCENE 1: A Robotis OP3 robot on a samurai temple
    if scene_number == 1:
        plane_id    = pb.loadURDF('samurai.urdf')
        start_pos   = [0, 0, 0.27]
        start_orn   = pb.getQuaternionFromEuler([0, 0, 0])
        op3_id      = pb.loadURDF(ROBOTIS_OP3_DESC_PATH, start_pos, start_orn)


    # Simulate
    for i in range(4000):
        pb.stepSimulation()
        time.sleep(1./240.)
    

    # Disconnect from physics server
    pb.disconnect()



if __name__ == '__main__':
    main()