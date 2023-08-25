import pybullet as pb
import pybullet_data
import time

from .submodule.dobotics_pybullet_robot_node import DoboticsPybulletRobotNode
from .submodule.dobotics_pybullet_robot_app import DoboticsPybulletRobotApp


def main():
    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    pb.setGravity(0,0,-10)
    planeId = pb.loadURDF("plane.urdf")
    cubeStartPos = [0,0,1]
    cubeStartOrientation = pb.getQuaternionFromEuler([0,0,0])
    boxId = pb.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)

    for i in range (1000):
        pb.stepSimulation()
        time.sleep(1./240.)
    
    cubePos, cubeOrn = pb.getBasePositionAndOrientation(boxId)
    print(cubePos,cubeOrn)
    pb.disconnect()


if __name__ == '__main__':
    main()