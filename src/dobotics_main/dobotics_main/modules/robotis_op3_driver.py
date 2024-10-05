import rclpy
import controller as webots
import dobotics_interfaces.msg as dobotics_interfaces
from .config import robotis_op3_config
from typing import Dict


class RobotisOp3Driver:

    def init(self, webots_node, properties) -> None:
        self.robot: webots.Robot = webots_node.robot
        self.robot_name: str = properties['robotName']
        self.timestep: int = int(self.robot.getBasicTimeStep())
        self.simulation_time_ms: int = 0
        self.joint: Dict[str, webots.Motor] = {}
        self.joint_name: list = robotis_op3_config['joint_name']
        self.joint_num: int = len(self.joint_name)
        self.joint_updated: bool = False
        self.joint_vel_limit: float = robotis_op3_config['velocity_limit']
        self.joint_pos_min: float = robotis_op3_config['position_min']
        self.joint_pos_max: float = robotis_op3_config['position_max']
        self.sensor: Dict[str, webots.PositionSensor] = {}
        self.sensor_name: list = robotis_op3_config['sensor_name']
        self.accel: webots.Accelerometer = self.robot.getDevice('Accelerometer')
        self.gyro: webots.Gyro = self.robot.getDevice('Gyro')
        self.camera: webots.Camera = self.robot.getDevice('Camera')

        for joint_name in self.joint_name:
            self.joint.update({joint_name: self.robot.getDevice(joint_name)})
            self.joint[joint_name].setPosition(float('inf'))
            self.joint[joint_name].setVelocity(0.0)

        for sensor_name in self.sensor_name:
            self.sensor.update({sensor_name: self.robot.getDevice(sensor_name)})
            self.sensor[sensor_name].enable(self.timestep)

        self.accel.enable(self.timestep)
        self.gyro.enable(self.timestep)
        self.camera.enable(self.timestep)

        rclpy.init(args=None)
        self.node = rclpy.create_node(self.robot_name + '_driver')
        
        self.sensor_msg = dobotics_interfaces.RobotisOp3Sensor()
        self.sensor_msg.pos = [0.0 for __ in range(self.joint_num)]
        self.sensor_pub = self.node.create_publisher(
            msg_type=dobotics_interfaces.RobotisOp3Sensor,
            topic=f'{self.robot_name}/sensor',
            qos_profile=1000
        )

        self.inertial_msg = dobotics_interfaces.RobotisOp3Inertial()
        self.inertial_pub = self.node.create_publisher(
            msg_type=dobotics_interfaces.RobotisOp3Inertial,
            topic=f'{self.robot_name}/inertial',
            qos_profile=1000
        )

        self.joint_msg = dobotics_interfaces.RobotisOp3Joint()
        self.joint_sub = self.node.create_subscription(
            msg_type=dobotics_interfaces.RobotisOp3Joint,
            topic=f'{self.robot_name}/joint',
            callback=self.jointSubCallback,
            qos_profile=1000
        )


    def jointSubCallback(self, msg:dobotics_interfaces.RobotisOp3Joint) -> None:
        self.joint_msg.timestamp = msg.timestamp
        self.joint_msg.vel = msg.vel.copy()
        self.joint_updated = True


    def updateJointVelocity(self) -> None:
        if self.joint_updated:
            self.joint_updated = False
            for i in range(self.joint_num):
                self.joint[self.joint_name[i]].setVelocity(self.joint_msg.vel[i])


    def updateSensor(self) -> None:
        self.sensor_msg.timestamp = self.simulation_time_ms
        for i in range(self.joint_num):
            self.sensor_msg.pos[i] = self.sensor[self.sensor_name[i]].getValue()


    def updateInertial(self) -> None:
        self.inertial_msg.timestamp = self.simulation_time_ms
        self.inertial_msg.accel = self.accel.getValues()
        self.inertial_msg.gyro = self.gyro.getValues()


    def publishAll(self) -> None:
        self.sensor_pub.publish(self.sensor_msg)
        self.inertial_pub.publish(self.inertial_msg)


    def step(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)

        self.updateJointVelocity()
        self.updateSensor()
        self.updateInertial()

        self.publishAll()
        self.simulation_time_ms += self.timestep
