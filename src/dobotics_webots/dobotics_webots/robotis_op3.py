import rclpy
import std_msgs.msg as std_msgs
import controller as webots
from typing import Dict


class RobotisOp3Config:
    joint_name = [
        'Neck',
        'Head',
        'ShoulderR',
        'ArmUpperR',
        'ArmLowerR',
        'ShoulderL',
        'ArmUpperL',
        'ArmLowerL',
        'PelvYR',
        'PelvR',
        'LegUpperR',
        'LegLowerR',
        'AnkleR',
        'FootR',
        'PelvYL',
        'PelvL',
        'LegUpperL',
        'LegLowerL',
        'AnkleL',
        'FootL'
    ]

    joint_sensor_name = [
        'NeckS',
        'HeadS',
        'ShoulderRS',
        'ArmUpperRS',
        'ArmLowerRS',
        'ShoulderLS',
        'ArmUpperLS',
        'ArmLowerLS',
        'PelvYRS',
        'PelvRS',
        'LegUpperRS',
        'LegLowerRS',
        'AnkleRS',
        'FootRS',
        'PelvYLS',
        'PelvLS',
        'LegUpperLS',
        'LegLowerLS',
        'AnkleLS',
        'FootLS'
    ]

    joint_num               = len(joint_name)
    joint_velocity_limit    = 24.5324
    joint_position_min      = -3.14159
    joint_position_max      = 3.14159


class RobotisOp3Driver:

    def init(self, webots_node, properties) -> None:
        self.robot: webots.Robot        = webots_node.robot
        self.robot_name: str            = properties['robotName']
        self.timestep: int              = int(self.robot.getBasicTimeStep())
        self.simulation_time_ms: int    = 0
        self.config                     = RobotisOp3Config()

        self.joint: Dict[str, webots.Motor]                 = {}
        self.joint_sensor: Dict[str, webots.PositionSensor] = {}
        self.joint_velocity_set: bool                       = False
        self.accel: webots.Accelerometer                    = self.robot.getDevice('Accelerometer')
        self.gyro: webots.Gyro                              = self.robot.getDevice('Gyro')
        self.camera: webots.Camera                          = self.robot.getDevice('Camera')

        for joint_name in self.config.joint_name:
            self.joint.update({joint_name: self.robot.getDevice(joint_name)})
            self.joint[joint_name].setPosition(float('inf'))
            self.joint[joint_name].setVelocity(0.0)

        for sensor_name in self.config.joint_sensor_name:
            self.joint_sensor.update({sensor_name: self.robot.getDevice(sensor_name)})
            self.joint_sensor[sensor_name].enable(self.timestep)

        self.accel.enable(self.timestep)
        self.gyro.enable(self.timestep)
        self.camera.enable(self.timestep)

        rclpy.init(args=None)
        self.node = rclpy.create_node(self.robot_name + '_DriverNode')
        
        self.joint_sensor_msg       = std_msgs.Float64MultiArray()
        self.joint_sensor_msg.data  = [0 for __ in range(self.config.joint_num)]
        self.joint_sensor_pub       = self.node.create_publisher(
            msg_type    = std_msgs.Float64MultiArray,
            topic       = f'{self.robot_name}/JointSensor',
            qos_profile = 1000
        )

        self.inertial_msg   = std_msgs.Float64MultiArray()
        self.inertial_pub   = self.node.create_publisher(
            msg_type    = std_msgs.Float64MultiArray,
            topic       = f'{self.robot_name}/Inertial',
            qos_profile = 1000
        )

        self.joint_velocity     = []
        self.joint_velocity_sub = self.node.create_subscription(
            msg_type    = std_msgs.Float64MultiArray,
            topic       = f'{self.robot_name}/JointVelocity',
            callback    = self.jointSubCallback,
            qos_profile = 1000
        )


    def jointVelocitySubCallback(self, msg:std_msgs.Float64MultiArray) -> None:
        self.joint_velocity     = msg.data.tolist().copy()
        self.joint_velocity_set = True


    def update(self) -> None:
        if self.joint_velocity_set:
            self.joint_velocity_set = False
            for i in range(self.config.joint_num):
                self.joint[self.joint_name[i]].setVelocity(self.joint_velocity[i])

        for i in range(self.config.joint_num):
            self.joint_sensor_msg.data[i] = self.joint_sensor[self.config.joint_sensor_name[i]].getValue()

        self.inertial_msg.data = self.accel.getValues() + self.gyro.getValues()

        self.joint_sensor_pub.publish(self.joint_sensor_msg)
        self.inertial_pub.publish(self.inertial_msg)


    def step(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.update()
        self.simulation_time_ms += self.timestep
