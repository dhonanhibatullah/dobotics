import rclpy
import std_msgs.msg as std_msgs

from .config import *



class DoboticsRobotisOP3Driver:



    def init(self, webots_node, properties):
        # Get webots parameters
        self.__robot        = webots_node.robot
        self.__joint        = {}
        self.__pos_sensor   = {}
        self.__inertial     = {}


        # Get robot devices
        for joint_name in ROBOT_CONFIG_YAML['robot_info'][ROBOTIS_OP3_ENUM]['webots_joints']:
            # Joint actuators
            self.__joint[joint_name] = self.__robot.getDevice(joint_name)
            self.__joint[joint_name].setPosition(float('inf'))
            self.__joint[joint_name].setVelocity(0)
            
            # Joint sensors
            self.__pos_sensor[joint_name + 'S'] = self.__robot.getDevice(joint_name + 'S')
            self.__pos_sensor[joint_name + 'S'].enable(SIM_CONFIG_YAML['webots_basic_timestep'])

        self.__inertial['accel'] = self.__robot.getDevice('Accelerometer')
        self.__inertial['accel'].enable(SIM_CONFIG_YAML['webots_basic_timestep'])
        self.__inertial['gyro']  = self.__robot.getDevice('Gyro')
        self.__inertial['gyro'].enable(SIM_CONFIG_YAML['webots_basic_timestep'])


        # Initiate rclpy node
        rclpy.init(args=None)
        self.node = rclpy.create_node(properties['robotName'] + '_driver')


        # Create ROS2 publishers and subscribers
        self.joint_val_pub  = self.node.create_publisher(std_msgs.Float64MultiArray, properties['robotName'] + '/joint_val', 10)
        self.accel_val_pub  = self.node.create_publisher(std_msgs.Float64MultiArray, properties['robotName'] + '/accel_val', 10)
        self.gyro_val_pub   = self.node.create_publisher(std_msgs.Float64MultiArray, properties['robotName'] + '/gyro_val', 10)


        # Create ROS2 message holders
        self.joint_val_msg  = std_msgs.Float64MultiArray()
        layout              = std_msgs.MultiArrayLayout()
        layout.dim.append(
            std_msgs.MultiArrayDimension(
                label   = 'Joint values are sorted by joint_enumerator in robot_config.yaml',
                size    = 20
            )
        )
        self.joint_val_msg.layout = layout

        self.accel_val_msg  = std_msgs.Float64MultiArray()
        layout              = std_msgs.MultiArrayLayout()
        layout.dim.append(
            std_msgs.MultiArrayDimension(
                label   = 'X, Y, and Z accelerometer values respectively',
                size    = 3
            )
        )
        self.accel_val_msg.layout = layout

        self.gyro_val_msg   = std_msgs.Float64MultiArray()
        layout              = std_msgs.MultiArrayLayout()
        layout.dim.append(
            std_msgs.MultiArrayDimension(
                label   = 'X, Y, and Z gyro values respectively',
                size    = 3
            )
        )
        self.gyro_val_msg.layout = layout



    def sensorDataPublish(self):
        # Assign the joint value based on joint_enumerator order
        joint_val       = [0. for i in range(20)]
        for joint_name in ROBOT_CONFIG_YAML['robot_info'][ROBOTIS_OP3_ENUM]['webots_joints']:
            joint_val[GET_JOINT_ENUM(ROBOTIS_OP3_ENUM, joint_name) - 1] = self.__pos_sensor[joint_name + 'S'].getValue()
        
        self.joint_val_msg.data = joint_val


        # Assign the inertial values
        self.accel_val_msg.data = self.__inertial['accel'].getValues()
        self.gyro_val_msg.data  = self.__inertial['gyro'].getValues()


        # Publish
        self.joint_val_pub.publish(self.joint_val_msg)
        self.accel_val_pub.publish(self.accel_val_msg)
        self.gyro_val_pub.publish(self.gyro_val_msg)



    def step(self):
        # Node spin once and publish
        rclpy.spin_once(self.node, timeout_sec=0)
        self.sensorDataPublish()