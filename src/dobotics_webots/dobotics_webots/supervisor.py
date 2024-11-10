from ament_index_python.packages import get_package_share_directory
import json
import rclpy
import std_msgs.msg as std_msgs
import controller as webots

class SupervisorDriver:

    def init(self, webots_node, properties) -> None:
        self.parameter_json = json.load(open(get_package_share_directory('dobotics_webots') + f'/{properties['parameterFile']}', 'r'))
        self.parameters     = self.parameter_json['params']
        self.sim_name       = self.parameter_json['name']

        self.robot: webots.Robot            = webots_node.robot
        self.timestep                       = int(self.robot.getBasicTimeStep())
        self.sim_time_ms                    = 0
        self.supervisor: webots.Supervisor  = webots.Supervisor()
        self.sim_nodes: list[webots.Node]   = [self.supervisor.getFromDef(param['defName']) for param in self.parameters]
        self.sim_node_num                   = len(self.sim_nodes)
        
        rclpy.init(args = None)
        self.node = rclpy.create_node(self.sim_name + '_SupervisorNode')

        self.reset_sub = self.node.create_subscription(
            msg_type    = std_msgs.Empty,
            topic       = f'{self.sim_name}/reset',
            callback    = self.resetCallback,
            qos_profile = 1000
        )


    def resetCallback(self, msg: std_msgs.Empty) -> None:
        for i in range(self.sim_node_num):
            self.sim_nodes[i].getField('translation').setSFVec3f(self.parameters[i]['translation'])
            self.sim_nodes[i].getField('rotation').setSFRotation(self.parameters[i]['rotation'])

        self.node.get_logger().info('The simulation is reset!')

    
    def step(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.sim_time_ms += self.timestep
