from ament_index_python.packages import get_package_share_directory
import json
import rclpy
import std_msgs.msg as std_msgs
import controller as webots


class SupervisorDriver:

    def init(self, webots_node, properties) -> None:
        self.parameter_json = json.load(open(get_package_share_directory('dobotics_webots') + f'/{properties['parameterFile']}', 'r'))
        self.states         = self.parameter_json['states']
        self.sim_name       = self.parameter_json['name']

        self.robot: webots.Robot            = webots_node.robot
        self.timestep                       = int(self.robot.getBasicTimeStep())
        self.sim_time_ms                    = 0
        self.supervisor: webots.Supervisor  = webots.Supervisor()
        
        rclpy.init(args = None)
        self.node = rclpy.create_node(self.sim_name + '_SupervisorNode')

        self.reset_sub = self.node.create_subscription(
            msg_type    = std_msgs.String,
            topic       = f'{self.sim_name}/reset',
            callback    = self.resetCallback,
            qos_profile = 1000
        )


    def resetCallback(self, msg: std_msgs.String) -> None:
        state_key = str(msg.data)

        if state_key in self.states:
            for data in self.states[state_key]:
                node = self.supervisor.getFromDef(data['defName'])
                node.getField('translation').setSFVec3f(data['translation'])
                node.getField('rotation').setSFRotation(data['rotation'])

            self.node.get_logger().info('The simulation is reset!')

        else:
            self.node.get_logger().error('Unknown state name.')

    
    def step(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.sim_time_ms += self.timestep
