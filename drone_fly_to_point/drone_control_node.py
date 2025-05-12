import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Twist

class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')
        
        # publishers
        self.local_position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.local_velocity_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # timers
        self.create_timer(1.0, self.publish_position)
        self.create_timer(1.0, self.publish_velocity)

        self.current_state = State()
    
    def state_callback(self, msg):
        self.current_state = msg

    def publish_position(self):
        position_msg = PoseStamped()
        position_msg.pose.position.x = 5.0
        position_msg.pose.position.y = 5.0
        position_msg.pose.position.z = 15.0
        self.local_position_pub.publish(position_msg)

    def publish_velocity(self):
        velocity_msg = Twist()
        velocity_msg.linear.x = 2.0
        velocity_msg.linear.z = 2.0
        self.local_velocity_pub.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
