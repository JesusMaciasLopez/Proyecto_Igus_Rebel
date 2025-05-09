import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class PoseMoveit(Node):
    def __init__(self):                                                     
        super().__init__('nodo_cinematica_inv')
        AnchoBanda = QoSProfile(depth=10)
        AnchoBanda.reliability = QoSReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(
            PoseStamped,'/pose_target',AnchoBanda
        )
        
        self.subscripcion = self.create_subscription(
            Pose, 'cinematica_directa', self.pose_a_moveit, AnchoBanda
        )

        self.modo = 1

    def pose_a_moveit(self,msg):
        pos = PoseStamped()
        pos.header.frame_id = 'base_link'
        pos.header.stamp = self.get_clock().now().to_msg()
        if self.modo == 1:
            pos.pose = msg
        self.publisher.publish(pos)

def main(args=None):
    try:
        rclpy.init(args=args)
        Pose = PoseMoveit()
        rclpy.spin(Pose)
    except KeyboardInterrupt:
        print('...exit node')
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()

