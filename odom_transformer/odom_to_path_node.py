import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped

class OdometryToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path_node')
        print("pubing_start")
        
        self.topic_out = '/drone0/odom_base'
        self.path_out = '/drone0/path_base'
        self.parent_frame = 'drone0/map'
        
        self.do_path = True
        
        if not self.do_path:
            return
        
        self.subscription = self.create_subscription(
            Odometry,
            self.topic_out,
            self.odometry_callback,
            10)
        
        #for path
        self.path_publisher = self.create_publisher(Path, self.path_out, 10)
        self.path = Path()
        self.path.header.frame_id = self.parent_frame


    def odometry_callback(self, msg):
        self.getpath_from_odom(msg)
        
    def getpath_from_odom(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)

def main(args=None):
    rclpy.init(args=args)
    odom_to_path = OdometryToPath()
    if odom_to_path.do_path: rclpy.spin(odom_to_path)
    odom_to_path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("pubing_start")
    main()