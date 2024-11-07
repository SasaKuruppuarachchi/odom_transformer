import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Vector3
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from scipy.spatial.transform import Rotation as R

class OdometryToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        print("pubing_start")
        
        self.parent_frame = 'drone0/map'
        self.child_frame = 'drone0/odom_base'
        self.odom_topic = '/drone0/odom_base'
        self.path_topic = '/drone0/rot_path'
        
        self.R =  np.array([[0.7071068,  0.0000000, 0.7071068],
                             [0.0000000,  1.0000000,  0.0000000],
                             [-0.7071068,  0.0000000,  0.7071068]]) # Rotation matrix for 180 degrees around the x-axis
        
        self.ext_R = np.array([[0.7071068,  0.0000000, -0.7071068],
                             [0.0000000,  1.0000000,  0.0000000],
                             [0.7071068,  0.0000000,  0.7071068]]) # Rotation matrix for 180 degrees around the x-axis
        
        self.rotate_ori = True
        
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odometry_callback,
            10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        #for path
        self.path_publisher = self.create_publisher(Path, self.path_topic, 10)
        self.path = Path()
        self.path.header.frame_id = self.parent_frame

    def quaternion_multiply(self, q1, q2):
        """Multiplies two quaternions."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
        
    def rotate_covariance(self, covariance, R):
        """Rotates a 6x6 covariance matrix using the rotation matrix R."""
        cov_matrix = np.array(covariance).reshape(6, 6)
        # Rotate the position part (top-left 3x3 submatrix)
        cov_matrix[:3, :3] = R @ cov_matrix[:3, :3] @ R.T
        # Rotate the orientation part (bottom-right 3x3 submatrix)
        cov_matrix[3:6, 3:6] = R @ cov_matrix[3:6, 3:6] @ R.T
        return cov_matrix.flatten().tolist()


    def odometry_callback(self, msg):
        rotated_odometry_msg, tf = self.rotate_odom(msg, self.R, self.ext_R)
        

        # Publish the vehicle odometry message
        self.tf_broadcaster.sendTransform(tf)
        self.path_publisher.publish(self.path)
        
    def getpath_from_odom(self, msg, R_matrix, Ext_matrix):
        transform = TransformStamped()
        pose = PoseStamped()
        
        

        # Publish the transform from drone0/map to odom2
        
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = rotated_position[0]
        transform.transform.translation.y = rotated_position[1]
        transform.transform.translation.z = rotated_position[2]
        transform.transform.rotation.w = msg.pose.pose.orientation.w
        transform.transform.rotation.x = msg.pose.pose.orientation.x
        transform.transform.rotation.y = msg.pose.pose.orientation.y
        transform.transform.rotation.z = msg.pose.pose.orientation.z
        
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)

        
        return odom, transform

def main(args=None):
    rclpy.init(args=args)
    odom_to_path = OdometryToPath()
    rclpy.spin(odom_to_path)
    odom_to_path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("pubing_start")
    main()