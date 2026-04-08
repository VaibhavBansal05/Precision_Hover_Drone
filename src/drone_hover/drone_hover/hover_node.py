import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

class HoverController(Node):
    def __init__(self):
        super().__init__('hover_controller')
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar/scan', self.lidar_cb, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.arm_pub = self.create_publisher(Bool, '/drone/enable', 10)
        
        self.current_z = 0.0
        self.marker_visible = False
        self.error_x = 0.0
        self.error_y = 0.0
        
        self.target_z = 1.0
        self.kp_z = 0.5
        self.kp_xy = 0.002 
        
        # --- NEW OPENCV 4.7+ ARUCO API ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Hover Controller Active! Pushing flight commands...")

    def lidar_cb(self, msg):
        if len(msg.ranges) > 0:
            distance = msg.ranges[len(msg.ranges) // 2]
            if distance != float('inf'):
                self.current_z = distance

    def image_cb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = cv_image.shape
        
        # --- NEW OPENCV 4.7+ ARUCO API ---
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        if ids is not None:
            self.marker_visible = True
            c = corners[0][0]
            marker_x = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4
            marker_y = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4
            self.error_x = (width / 2) - marker_x
            self.error_y = (height / 2) - marker_y
            
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            cv2.imshow("Drone Downward Camera", cv_image)
            cv2.waitKey(1)
        else:
            self.marker_visible = False

    def control_loop(self):
        # 1. Continuously assert the Arming command (Heartbeat)
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)

        # 2. Flight Control
        cmd = Twist()
        cmd.linear.z = max(min(self.kp_z * (self.target_z - self.current_z), 0.5), -0.5) 
        
        if self.marker_visible and self.current_z > 0.3:
            cmd.linear.x = max(min(self.kp_xy * self.error_y, 0.3), -0.3)
            cmd.linear.y = max(min(self.kp_xy * self.error_x, 0.3), -0.3)
            
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = HoverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
