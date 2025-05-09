import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

class VelocityEstimator(Node):
    def __init__(self):
        super().__init__('velocity_estimator')

        self.declare_parameter('fx', 500.0)
        self.declare_parameter('fy', 500.0)
        self.declare_parameter('fps', 5.0)

        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.fps = self.get_parameter('fps').value

        self.altitude = None
        self.prev_frame = None
        self.bridge = CvBridge()

        self.subscription_image = self.create_subscription(
            Image, 'a15/camera/back/image_raw', self.image_callback, 10)

        self.subscription_altitude = self.create_subscription(
            Float32, '/a15/dvl/altitude', self.altitude_callback, 10)

        self.velocity_pub = self.create_publisher(Vector3, '/a15/flow_velocity', 10)

        self.get_logger().info("Velocity Estimator Node Initialized.")

    def altitude_callback(self, msg):
        self.altitude = msg.data

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.prev_frame is None:
            self.prev_frame = frame
            return

        # Compute optical flow
        flow = self.compute_optical_flow(self.prev_frame, frame)
 
        # Compute median flow
        dx, dy = self.compute_median_flow(flow)

        vx, vy = dx, dy

        # If altitude is available, compute metric velocity
        if self.altitude is not None:
            vx, vy = self.compute_metric_velocity(dx, dy)

        vel_msg = Vector3()
        vel_msg.x = vx
        vel_msg.y = vy
        vel_msg.z = 0.0

        self.velocity_pub.publish(vel_msg)

        self.prev_frame = frame

    def compute_median_flow(self, flow):
        dx = np.median(flow[..., 0])
        dy = np.median(flow[..., 1])
        return dx, dy
    
    def compute_metric_velocity(self, dx, dy):
        dt = 1.0 / self.fps
        vx_mps = (dx * self.altitude) / (self.fx * dt)
        vy_mps = (dy * self.altitude) / (self.fy * dt)
        return vx_mps, vy_mps

    def compute_optical_flow(self, prev_img, curr_img):
        prev_gray = cv2.cvtColor(prev_img, cv2.COLOR_BGR2GRAY)
        curr_gray = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)

        flow = cv2.calcOpticalFlowFarneback(
            prev_gray, curr_gray, None,
            pyr_scale=0.5, levels=3, winsize=15,
            iterations=3, poly_n=5, poly_sigma=1.2,
            flags=cv2.OPTFLOW_FARNEBACK_GAUSSIAN
        )
        return flow

def main(args=None):
    rclpy.init(args=args)
    node = VelocityEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
