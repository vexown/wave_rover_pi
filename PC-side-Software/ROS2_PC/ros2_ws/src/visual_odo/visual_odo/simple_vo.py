#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class SimpleVisualOdometry(Node):
    def __init__(self):
        super().__init__('simple_visual_odometry')
        
        self.bridge = CvBridge()
        
        # QoS Profile matching camera publisher (BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            sensor_qos)
        
        # Publish odometry
        self.odom_pub = self.create_publisher(Odometry, '/visual_odom', 10)
        
        # Transform broadcaster for RViz2
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Visual odometry state
        self.prev_gray = None
        self.prev_kp = None
        self.prev_des = None
        
        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # ORB detector for feature detection
        self.orb = cv2.ORB_create(nfeatures=500)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Camera parameters (simplified - should be calibrated)
        self.focal_length = 800  # Approximate focal length
        self.pp = (400, 300)     # Principal point (image center)
        
        self.get_logger().info('Simple Visual Odometry started')
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            if self.prev_gray is not None:
                # Detect features and compute descriptors
                kp, des = self.orb.detectAndCompute(gray, None)
                
                if self.prev_des is not None and des is not None and len(des) > 10:
                    # Match features between frames
                    matches = self.matcher.match(self.prev_des, des)
                    matches = sorted(matches, key=lambda x: x.distance)
                    
                    # Keep only good matches
                    good_matches = matches[:min(50, len(matches))]
                    
                    if len(good_matches) > 10:
                        # Extract matched point coordinates
                        prev_pts = np.float32([self.prev_kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                        curr_pts = np.float32([kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                        
                        # Estimate motion using essential matrix
                        E, mask = cv2.findEssentialMat(curr_pts, prev_pts, 
                                                     focal=self.focal_length, 
                                                     pp=self.pp,
                                                     method=cv2.RANSAC, 
                                                     prob=0.999, 
                                                     threshold=1.0)
                        
                        if E is not None:
                            # Recover pose from essential matrix
                            _, R, t, mask = cv2.recoverPose(E, curr_pts, prev_pts, 
                                                          focal=self.focal_length, 
                                                          pp=self.pp)
                            
                            # Simple scale estimation (needs improvement)
                            scale = 0.1  # Placeholder - should be estimated properly
                            
                            # Update robot pose
                            dx = scale * t[0, 0]
                            dz = scale * t[2, 0]  # Forward motion
                            dtheta = math.atan2(R[1, 0], R[0, 0])
                            
                            # Transform to robot coordinate frame
                            cos_theta = math.cos(self.theta)
                            sin_theta = math.sin(self.theta)
                            
                            self.x += cos_theta * dz - sin_theta * dx
                            self.y += sin_theta * dz + cos_theta * dx
                            self.theta += dtheta
                            
                            # Publish odometry
                            self.publish_odometry(msg.header.stamp)
            
            # Store current frame data for next iteration
            self.prev_gray = gray.copy()
            self.prev_kp, self.prev_des = self.orb.detectAndCompute(gray, None)
            
        except Exception as e:
            self.get_logger().error(f'Visual odometry error: {e}')
    
    def publish_odometry(self, timestamp):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Covariance (high uncertainty for now)
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y
        odom.pose.covariance[35] = 0.1  # yaw
        
        self.odom_pub.publish(odom)
        
        # Broadcast transform for RViz2
        self.broadcast_transform(timestamp)
        
        self.get_logger().debug(f'Published odom: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')
    
    def broadcast_transform(self, timestamp):
        """Broadcast transform from odom to base_link"""
        t = TransformStamped()
        
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Rotation (quaternion from yaw)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleVisualOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
