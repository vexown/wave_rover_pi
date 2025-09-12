#!/usr/bin/env python3
"""
Simple Visual Odometry node for ROS2

Important notes:
 - This implementation is a *very simple* monocular (single camera) visual odometry example.
 - Scale estimation is a placeholder and must be replaced with a proper method
   (e.g. stereo, depth sensor, known landmark sizes, or scale priors).
 - Camera intrinsics (focal length and principal point) should be calibrated
   for accurate motion recovery.
 - Use better outlier rejection and more robust matching for production.
"""

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for creating ROS 2 nodes
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # QoS utilities
from sensor_msgs.msg import Image  # ROS message type for images
from nav_msgs.msg import Odometry  # ROS message type for odometry
from geometry_msgs.msg import TransformStamped  # Message for tf transforms
from tf2_ros import TransformBroadcaster  # Utility to broadcast tf transforms
from cv_bridge import CvBridge  # Convert between ROS Image and OpenCV images
import cv2  # OpenCV for computer vision routines
import numpy as np  # Numerical operations (arrays)
import math  # Math utilities (trig, etc.)


class SimpleVisualOdometry(Node):
    """
    SimpleVisualOdometry ROS2 node implementing a basic monocular visual odometry
    pipeline using ORB (Oriented FAST and Rotated BRIEF) features and the essential matrix.
    - ORB is a feature detector and descriptor extractor from OpenCV used to identify and describe 
    keypoints in images. These keypoints are then matched between consecutive frames to estimate motion.
    - The essential matrix is a compact rule (a 3x3 matrix) that describes the geometric relationship 
    between two views/frames taken by the same calibrated camera — it tells how points move between the 
    images because the camera rotated and moved, and from it you can recover the camera's rotation and 
    the direction of translation (but not the actual distance).

    This class subscribes to a camera image topic, extracts ORB features, matches them to features from 
    the previous frame, estimates the essential matrix, and recovers the relative camera motion
    (R = rotation, t = translation direction). A "pose" here means the robot's position (x, y)
    and orientation (yaw) in the world. The recovered motion between frames is integrated over time to 
    build a running pose estimate. This estimate is then published as a nav_msgs/Odometry message and
    broadcast as a TF transform for visualization and use by other ROS2 components.
    """

    def __init__(self):
        """
        Initialize the visual odometry node.

        - sets up ROS publishers / subscribers
        - initializes OpenCV feature detectors/matchers
        - prepares robot pose state and camera intrinsics
        """
        super().__init__('simple_visual_odometry')  # Initialize the ROS2 Node with a name

        # CvBridge instance to convert between ROS Image and OpenCV images
        self.bridge = CvBridge()  # Used in image_callback to convert msg -> cv image

        # Create a QoSProfile matching our camera publisher (best-effort)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Best-effort for cameras
            history=HistoryPolicy.KEEP_LAST,  # Keep only the last image
            depth=1  # Depth of 1 (take single most-recent frame, others are dropped)
        )

        # Subscribe to the raw camera image topic
        # - message type: sensor_msgs/Image
        # - topic name: '/camera/image_raw'
        # - callback: self.image_callback
        # - qos profile: sensor_qos (above)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            sensor_qos)

        # Publisher for odometry messages
        # - message type: nav_msgs/Odometry
        # - topic: '/visual_odom'
        # - queue size: 10
        self.odom_pub = self.create_publisher(Odometry, '/visual_odom', 10)
        
        # TransformBroadcaster sends the camera/robot's pose as a transform (position + orientation) 
        # to the ROS2 tf2 system. Other nodes (like RViz) use this to know where the robot or camera 
        # is in the world frame, so they can correctly display its movement and relate sensor data 
        # to the map or other coordinate frames.
        self.tf_broadcaster = TransformBroadcaster(self)

        # Variables to keep the previous frame (grayscale image), along with its keypoints and descriptors.
        # - Keypoints are "interesting spots" in the image, like corners or unique texture patterns, 
        #   that can be reliably found again in later frames. (WHERE they are)
        # - Descriptors are small numeric fingerprints calculated for each keypoint that describe the 
        #   local image area around it. They let us match the same physical points between frames by 
        #   comparing these fingerprints. (WHAT they look like)
        self.prev_gray = None  # Previous grayscale image (numpy array)
        self.prev_kp = None  # Previous ORB keypoints (list of cv2.KeyPoint)
        self.prev_des = None  # Previous descriptors (numpy array)

        # Robot pose state (2D pose with yaw)
        self.x = 0.0  # x position in odom frame (meters)
        self.y = 0.0  # y position in odom frame (meters)
        self.theta = 0.0  # yaw angle (radians)

        # ORB feature detector
        # The 'nfeatures' parameter sets the maximum number of keypoints ORB will detect in each image frame. 
        # - More features → more keypoints to match → improves robustness of motion estimation
        #   because even if some matches are wrong, you still have enough good matches
        #   to reliably compute the essential matrix and recover the pose.
        # - BUT more features also → more computation and more time spent on matching,
        #   which can overload your CPU and increase latency, especially on small
        #   embedded boards like a Raspberry Pi or microcontroller-based systems.
        #
        # Typical ranges for nfeatures:
        #   * 100–300 → very lightweight and fast, but less accurate in complex or low-texture scenes
        #   * 300–800 → good balance between speed and accuracy (500 is a common default)
        #   * 800–2000+ → very robust but CPU heavy; useful for high-resolution images or offline processing where speed is less critical.
        self.orb = cv2.ORB_create(nfeatures=500)

        # Brute-force matcher with Hamming norm for ORB (binary descriptors)
        # crossCheck=True means matches are symmetric (faster but stricter)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Simplified camera intrinsics (should be replaced by real calibration)
        # focal length in pixels (approximate)
        self.focal_length = 800
        # principal point (cx, cy) - image center assumption
        self.pp = (400, 300)

        # Timer to periodically publish the static transform even if no motion
        # create_timer(period, callback) where period is in seconds
        self.timer = self.create_timer(0.1, self.publish_static_transform)

        # Informational log indicating the node has started
        self.get_logger().info('Simple Visual Odometry started')

    def image_callback(self, msg):
        """
        Callback invoked each time an Image message arrives on '/camera/image_raw'.

        Steps performed:
        1. Convert ROS Image to OpenCV BGR image via CvBridge.
        2. Convert to grayscale for feature detection.
        3. If we have a previous frame, detect ORB features and match them to
           the previous descriptors.
        4. Use matched points to estimate the essential matrix and recover pose
           (R, t) between frames.
        5. Integrate the relative pose into the node's x, y, theta state.
        6. Publish odometry and broadcast transforms.

        Note: This function intentionally contains simplified/placeholder
        approaches (scale, outlier filtering) and should be improved for real
        deployments.
        """
        try:
            # Convert incoming ROS Image message to OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # May raise CvBridgeError

            # Convert BGR image to grayscale for feature detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Only attempt to compute motion if we have a previous frame
            if self.prev_gray is not None:
                # Detect ORB keypoints and compute descriptors in current frame
                kp, des = self.orb.detectAndCompute(gray, None)

                # Ensure we have descriptors in both frames and enough matches
                # prev_des: descriptors from the previous frame
                # des: descriptors from the current frame
                if self.prev_des is not None and des is not None and len(des) > 10:
                    # Match descriptors between previous and current frames
                    # matches is a list of DMatch objects
                    matches = self.matcher.match(self.prev_des, des)

                    # Sort matches by descriptor distance (lower is better)
                    matches = sorted(matches, key=lambda x: x.distance)

                    # Keep only a limited number of good matches (top N)
                    good_matches = matches[:min(50, len(matches))]

                    # Continue only if we have a minimum number of good matches
                    if len(good_matches) > 10:
                        # Extract the matched keypoint coordinates for both frames
                        # prev_pts: points in previous frame corresponding to queryIdx
                        prev_pts = np.float32([self.prev_kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                        # curr_pts: points in current frame corresponding to trainIdx
                        curr_pts = np.float32([kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                        # Estimate essential matrix using RANSAC to be robust to outliers
                        E, mask = cv2.findEssentialMat(curr_pts, prev_pts,
                                                     focal=self.focal_length,
                                                     pp=self.pp,
                                                     method=cv2.RANSAC,
                                                     prob=0.999,
                                                     threshold=1.0)

                        # If a valid essential matrix was found
                        if E is not None:
                            # Recover the relative pose (rotation R and translation t)
                            # between the two frames from the essential matrix.
                            # The function also returns a mask of inliers.
                            _, R, t, mask = cv2.recoverPose(E, curr_pts, prev_pts,
                                                          focal=self.focal_length,
                                                          pp=self.pp)

                            # Simple scale estimation (placeholder)
                            # For monocular VO the scale cannot be recovered without
                            # additional info: stereo/depth/prior/loop closure, etc.
                            scale = 0.1  # TODO: replace with proper scale estimation

                            # Translate the recovered translation vector into robot frame
                            # dx is the lateral component (x in camera frame)
                            dx = scale * t[0, 0]
                            # dz is the forward component (z in camera frame)
                            dz = scale * t[2, 0]

                            # Estimate change in yaw from the rotation matrix.
                            # This is a simple approximation using atan2 on R components.
                            dtheta = math.atan2(R[1, 0], R[0, 0])

                            # Compute current orientation cos/sin once for integration
                            cos_theta = math.cos(self.theta)
                            sin_theta = math.sin(self.theta)

                            # Integrate motion into the robot's odom pose using a
                            # 2D planar approximation where 'dz' is forward and
                            # 'dx' is lateral (right-positive) in the camera frame.
                            # We rotate the camera-frame displacement into the odom
                            # frame using the robot's current yaw (theta).
                            self.x += cos_theta * dz - sin_theta * dx
                            self.y += sin_theta * dz + cos_theta * dx
                            self.theta += dtheta

                            # Publish odometry message using the timestamp of the image
                            self.publish_odometry(msg.header.stamp)

            # Save the current frame as previous for the next callback
            # .copy() ensures we keep a separate array rather than a view
            self.prev_gray = gray.copy()
            # Also store keypoints and descriptors for matching on next frame
            self.prev_kp, self.prev_des = self.orb.detectAndCompute(gray, None)

        except Exception as e:
            # Log any exceptions so the node does not crash silently
            self.get_logger().error(f'Visual odometry error: {e}')

    def publish_odometry(self, timestamp):
        """
        Create and publish a nav_msgs/Odometry message representing the
        integrated robot pose stored in self.x, self.y, self.theta.

        This function also broadcasts the corresponding tf (odom -> base_link).

        Args:
            timestamp (builtin_interfaces.msg.Time): timestamp to stamp the odom msg
        """
        # Create an Odometry message instance
        odom = Odometry()

        # Stamp the message with the provided timestamp
        odom.header.stamp = timestamp
        # Coordinate frame of the pose (generally 'odom')
        odom.header.frame_id = "odom"
        # Child frame which the pose refers to (usually the robot's base link)
        odom.child_frame_id = "base_link"

        # Fill in the pose position fields from the integrated state
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0  # 2D planar robot assumed

        # Convert yaw angle to quaternion (only z and w are non-zero for yaw-only)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Populate a small, hand-wavy covariance matrix to indicate uncertainty
        # Note: nav_msgs/Odometry.pose.covariance is a 6x6 row-major array.
        odom.pose.covariance[0] = 0.1   # variance on x
        odom.pose.covariance[7] = 0.1   # variance on y
        odom.pose.covariance[35] = 0.1  # variance on yaw (theta)

        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Broadcast the transform for visualization / TF consumers
        self.broadcast_transform(timestamp)

        # Debug log showing current pose (helpful for tuning)
        self.get_logger().debug(f'Published odom: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')

    def broadcast_transform(self, timestamp):
        """
        Broadcast two transforms:
          1) odom -> base_link using the current integrated pose
          2) base_link -> camera_link representing a fixed camera mount

        The camera transform is broadcast as a regular transform here (not a
        true static transform publisher) for simplicity — in production you may
        want to use a dedicated static_transform_publisher or a static TF file.
        """
        # Create a TransformStamped message for odom -> base_link
        t = TransformStamped()

        # Fill header timestamp and frame ids
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Fill translation from integrated pose
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Convert yaw to quaternion for transform rotation
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        # Send the transform over tf2
        self.tf_broadcaster.sendTransform(t)

        # Create and publish the camera transform relative to the robot base
        camera_transform = TransformStamped()
        camera_transform.header.stamp = timestamp
        camera_transform.header.frame_id = 'base_link'
        camera_transform.child_frame_id = 'camera_link'

        # These values assume the camera is mounted slightly forward and above
        # the robot base; update to match your actual mounting geometry.
        camera_transform.transform.translation.x = 0.1  # 10 cm forward
        camera_transform.transform.translation.y = 0.0  # centered laterally
        camera_transform.transform.translation.z = 0.2  # 20 cm above base

        # Camera orientation: identity quaternion (camera pointing forward)
        camera_transform.transform.rotation.x = 0.0
        camera_transform.transform.rotation.y = 0.0
        camera_transform.transform.rotation.z = 0.0
        camera_transform.transform.rotation.w = 1.0

        # Send the camera transform as well
        self.tf_broadcaster.sendTransform(camera_transform)

    def publish_static_transform(self):
        """
        Periodic timer callback to publish the current transforms even when no
        new odometry has been computed. This helps keep TF listeners up-to-date
        and avoids stale transforms if the robot is idle.
        """
        # Get the current ROS time and convert to a builtin_interfaces Time msg
        timestamp = self.get_clock().now().to_msg()
        # Delegate to broadcast_transform which fills and sends both transforms
        self.broadcast_transform(timestamp)


def main(args=None):
    """
    Entry point for the node. Initializes rclpy, creates the node, and spins
    until shutdown (Ctrl-C).
    """
    # Initialize ROS client library
    rclpy.init(args=args)

    # Create the SimpleVisualOdometry node instance
    node = SimpleVisualOdometry()

    try:
        # Keep the node alive to process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Graceful shutdown on Ctrl-C
        node.get_logger().info('Shutting down visual odometry...')
    finally:
        # Destroy the node explicitly and shutdown rclpy
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # If shutdown raised because it was already shutdown, ignore
            pass


if __name__ == '__main__':
    main()
