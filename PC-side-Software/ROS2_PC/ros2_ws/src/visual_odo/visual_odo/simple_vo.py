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

        # Brute-Force Matcher for comparing ORB descriptors between two frames.
        #
        # ORB creates "descriptors" for each keypoint — basically small fingerprints
        # (a vector of numbers) that describe the visual pattern around that point.
        # ORB uses binary descriptors, which means each descriptor is just a sequence
        # of 0s and 1s (bits). This makes them very lightweight and fast to compare.
        #
        # To measure how similar two binary descriptors are, we use the Hamming distance:
        # - The Hamming distance counts how many bits are different between two descriptors.
        # - A distance of 0 means the descriptors are identical (perfect match),
        #   while a larger number means they are less similar.
        #
        # crossCheck=True → This requires matches to be mutual (symmetric):
        # - Point A in frame 1 must match point B in frame 2,
        #   AND point B in frame 2 must match point A in frame 1.
        # - This helps filter out bad matches automatically and gives more reliable matches,
        #   but it is stricter and can result in fewer total matches.
        #
        # Summary of the process:
        # 1. ORB detects keypoints in both the current frame and previous frame.
        # 2. Each keypoint gets a binary descriptor.
        # 3. BFMatcher compares every descriptor in the previous frame to every descriptor
        #    in the current frame using Hamming distance (brute-force search).
        # 4. The best matches (lowest distances) are returned.
        #
        # Note: Brute-force matching can be slow if there are many keypoints.
        # If you increase ORB's nfeatures significantly, you may need to switch to
        # a faster approximate matcher like FLANN for real-time performance.
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Simplified camera intrinsics
        #
        # These are approximate values for your camera's internal parameters
        # (a.k.a. intrinsics). They are used by OpenCV's functions like
        # findEssentialMat() and recoverPose() to correctly interpret how
        # points move between frames and estimate motion in 3D space.
        #
        # 1. FOCAL LENGTH (in pixels)
        #    - This represents how "zoomed in" the camera is.
        #    - It tells the math how much the image stretches as objects get closer or further.
        #    - Typical values depend on your camera and image resolution.
        #      For example, 800 is a rough guess for a 640x480 or 800x600 image.
        #    - If this is wrong, the scale and motion direction from visual odometry
        #      will be inaccurate or unstable.
        self.focal_length = 800

        # 2. PRINCIPAL POINT (cx, cy)
        #    - This is the pixel coordinate where the camera's optical axis
        #      (center of the lens) hits the image sensor.
        #    - For many cameras, it's approximately the image center.
        #      Example: for an 800x600 image → pp = (400, 300).
        #    - If slightly off, the math still works, but accuracy drops.
        self.pp = (400, 300)

        # Why you need REAL calibration:
        # --------------------------------
        # These values are only rough guesses! Every real camera has tiny
        # imperfections: the lens might bend the image (distortion), the true
        # focal length might differ slightly, or the principal point may not
        # be exactly at the center.
        #
        # Camera calibration is the process of taking pictures of a known
        # pattern (like a checkerboard) and using OpenCV to measure:
        #    - Focal length (fx, fy)
        #    - Principal point (cx, cy)
        #    - Lens distortion coefficients
        #
        # With real calibration:
        #    - The recovered motion will be much more accurate and stable.
        #    - It allows you to undistort images so straight lines stay straight,
        #      which is critical for precise visual odometry.
        #
        # For now, these approximate values are "good enough" to get something
        # working, but for reliable robotics, always replace them with real
        # calibration data from OpenCV's camera_calibration tutorial.

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
            # Convert incoming ROS Image message to an OpenCV image.
            #
            # ROS2 cameras publish images as sensor_msgs/Image messages,
            # which are just raw pixel data + some metadata like encoding type.
            # cv_bridge is a helper that converts between ROS images and
            # OpenCV images so we can use OpenCV functions.
            #
            # The "bgr8" argument tells CvBridge to output a standard BGR color image:
            #   - 8-bit per channel (0-255 values),
            #   - 3 channels: Blue, Green, Red (OpenCV's default color order).
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # May raise CvBridgeError

            # Convert the BGR color image to a single-channel grayscale image.
            #
            # Why grayscale for feature detection?
            # - Algorithms like ORB, SIFT, and FAST care about intensity changes,
            #   like edges and corners, not about color.
            # - Working in grayscale is:
            #     1. Faster → less data to process (1 channel instead of 3),
            #     2. More reliable → color can vary with lighting but intensity patterns
            #        (e.g., edges) stay consistent.
            #
            # In this step, OpenCV combines the B, G, and R channels using a
            # standard weighted formula:
            #   Gray = 0.299*R + 0.587*G + 0.114*B
            #
            # This results in a clean single-channel image ideal for feature detection.
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

                    # Keep only a limited number of "good" matches (top N).
                    #
                    # ORB was initially configured to detect up to 500 features per frame.
                    # However, when comparing two frames, not all those features will
                    # have reliable matches — many might be weak, noisy, or incorrect.
                    #
                    # After the brute-force matching step, we sort matches by their
                    # distance score (lower distance = better match). By taking only
                    # the *top 50*, we focus on the most reliable correspondences.
                    #
                    # Why detect 500 features but only keep 50?
                    # -------------------------------------------------------------
                    # 1. **High initial pool = higher chance of strong matches:**
                    #    - If you only detected 50 features at the start, you might
                    #      miss good ones entirely if they are in areas with motion blur,
                    #      poor texture, or occlusion.
                    #    - Detecting 500 gives the algorithm a wide net to capture
                    #      all potentially useful details.
                    #
                    # 2. **Filtering later improves robustness:**
                    #    - The Essential Matrix calculation (cv2.findEssentialMat)
                    #      is sensitive to outliers (bad matches).
                    #    - By trimming to the best 50, we reduce the risk of noisy
                    #      points introducing large errors.
                    #
                    # 3. **Performance balance:**
                    #    - Using all 500 matches every frame would be slower and
                    #      might overwhelm the math with redundant or noisy data.
                    #    - 50 strong matches are often *enough* to estimate
                    #      camera motion while keeping computation fast.
                    #
                    # Tuning:
                    # - For slow camera movement and stable scenes, you can even use
                    #   fewer matches (like 30).
                    # - For fast movement or very dynamic environments, you might
                    #   increase this limit to 100 or more to maintain accuracy.
                    good_matches = matches[:min(50, len(matches))]

                    # Continue only if we have a minimum number of good matches
                    if len(good_matches) > 10:
                        # Extract the matched keypoint coordinates for both frames
                        # prev_pts: points in previous frame corresponding to queryIdx
                        prev_pts = np.float32([self.prev_kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                        # curr_pts: points in current frame corresponding to trainIdx
                        curr_pts = np.float32([kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                        # Estimate the Essential Matrix (E) between the current
                        # and previous frames, using RANSAC to handle noisy data.
                        #
                        # Purpose:
                        # --------
                        # Given a set of matched feature points between two images,
                        # this step figures out the relationship between those images
                        # caused by the camera's movement.
                        #
                        # The Essential Matrix encodes the camera motion (rotation + 
                        # translation direction) but NOT the actual distance moved.
                        # 
                        # Why we need this:
                        # - To compute how the camera moved between frames,
                        #   we need to separate true matches (good, consistent points)
                        #   from outliers (bad matches caused by noise, moving objects,
                        #   or incorrect feature matches).
                        #
                        # RANSAC (Random Sample Consensus):
                        # ---------------------------------
                        # - RANSAC is a robust estimation algorithm that tries to
                        #   find a model (here, the Essential Matrix) while ignoring
                        #   bad data.
                        #
                        #   It works like this:
                        #   1. Randomly pick a small subset of point matches.
                        #   2. Compute a candidate Essential Matrix (E) from those.
                        #   3. Check how many of the other points agree with this model
                        #      by seeing if they satisfy the epipolar geometry rule
                        #      (basically: do they line up correctly given this motion?).
                        #   4. Repeat many times and keep the solution that has the most
                        #      "inliers" (agreeing points).
                        #
                        #   This way, a few bad matches won't ruin the result.
                        #
                        # Output:
                        # -------
                        # - E: The computed Essential Matrix (3x3)
                        # - mask: A binary array indicating which matches were
                        #   classified as inliers (1 = good, 0 = rejected).
                        E, mask = cv2.findEssentialMat(curr_pts, prev_pts,      # The matched 2D point coordinates between current and previous frames.
                                                       focal=self.focal_length, # The camera's approximate focal length in pixels.
                                                       pp=self.pp,              # The principal point (cx, cy), usually the image center.
                                                       method=cv2.RANSAC,       # Tell OpenCV to use RANSAC for outlier rejection.
                                                       prob=0.999,              # The probability that the correct solution is found.
                                                       threshold=1.0)           # Pixel error tolerance for deciding whether a point fits the model.

                        # If a valid essential matrix was found, we use it to get the relative pose.
                        #
                        # The essential matrix is a compact 3×3 object that encodes how one camera view relates to another (or how one frame relates
                        # to the next from the same moving camera). Practically, it answers this:
                        # “Given a point seen in image A, where must its matching point lie in image B?”
                        # It doesn’t give the exact matching pixel — it gives a line in image B where the match must sit. That line is called the epipolar line. 
                        # The essential matrix encodes the camera rotation and the direction of translation between the two views.
                        if E is not None:
                            # Recover the relative pose (rotation R and translation t)
                            # between the two frames from the essential matrix.
                            #
                            # - R (3x3 matrix): how the camera rotated between the
                            #   previous frame and the current frame.
                            #
                            # - t (3x1 vector): the *direction* the camera moved,
                            #   but NOT the actual distance.
                            #     Example: t might indicate "forward and slightly to the right",
                            #     but it won't tell you if the camera moved 1 cm or 1 meter.
                            #
                            # Why no scale? With a single (monocular) camera, depth
                            # cannot be directly measured — only the relative
                            # direction of motion can be inferred from parallax.
                            # To get real-world distance, you'd need extra info such as
                            # stereo cameras, LiDAR, IMU data, or wheel odometry.
                            _, R, t, mask = cv2.recoverPose(E, curr_pts, prev_pts,
                                                             focal=self.focal_length,
                                                             pp=self.pp)

                            # 'scale' converts the unit-length translation vector (t) from recoverPose
                            # into a real-world distance. Monocular visual odometry cannot determine
                            # absolute scale by itself because a single camera only sees relative motion.
                            # 
                            # Example: if recoverPose returns t=[0,0,1] and the robot actually moved
                            # 20 cm forward, then scale=0.20 (in meters). 
                            # 
                            # Without proper scale (e.g., stereo vision, depth sensor, wheel odometry),
                            # this value is just a placeholder for visualization. Setting it wrong will
                            # make the trajectory in RViz appear too large or too small.
                            scale = 0.1  # TODO: Replace with actual scale estimation method

                            # Even though 't' is just a direction, it's still a 3D vector
                            # with proportions between its components (left/right, forward/back).
                            # By multiplying 't' by a scalar value ('scale'), we stretch
                            # this unit vector into a real-world displacement. 
                            #   Example:
                            #       t = [0, 0, 1] → "straight forward"
                            #       scale = 0.20   → 20 cm per step
                            #       real translation = scale * t = [0, 0, 0.20]
                            #
                            # Camera coordinate convention (OpenCV default):
                            #   - X (t[0]) = left/right (lateral motion)
                            #   - Z (t[2]) = forward/backward motion
                            #
                            # Applying scale to each component gives the actual distance
                            # moved along that axis in the chosen unit (e.g., meters).
                            dx = scale * t[0, 0]  # Sideways motion
                            dz = scale * t[2, 0]  # Forward motion

                            # Estimate change in yaw (rotation around the vertical axis) from the
                            # 3x3 rotation matrix 'R' returned by cv2.recoverPose().
                            # We assume a 2D planar robot (like a ground robot) so we only care about yaw.
                            # atan2(y, x) computes the correct angle of a 2D vector, using the signs of both
                            # inputs to determine the proper quadrant (-π to π). Unlike atan(y/x), it avoids
                            # ambiguity, making it ideal for directions like robot yaw.
                            dtheta = math.atan2(R[1, 0], R[0, 0])

                            # Compute cosine and sine of the robot's current orientation 'theta'
                            # (theta is the accumulated yaw from the start).
                            # These are used to rotate the displacement from the camera frame
                            # into the odometry/world frame efficiently.
                            cos_theta = math.cos(self.theta)
                            sin_theta = math.sin(self.theta)

                            # Integrate the displacement into the robot's global odometry pose.
                            # Assumptions:
                            # - The robot moves in a 2D plane (like a floor).
                            # - 'dz' is the forward movement in the camera frame
                            # - 'dx' is the sideways (lateral) movement in the camera frame
                            #   (positive to the right).
                            # The camera frame displacement is rotated by the robot's current global
                            # orientation 'theta' to express motion in the global/world frame.
                            self.x += cos_theta * dz - sin_theta * dx
                            self.y += sin_theta * dz + cos_theta * dx

                            # Update the robot's global orientation (theta) by adding the
                            # relative change in yaw detected between the two frames.
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
