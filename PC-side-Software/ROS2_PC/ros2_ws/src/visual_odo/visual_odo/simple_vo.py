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

        # Declare ROS2 parameters to replace hard-coded values
        self.declare_parameters('', [
            ('focal', 800.0),          # Focal length in pixels
            ('cx', 400.0),             # Principal point x-coordinate
            ('cy', 300.0),             # Principal point y-coordinate
            ('scale', 0.1),            # Scale factor for translation
            ('orb_nfeatures', 500),    # Number of ORB features
            ('min_matches', 10),       # Minimum number of good matches required
            ('ransac_thresh', 1.0),    # RANSAC threshold for essential matrix
            ('max_good_matches', 50),  # Maximum number of good matches to keep
            ('ratio_thresh', 0.75),    # Lowe's ratio test threshold for kNN matching
            ('min_inliers', 5),        # Minimum number of RANSAC inliers for pose estimation
            ('enable_debug_viz', False), # Enable debug visualization
            ('motion_threshold', 0.002), # Minimum motion magnitude to update pose (prevents stationary drift)
        ])

        # Retrieve parameter values and set instance variables
        self.focal_length = self.get_parameter('focal').value
        self.pp = (self.get_parameter('cx').value, self.get_parameter('cy').value)
        self.scale = self.get_parameter('scale').value
        self.orb_nfeatures = self.get_parameter('orb_nfeatures').value
        self.min_matches = self.get_parameter('min_matches').value
        self.ransac_thresh = self.get_parameter('ransac_thresh').value
        self.max_good_matches = self.get_parameter('max_good_matches').value
        self.ratio_thresh = self.get_parameter('ratio_thresh').value
        self.min_inliers = self.get_parameter('min_inliers').value
        self.debug_viz_enabled = self.get_parameter('enable_debug_viz').value
        self.motion_threshold = self.get_parameter('motion_threshold').value

        # Debug visualization publisher (only create if enabled)
        if self.debug_viz_enabled:
            self.debug_pub = self.create_publisher(Image, '/visual_odo/debug_image', 10)

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
        self.orb = cv2.ORB_create(nfeatures=self.orb_nfeatures)

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
        # crossCheck=False → We disable cross-checking to enable k-nearest neighbors (kNN) matching.
        # Instead of requiring mutual best matches, we'll use Lowe's ratio test for better
        # outlier rejection. The ratio test is more sophisticated and effective than crossCheck
        # because it evaluates the uniqueness of each match by comparing the distance to the 
        # best match against the distance to the second-best match.
        #
        # Why kNN + ratio test is superior to crossCheck:
        # ------------------------------------------------
        # 1. **Better ambiguity detection**: crossCheck only ensures mutual best matches,
        #    but doesn't detect when a feature has multiple similar candidates (ambiguous matches).
        #    The ratio test specifically identifies and rejects such ambiguous cases.
        #
        # 2. **More robust filtering**: Lowe's ratio test (from the original SIFT paper) has been
        #    proven to be more effective at removing false matches in challenging scenarios like
        #    repetitive textures, motion blur, or scenes with similar-looking features.
        #
        # 3. **Tunable precision**: The ratio threshold (typically 0.75) can be adjusted to
        #    balance between match quantity and quality, whereas crossCheck is binary.
        #
        # 4. **Industry standard**: This approach is used in production SLAM systems and
        #    computer vision applications where reliability is critical.
        #
        # Summary of the improved process:
        # 1. ORB detects keypoints in both the current frame and previous frame.
        # 2. Each keypoint gets a binary descriptor.
        # 3. BFMatcher finds the k=2 nearest neighbors for each descriptor using Hamming distance.
        # 4. Lowe's ratio test filters matches: keep only if best_distance < 0.75 * second_best_distance.
        # 5. This ensures we only keep distinctive, unambiguous matches.
        #
        # Note: Brute-force matching can be slow if there are many keypoints.
        # If you increase ORB's nfeatures significantly, you may need to switch to
        # a faster approximate matcher like FLANN for real-time performance.
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

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
                    # ========================================================================
                    # IMPROVED MATCHING: k-Nearest Neighbors (kNN) + Lowe's Ratio Test
                    # ========================================================================
                    # 
                    # Instead of simple brute-force matching with crossCheck, we use a more
                    # sophisticated approach that provides better outlier rejection and 
                    # higher-quality matches for visual odometry.
                    #
                    # The kNN matching finds the k=2 best matches for each descriptor, then
                    # applies Lowe's ratio test to filter out ambiguous matches.
                    
                    # Step 1: Find k=2 nearest neighbors for each descriptor
                    # -------------------------------------------------------
                    # For each descriptor in the previous frame, find the 2 closest matches
                    # in the current frame. This gives us both the best match and the 
                    # second-best match for comparison.
                    knn_matches = self.matcher.knnMatch(self.prev_des, des, k=2)
                    
                    # Step 2: Apply Lowe's Ratio Test for outlier rejection
                    # -----------------------------------------------------
                    # The ratio test compares the distance of the best match to the distance
                    # of the second-best match. If the best match is significantly better
                    # (lower distance) than the second-best, we consider it a good match.
                    #
                    # Mathematical principle:
                    # If best_distance / second_best_distance < threshold (typically 0.75),
                    # then the match is distinctive and likely correct.
                    #
                    # Why 0.75 threshold?
                    # - This value comes from David Lowe's original SIFT paper (2004)
                    # - Empirically proven to work well across various scenarios
                    # - Lower values (0.6-0.7) = stricter filtering, fewer but higher quality matches
                    # - Higher values (0.8-0.9) = more permissive, more matches but potentially noisier
                    #
                    # Why this works better than crossCheck:
                    # 1. **Detects ambiguous matches**: If a feature looks similar to multiple
                    #    features in the other frame, the ratio between best and second-best
                    #    distances will be close to 1.0, indicating uncertainty.
                    # 2. **Handles repetitive patterns**: In scenes with repetitive textures
                    #    (like brick walls, windows), crossCheck might pass matches that are
                    #    geometrically inconsistent, but ratio test catches them.
                    # 3. **Better for motion blur**: When features are slightly distorted,
                    #    ratio test is more forgiving of small descriptor variations while
                    #    still rejecting truly bad matches.
                    
                    good_matches = []
                    for match_pair in knn_matches:
                        # Ensure we have exactly 2 matches (some descriptors might have fewer neighbors)
                        if len(match_pair) == 2:
                            best_match, second_best_match = match_pair
                            
                            # Apply Lowe's ratio test
                            # If the best match is significantly better than the second-best,
                            # we consider it a reliable match
                            if best_match.distance < self.ratio_thresh * second_best_match.distance:
                                good_matches.append(best_match)
                    
                    # Step 3: Limit the number of matches for performance and robustness
                    # -----------------------------------------------------------------
                    # Even after ratio test filtering, we still limit the total number of matches.
                    # This serves multiple purposes:
                    #
                    # 1. **Computational efficiency**: Essential matrix estimation and RANSAC
                    #    scale with the number of points. Too many points slow down processing.
                    #
                    # 2. **Memory management**: Large numbers of matches consume more memory
                    #    and can overwhelm the algorithm on resource-constrained systems.
                    #
                    # 3. **RANSAC effectiveness**: While RANSAC handles outliers, having
                    #    a reasonable number of high-quality matches improves convergence
                    #    and reduces the chance of finding incorrect solutions.
                    #
                    # 4. **Quality over quantity**: 50-100 excellent matches are much better
                    #    than 500 mediocre ones for pose estimation accuracy.
                    #
                    # Note: We don't need to sort by distance anymore since the ratio test
                    # already ensures we're getting high-quality matches. However, if you want
                    # to be extra conservative, you can uncomment the sorting line below.
                    # good_matches = sorted(good_matches, key=lambda x: x.distance)
                    
                    # Limit to maximum number of matches
                    good_matches = good_matches[:min(self.max_good_matches, len(good_matches))]

                    # Continue only if we have a minimum number of good matches
                    if len(good_matches) > self.min_matches:
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
                                                       focal=self.focal_length, # The camera's approximate focal length in pixels
                                                       pp=self.pp,              # The principal point (cx, cy), usually the image center
                                                       method=cv2.RANSAC,       # Tell OpenCV to use RANSAC for outlier rejection.
                                                       prob=0.999,              # The probability that the correct solution is found.
                                                       threshold=self.ransac_thresh)  # Pixel error tolerance for deciding whether a point fits the model

                        # If a valid essential matrix was found, we use it to get the relative pose.
                        #
                        # The essential matrix is a compact 3×3 object that encodes how one camera view relates to another (or how one frame relates
                        # to the next from the same moving camera). Practically, it answers this:
                        # “Given a point seen in image A, where must its matching point lie in image B?”
                        # It doesn’t give the exact matching pixel — it gives a line in image B where the match must sit. That line is called the epipolar line. 
                        # The essential matrix encodes the camera rotation and the direction of translation between the two views.
                        if E is not None:
                            # ========================================================================
                            # OUTLIER REJECTION: Filter matches using RANSAC inliers
                            # ========================================================================
                            #
                            # The findEssentialMat function above used RANSAC to find the best essential
                            # matrix while rejecting outlier matches. The 'mask' returned tells us which
                            # of our original matches were considered "inliers" (geometrically consistent)
                            # and which were "outliers" (rejected as inconsistent with the motion model).
                            #
                            # Why filter to inliers only?
                            # ----------------------------
                            # 1. **Improved pose accuracy**: Using only the points that fit the geometric
                            #    model reduces noise and bias in the final rotation and translation estimates.
                            #
                            # 2. **Consistent data**: The essential matrix E was computed using these exact
                            #    inlier points, so using the same points for pose recovery ensures consistency.
                            #
                            # 3. **Reduced computational load**: Fewer points mean faster pose recovery,
                            #    especially beneficial on resource-constrained systems like Raspberry Pi.
                            #
                            # 4. **Better debugging**: Monitoring the inlier ratio helps assess the quality
                            #    of feature matching and can indicate issues like motion blur, lighting
                            #    changes, or challenging scenes.
                            #
                            # How the mask works:
                            # -------------------
                            # - mask is a numpy array with the same length as curr_pts and prev_pts
                            # - mask[i] = 1 means the i-th match is an inlier (good match)
                            # - mask[i] = 0 means the i-th match is an outlier (rejected by RANSAC)
                            # - mask.ravel() flattens the mask to 1D for boolean indexing
                            
                            # Extract only the inlier points using the RANSAC mask
                            inlier_curr_pts = curr_pts[mask.ravel() == 1]
                            inlier_prev_pts = prev_pts[mask.ravel() == 1]
                            
                            # Calculate and log inlier statistics for monitoring and debugging
                            # This helps assess the quality of feature matching:
                            # - High inlier ratio (>70%) = good feature tracking, stable scene
                            # - Medium inlier ratio (40-70%) = acceptable, some challenging features
                            # - Low inlier ratio (<40%) = poor matching, may indicate motion blur,
                            #   lighting changes, or insufficient texture in the scene
                            inlier_count = np.sum(mask)
                            total_matches = len(mask)
                            inlier_ratio = inlier_count / total_matches if total_matches > 0 else 0.0
                            
                            # Log inlier statistics (use debug level to avoid spam, but can be changed to info for debugging)
                            self.get_logger().debug(f'RANSAC outlier rejection: {inlier_count}/{total_matches} inliers ({inlier_ratio:.1%})')
                            
                            # Continue with pose recovery only if we have sufficient inliers
                            # We need at least self.min_inliers points for recoverPose to work reliably
                            if inlier_count >= self.min_inliers:
                                # Recover the relative pose (rotation R and translation t)
                                # between the two frames from the essential matrix.
                                #
                                # Now using only the filtered inlier points for more accurate pose estimation:
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
                                #
                                # Note: recoverPose will return its own mask indicating which of the
                                # inlier points passed additional geometric checks (cheirality constraint).
                                # This second mask further filters for points that are in front of both cameras.
                                _, R, t, pose_mask = cv2.recoverPose(E, inlier_curr_pts, inlier_prev_pts,
                                                                   focal=self.focal_length,
                                                                   pp=self.pp)
                                
                                # Log additional pose recovery statistics if desired
                                pose_inliers = np.sum(pose_mask) if pose_mask is not None else inlier_count
                                self.get_logger().debug(f'Pose recovery: {pose_inliers}/{inlier_count} points passed cheirality check')
                                
                                # ========================================================================
                                # DEBUG VISUALIZATION: Show feature matches and inliers
                                # ========================================================================
                                # 
                                # If debug visualization is enabled, create and publish an image showing:
                                # - All feature matches (blue lines)
                                # - RANSAC inlier matches (green lines)  
                                # - Match statistics as text overlay
                                #
                                # This provides immediate visual feedback for algorithm development and tuning.
                                if self.debug_viz_enabled:
                                    self.publish_debug_visualization(cv_image, kp, good_matches, mask)
                                
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
                                scale = self.scale # TODO: Replace with actual scale estimation method

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

                                # Check if motion is above threshold to prevent stationary drift
                                motion_magnitude = math.sqrt(dx*dx + dz*dz) + abs(dtheta)
                                
                                if motion_magnitude < self.motion_threshold:
                                    # Motion is too small - likely noise when stationary
                                    # Skip pose update but still save frame data for next iteration
                                    self.get_logger().debug(f'Motion below threshold ({motion_magnitude:.4f} < {self.motion_threshold:.4f}), skipping pose update')
                                    self.prev_gray = gray.copy()
                                    self.prev_kp = kp
                                    self.prev_des = des
                                    return

                                # Motion is significant - proceed with pose update
                                self.get_logger().debug(f'Motion detected ({motion_magnitude:.4f}), updating pose: dx={dx:.3f}, dz={dz:.3f}, dtheta={dtheta:.3f}')

                                # Now we use the movements we calculated (dx for side-to-side, dz for forward/back, and dtheta for turning)
                                # to update the robot's overall position and direction (self.x, self.y, self.theta).
                                #
                                # Assumptions:
                                # - The robot moves on a flat surface (like a floor).
                                # - 'dz' is how much the robot moved forward from the camera's view.
                                # - 'dx' is how much the robot moved sideways from the camera's view (positive means to the right).
                                # We adjust these movements based on the robot's current direction (theta) to match the real-world map.
                                self.x += cos_theta * dz - sin_theta * dx  # Update the robot's forward/back position
                                self.y += sin_theta * dz + cos_theta * dx  # Update the robot's side-to-side position
                                # Update the robot's direction by adding the turn we detected between frames.
                                self.theta += dtheta

                                # Publish odometry message using the timestamp of the image
                                self.publish_odometry(msg.header.stamp)
                            else:
                                # Not enough inliers to reliably estimate pose
                                # Skip this frame and continue with previous pose estimate
                                self.get_logger().debug(f'Insufficient inliers ({inlier_count}) for pose estimation, skipping frame')
                                # Store current frame data for next iteration
                                self.prev_gray = gray.copy()
                                self.prev_kp = kp
                                self.prev_des = des
                                return

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

        # --- Covariance (6x6) for nav_msgs/Odometry.pose.covariance (row-major) ---
        # Covariance describes the uncertainty in your pose estimate and how errors in different variables relate.
        # - Variance (diagonal elements): Uncertainty for a single variable, like position or orientation.
        #   It's the square of the standard deviation (e.g., 0.1 means ~0.316m or rad uncertainty).
        # - Covariance (off-diagonal): How errors in two variables correlate (positive: errors move together; negative: opposite; zero: independent).
        #
        # Why it matters:
        # - Sensor fusion tools (e.g., EKF in robot_localization) use covariance to weight data: lower variance = more trust.
        # - Too low (e.g., 0.01): Over-trusts noisy data, leading to bad fused estimates.
        # - Too high (e.g., 1.0): Ignores your VO, reducing its usefulness.
        #
        # Layout (rows/cols): [x, y, z, roll, pitch, yaw]
        # Here, we set small variances (0.1) as a rough guess for low uncertainty in x, y, and yaw.
        # In production, compute from real error data (e.g., experiments with ground truth) or models.
        # Note: nav_msgs/Odometry.pose.covariance is a 6x6 row-major array.
        odom.pose.covariance[0] = 0.1   # variance on x (m²)
        odom.pose.covariance[7] = 0.1   # variance on y (m²)
        odom.pose.covariance[35] = 0.1  # variance on yaw (rad²)

        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Broadcast the transform for visualization / TF consumers
        self.broadcast_transform(timestamp)

        # Debug log showing current pose (helpful for tuning)
        self.get_logger().debug(f'Published odom: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')

    def broadcast_transform(self, timestamp):
        """
        Broadcast two transforms to the ROS TF2 system, which manages coordinate frames:
        
        - A transform describes the spatial relationship (position + orientation) between two frames.
        - Frames are reference points (e.g., 'odom' for odometry origin, 'base_link' for robot base).
            In ROS, a frame is a coordinate system attached to a specific thing (e.g., 'base_link' for the robot's base, 'camera_link' for the camera).
            There's also a global frame like 'odom' (odometry origin) or 'map' (world map) that serves as the overall reference point.
            Transforms relate these frames to each other, allowing everything to be positioned consistently in the global space.
        - Transforms allow other nodes (e.g., RViz, navigation) to relate data across frames, enabling visualization, mapping, and sensor fusion.
        
        Broadcasted transforms:
        1) odom -> base_link: Dynamic transform using current pose (self.x, self.y, self.theta).
            - Translation: Robot's position in odom frame.
            - Rotation: Robot's yaw as quaternion.
            - Purpose: Tracks robot movement for odometry and visualization.
        2) base_link -> camera_link: Static transform for camera mount.
            - Translation: Fixed offset (e.g., 0.1m forward, 0.2m up).
            - Rotation: Identity (camera points forward).
            - Purpose: Links camera data to robot frame for accurate sensor integration.
        
        Note: Camera transform is sent dynamically here for simplicity; in production, use a static publisher or URDF for efficiency.
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

    def publish_debug_visualization(self, current_image, current_kp, matches, mask):
        """
        Create and publish a debug visualization showing feature matches and inliers.
        
        This method creates a side-by-side image showing the previous frame (left) and 
        current frame (right) with feature matches drawn as colored lines:
        - Blue lines: All feature matches between frames
        - Green lines: Inlier matches that passed RANSAC filtering
        - Text overlay: Shows total matches and inlier count
        
        Args:
            current_image: The current BGR image from the camera
            current_kp: Current frame keypoints from ORB detector
            matches: List of DMatch objects representing feature correspondences
            mask: RANSAC mask indicating which matches are inliers (1) or outliers (0)
        """
        try:
            # Only proceed if we have a previous frame to compare against
            if self.prev_gray is not None and self.prev_kp is not None:
                # Convert grayscale previous frame back to BGR for visualization
                # (OpenCV's drawMatches requires both images to have the same number of channels)
                prev_bgr = cv2.cvtColor(self.prev_gray, cv2.COLOR_GRAY2BGR)
                
                # Draw all feature matches in blue
                # This shows every correspondence found by the feature matcher before RANSAC filtering
                debug_img = cv2.drawMatches(
                    prev_bgr, self.prev_kp,              # Previous frame and its keypoints (left side)
                    current_image, current_kp,           # Current frame and its keypoints (right side)  
                    matches, None,                       # Matches to draw, no existing image to overlay
                    matchColor=(255, 0, 0),              # Blue color for all matches
                    singlePointColor=(0, 255, 255),      # Yellow for unmatched keypoints
                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS  # Don't draw unmatched points
                )
                
                # Overlay inlier matches in green (only the matches that passed RANSAC)
                # This shows which correspondences are geometrically consistent with camera motion
                if mask is not None and len(matches) > 0:
                    # Filter matches to keep only the inliers based on RANSAC mask
                    inlier_matches = [m for i, m in enumerate(matches) if i < len(mask) and mask[i] == 1]
                    
                    if len(inlier_matches) > 0:
                        # Draw inlier matches in green over the existing image
                        debug_img = cv2.drawMatches(
                            prev_bgr, self.prev_kp,
                            current_image, current_kp,
                            inlier_matches, debug_img,       # Overlay on existing debug_img
                            matchColor=(0, 255, 0),          # Green color for inliers
                            singlePointColor=None,           # Don't change single point color
                            flags=cv2.DrawMatchesFlags_DRAW_OVER_OUTIMG | cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                        )
                
                # Add text overlay with match statistics for real-time monitoring
                # This helps assess the quality of feature tracking and parameter tuning
                inlier_count = np.sum(mask) if mask is not None else 0
                total_matches = len(matches)
                inlier_ratio = inlier_count / total_matches if total_matches > 0 else 0.0
                
                # Format statistics text with match counts and inlier percentage
                stats_text = f"Matches: {total_matches}, Inliers: {inlier_count} ({inlier_ratio:.1%})"
                
                # Add white text with black outline for visibility on any background
                cv2.putText(debug_img, stats_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.8, (0, 0, 0), 3)  # Black outline
                cv2.putText(debug_img, stats_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.8, (255, 255, 255), 2)  # White text
                
                # Add frame number or timestamp for debugging sequences
                frame_text = f"Frame: {self.get_clock().now().nanoseconds // 1000000}"  # Milliseconds
                cv2.putText(debug_img, frame_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (0, 0, 0), 3)  # Black outline
                cv2.putText(debug_img, frame_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (255, 255, 255), 2)  # White text
                
                # Convert the debug image back to ROS Image message format and publish
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
                debug_msg.header.stamp = self.get_clock().now().to_msg()
                debug_msg.header.frame_id = "camera_link"
                self.debug_pub.publish(debug_msg)
                
        except Exception as e:
            # Log warnings for debug visualization errors without crashing the main VO pipeline
            # Debug visualization should never interfere with the core odometry functionality
            self.get_logger().warn(f'Debug visualization error: {e}')


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
