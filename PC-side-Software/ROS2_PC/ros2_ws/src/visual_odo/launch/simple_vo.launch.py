from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Convert compressed images to raw
        Node(
            package='visual_odo',
            executable='compressed_to_raw',
            name='compressed_to_raw',
            output='screen'
        ),
        
        # Simple visual odometry with parameters
        Node(
            package='visual_odo',
            executable='simple_vo',
            name='simple_visual_odometry',
            output='screen',
            
            # Explanation of parameters:
            #
            # The first two (focal length and principal point) are approximate values for your camera's internal parameters
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
            # 2. PRINCIPAL POINT (cx, cy)
            #    - This is the pixel coordinate where the camera's optical axis
            #      (center of the lens) hits the image sensor.
            #    - For many cameras, it's approximately the image center.
            #      Example: for an 800x600 image → pp = (400, 300).
            #    - If slightly off, the math still works, but accuracy drops.
            #
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
            #
            # 3. SCALE
            #    - This is a scaling factor to convert the unit-length translation vector (t)
            #      from recoverPose into a real-world distance.
            #    - Monocular visual odometry cannot determine absolute scale by itself because
            #      a single camera only sees relative motion.
            #    - Example: if recoverPose returns t=[0,0,1] and the robot actually moved
            #      20 cm forward, then scale=0.20 (in meters).
            #    - Without proper scale (e.g., stereo vision, depth sensor, wheel odometry),
            #      this value is just a placeholder for visualization. Setting it wrong will
            #      make the trajectory in RViz appear too large or too small.
            #
            # 4. ORB FEATURES
            # The 'orb_nfeatures' parameter sets the maximum number of keypoints ORB will detect in each image frame. 
            # - More features → more keypoints to match → improves robustness of motion estimation
            #   because even if some matches are wrong, you still have enough good matches
            #   to reliably compute the essential matrix and recover the pose.
            # - BUT more features also → more computation and more time spent on matching,
            #   which can overload your CPU and increase latency, especially on small
            #   embedded boards like a Raspberry Pi or microcontroller-based systems.
            #
            # Typical ranges for orb_nfeatures:
            #   * 100–300 → very lightweight and fast, but less accurate in complex or low-texture scenes
            #   * 300–800 → good balance between speed and accuracy (500 is a common default)
            #   * 800–2000+ → very robust but CPU heavy; useful for high-resolution images or offline processing where speed is less critical.
            #
            # 5. MINIMUM GOOD MATCHES
            #    - This sets the minimum number of good feature matches required between consecutive frames
            #      to compute the essential matrix and recover the camera pose.
            #    - If the number of good matches is below this threshold, the algorithm will skip
            #      the pose estimation for that frame and retain the previous pose. This helps
            #      avoid incorrect pose estimates when there isn't enough reliable data.
            #    - Setting this too low may lead to noisy estimates, while setting it too high
            #      may cause the system to skip too many frames, resulting in a lagging trajectory
            #
            # 6. RANSAC THRESHOLD
            #    - This parameter defines the maximum allowable reprojection error (in pixels)
            #      for a point to be considered an inlier during the RANSAC process when
            #      estimating the essential matrix. (Pixel error tolerance for deciding whether a point fits the model)
            #
            # Ranges (starting points, pixels):
            #   - High-quality / low-noise:       0.5  – 1.0 px
            #   - Typical robot cameras:          1.0  – 3.0 px   (safe default: 1.0–1.5)
            #   - Noisy / motion-blur / low-res:  3.0  – 8.0 px (or higher if required)
            #   - Low-res images:                 use lower absolute values (0.5–2 px)
            #   - High-res images:                tolerate larger values (2–6 px)
            #
            # Trade-offs:
            #   - Too small threshold:
            #       • Very strict: few inliers → RANSAC may fail (unstable pose from few points).
            #       • High precision if it works, but fragile with noise or blur.
            #   - Too large threshold:
            #       • Permissive: accepts more inliers but more outliers slip in → biased/incorrect E.
            #       • Pose may look stable but be wrong; higher reprojection error.
            #   - Goal: accept a majority of true matches while excluding clear outliers.
            #
            # Quick tuning recipe:
            #   1. Start conservative: threshold = 1.0 px.
            #   2. Run on a representative short sequence (straight, turns, lighting changes).
            #   3. Monitor: inlier_count, median/mean reprojection or Sampson error, and pose stability.
            #   4. If inlier_count is very low → increase threshold (1 → 2 → 4 px).
            #   5. If many inliers but high residuals or bad pose → decrease threshold.
            #   6. Re-run until you find a balance: reasonable inlier_count + low median error.
            #   7. If resolution/focal changes, retune (pixel error scales with resolution).
            #
            # 7. MAX GOOD MATCHES
            #    - This parameter limits the number of top good matches (based on descriptor distance)
            #      used for estimating the essential matrix and recovering the camera pose.
            #      So basically we keep only a limited number of "good" matches (top N).
            #
            # ORB was initially configured to detect up to 500 features per frame (orb_nfeatures parameter).
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
            #
            # 8. RATIO THRESHOLD (Lowe's Ratio Test)
            #    - This parameter controls the threshold for Lowe's ratio test in k-nearest
            #      neighbors (kNN) matching, which is used to filter out ambiguous feature matches.
            #    - The ratio test compares the distance of the best match to the distance of the
            #      second-best match. A match is kept only if:
            #      best_match_distance < ratio_thresh × second_best_match_distance
            #
            # Why this parameter matters:
            # ---------------------------
            # - **Lower values (0.6-0.7)**: Stricter filtering, fewer but higher quality matches
            #   → Better for scenes with repetitive patterns or when precision is critical
            #   → May result in fewer matches, potentially causing tracking loss in low-texture areas
            #
            # - **Higher values (0.8-0.9)**: More permissive, more matches but potentially noisier
            #   → Better for challenging lighting conditions or when you need more matches
            #   → May allow some ambiguous matches through, potentially affecting accuracy
            #
            # - **Default (0.75)**: Proven optimal value from David Lowe's original SIFT paper
            #   → Good balance between match quality and quantity for most scenarios
            #   → Widely used in production computer vision systems
            #
            # When to adjust:
            # ---------------
            # - **Repetitive textures** (brick walls, windows): Use lower values (0.6-0.7)
            # - **Low-texture environments**: Use higher values (0.8-0.85) to get more matches
            # - **Motion blur or poor lighting**: Slightly higher values (0.8) can help
            # - **High-precision applications**: Lower values (0.65-0.7) for better accuracy
            #
            # This replaces the previous crossCheck method and provides superior outlier
            # rejection for visual odometry applications.
            #
            # 9. MINIMUM INLIERS (RANSAC Outlier Rejection)
            #    - This parameter sets the minimum number of inlier points required after
            #      RANSAC filtering to proceed with pose estimation via recoverPose().
            #    - RANSAC (used in findEssentialMat) identifies which feature matches are
            #      geometrically consistent with the camera motion model and which are outliers.
            #
            # Why this parameter is important:
            # -------------------------------
            # - **Reliability threshold**: Too few inlier points make pose estimation unreliable
            #   and prone to noise. The essential matrix might be computed correctly, but
            #   recovering accurate rotation and translation requires sufficient geometric constraints.
            #
            # - **Quality control**: If most matches are rejected as outliers, it indicates
            #   poor feature tracking conditions (motion blur, lighting changes, low texture).
            #   Skipping such frames prevents accumulation of incorrect pose estimates.
            #
            # - **Computational efficiency**: Avoiding pose computation on insufficient data
            #   saves processing time for frames that would produce unreliable results anyway.
            #
            # Parameter guidelines:
            # --------------------
            # - **Conservative (5-8 inliers)**: Good for most scenarios, ensures reliable pose recovery
            #   → Recommended for stable indoor environments or good lighting conditions
            #   → May skip more frames in challenging conditions but maintains accuracy
            #
            # - **Permissive (3-4 inliers)**: Allows pose estimation with fewer constraints
            #   → Useful for low-texture environments or when you need continuous tracking
            #   → May be less accurate but provides more frequent pose updates
            #
            # - **Strict (8+ inliers)**: Very conservative, only processes high-quality matches
            #   → Best for high-precision applications or outdoor environments with rich features
            #   → May lose tracking more frequently but ensures very reliable estimates
            #
            # Real-world considerations:
            # -------------------------
            # - **Fast motion**: Use higher values (6-8) as motion blur reduces match quality
            # - **Static/slow motion**: Lower values (3-5) may be sufficient
            # - **Rich textures**: Can use lower values as matches are typically higher quality
            # - **Poor lighting/low texture**: May need lower values to maintain tracking
            #
            # Monitoring tip: Watch the debug logs for "RANSAC outlier rejection" messages
            # to see typical inlier counts in your environment and adjust accordingly.

            parameters=[
                {'focal': 800.0},        # Focal length in pixels
                {'cx': 400.0},           # Principal point x-coordinate
                {'cy': 300.0},           # Principal point y-coordinate
                {'scale': 0.1},          # Scale factor for translation
                {'orb_nfeatures': 500},  # Number of ORB features
                {'min_matches': 10},     # Minimum number of good matches
                {'ransac_thresh': 1.0},  # RANSAC threshold
                {'max_good_matches': 50}, # Maximum number of good matches
                {'ratio_thresh': 0.75},  # Lowe's ratio test threshold for kNN matching
                {'min_inliers': 5}       # Minimum RANSAC inliers for pose estimation
            ]
        ),
    ])
