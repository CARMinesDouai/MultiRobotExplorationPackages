local PI = 3.1415926535898
local config = {
    scan_topic = "/scan",
    keypoint_topic = "keypoints",
    odom_topic = "/odom" , --  is used for  correction before scan matching
    localisation = {
        line_extraction = {
            bearing_std_dev = 1e-3,
            range_std_dev = 0.01, -- 0.1
            least_sq_angle_thresh = 1e-4,
            least_sq_radius_thresh = 1e-4,
            max_line_gap = 0.1, -- 0.4
            min_line_length = 0.2,
            min_range = 0.4,
            min_split_dist = 0.05,
            outlier_dist = 0.07, -- 0.06
            min_line_points = 15.0,
            line_scale = 0.06,
            max_dist = 6.0 -- max laser range
        },
        --icp = {
        --    max_distance = 0.05, -- m correspondence distance
        --    max_iteration = 200, -- maximum number of iterations 
        --    tf_epsilon = 1e-9, -- transformation epsilon
        --    eu_fitness = 0.1, -- the euclidean distance difference epsilon
        --    sample_fitness = 0.5,-- m
        --    use_non_linear = true
        --},
        scan_matcher = {
            use_imu = false,
            use_odom = true,
            max_angular_correction_deg = 45.0,
            max_linear_correction = 0.5,
            max_iterations = 50,
            epsilon_xy = 1e-6,
            epsilon_theta = 1e-6,
            max_correspondence_dist = 0.05,
            sigma = 0.01,
            use_corr_tricks = true,
            restart = false,
            restart_threshold_mean_error=0.01,
            restart_dt=1.0,
            restart_dtheta=0.1,
            clustering_threshold=0.25,
            orientation_neighbourhood=40.0,
            use_point_to_line_distance=true,
            do_alpha_test = false,
            do_alpha_test_thresholdDeg=20.0,
            outliers_maxPerc=0.9,
            outliers_adaptive_order=0.7,
            outliers_adaptive_mult=2.0,
            outliers_remove_doubles=true,
            do_visibility_test = false,
            do_compute_covariance = true,
            debug_verify_tricks=false,
            use_ml_weights=false,
            use_sigma_weights = false,
            confidence_factor = 1.8,
            max_laser_range = 8.0
        },
        correlative_scan_matcher = {
            kernel_resolution= 0.05,
            kernel_radius = 0.15,
            max_matching_score = 0.15,
            max_optimization_step=10,
            loop_closing_window=10.0,
            inlier_threshold = 2.0,
            min_inlier = 10.0,
            max_laser_range = 15.0
        },
        keyframe_sample_linear = 0.3,
        keyframe_sample_angular = 30.0 * (PI / 180.0),
        global_frame = "map",
        laser_frame = "laser",
        robot_base = "base_footprint",
        odom_frame = "odom",
        publish_odom = true,
        scan_topic = "/scan",
        odom_topic = "/odom",
        imu_topic = "/mobile_base/sensors/imu_data"
    },
    mapping = {
        p_occupied_with_observation=0.9,
        p_occupied_without_observation=0.3,
        angle_resolution= PI/720.0,
        large_log_odds=100.0,
        max_log_odds_for_belief=55.0,
        max_laser_range = 6.0, -- meter
        resolution = 0.05,
        map_frame = "map",
        init_width = 1.0, --m
        init_height = 1.0 --m
    },
    --line_seg_topic = "/line_markers",
    frequency = 5,
    map_topic = "/pmap"
}

local sim = false

if sim then
    config.localisation.scan_matcher.use_odom = true
    config.localisation.scan_matcher.max_correspondence_dist = 0.05
    config.localisation.scan_matcher.confidence_factor = 1.8
    config.localisation.scan_matcher.max_laser_range = 8.0
    config.localisation.keyframe_sample_linear = 0.5
    config.localisation.keyframe_sample_angular = 45.0 * (PI / 180.0)
    --config.localisation.scan_matcher.use_odom = false
end
return config