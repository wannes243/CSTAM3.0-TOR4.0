from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
def generate_launch_description():
    return LaunchDescription([
        Node(package='fabtino_webots_bridge',executable='fabtino-webots-bridge',name='webots_bridge',parameters=[{'host':'127.0.0.1','port':8766,'scan_period_s':0.10}]),
        # The Webots controller uses mecanum kinematics with K=L+W.  Since
        # this localization adapter currently reduces the four encoders to
        # left/right averages, its equivalent differential track is 2*K.
        Node(package='fabtino_localization',executable='fabtino-localization',name='localization',parameters=[{'track_width_m':1.043834,'wheel_radius_m':0.100,'lidar_angle_sign':-1.0,'use_imu_yaw_reference':True,'rotation_pending_wheel_rate_threshold':0.05,'rotation_confirm_gyro_rate_threshold':0.03,'rotation_active_gyro_rate_threshold':0.02,'rotation_in_place_linear_threshold':0.08,'rotation_settle_time_s':0.40}]),
        Node(package='fabtino_mapping',executable='fabtino-mapping',name='mapping',parameters=[{'resolution_m':0.10,'radius_m':20.0,'lidar_angle_sign':-1.0,'lidar_subsample':8,'scan_queue_size':256,'pose_history_size':512,'pose_match_tolerance_s':0.10,'publish_period_s':0.05,'map_publish_period_s':0.50}]),
        Node(package='fabtino_navigation',executable='fabtino-mission-manager',name='mission_manager',parameters=[{'state_file':str(Path.home()/'.local/share/fabtino/navigation.json')}]),
        Node(package='fabtino_navigation',executable='fabtino-global-planner',name='global_planner',parameters=[{'resolution_m':0.10,'radius_m':20.0,'robot_radius_m':0.52,'safety_margin_m':0.08,'allow_unknown':True,'unknown_cost':1.15}]),
        Node(package='fabtino_navigation',executable='fabtino-local-planner',name='local_planner',parameters=[{
            'path_max_angular_rps':0.70, 'path_heading_kd':0.35,
            'path_turn_timeout_s':20.0, 'path_turn_no_progress_s':5.0,
            'movement_no_progress_s':15.0,
            'goal_tolerance_m':0.05, 'yaw_tolerance_rad':0.05235987756,
            'final_approach_distance_m':0.40, 'final_linear_mps':0.08,
            'final_max_angular_rps':0.35, 'final_heading_kp':1.2, 'final_heading_kd':0.35,
            'stopped_linear_mps':0.01, 'stopped_angular_rps':0.03, 'settle_time_s':0.40,
            'yaw_hysteresis_rad':0.03490658504, 'alignment_no_progress_s':5.0,
            'alignment_timeout_s':30.0, 'pose_timeout_s':0.30}]),
        Node(package='fabtino_safety',executable='fabtino-safety',name='safety_supervisor'),
        Node(package='fabtino_viewer',executable='fabtino-viewer-gateway',name='viewer_gateway'),
    ])
