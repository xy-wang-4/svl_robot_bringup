#
# Copyright (c) 2021 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os

def generate_launch_description():
    demos_pkg_prefix = get_package_share_directory('autoware_demos')
    autoware_launch_pkg_prefix = get_package_share_directory('autoware_auto_launch')
    lgsvl_param_file = os.path.join(autoware_launch_pkg_prefix, 'param/lgsvl_interface.param.yaml')
    pure_pursuit_param_file = os.path.join(demos_pkg_prefix, 'param/pure_pursuit.param.yaml')
    recordreplay_planner_param_file = os.path.join(demos_pkg_prefix, 'param/recordreplay_planner.param.yaml')
    pointcloud = LaunchConfiguration('pointcloud')
    use_sim_time = LaunchConfiguration('use_sim_time')
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_param_file,
        description='Path to config file for LGSVL Interface'
    )
    pure_pursuit_param = DeclareLaunchArgument(
        'pure_pursuit_param_file',
        default_value=pure_pursuit_param_file,
        description='Path to config file for pure pursuit controller'
    )
    recordreplay_planner_param = DeclareLaunchArgument(
        'recordreplay_planner_param_file',
        default_value=recordreplay_planner_param_file,
        description='Path to config file for record/replay planner'
    )
    sim = DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true')
    
    declare_pointcloud = DeclareLaunchArgument(
        'pointcloud',
        default_value='False',
        description='Whether to use pointcloud_to_laserscan')

    lgsvl_bridge = ExecuteProcess(cmd=["lgsvl_bridge"])
    
    odom_tf_node = Node(
            package='svl_robot_bringup',
            executable='odom_tf',
            parameters=[{'use_sim_time': True}]
        )

    pointcloud_to_laserscan_node = Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            parameters=[{
                'target_frame': 'base_scan',
                'transform_tolerance': 0.01,
                'min_height': -1.0,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan',
            condition=IfCondition(pointcloud)
        )

    # static_tf_nodes = GroupAction([
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher',
    #         arguments=['0', '0', '0', '0', '0', '0', '1', 'base_footprint', 'base_link']
    #     ),
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher',
    #         arguments=['0.27', '0', '0.1125', '0', '0', '0', '1', 'base_link', 'base_scan']
    #     ),
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher',
    #         arguments=['0.344', '0', '1.014', '0', '0', '0', '1', 'base_link', 'camera_link']  
    #     ),
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher',
    #         arguments=['0', '0', '0.21', '0', '0', '0', '1', 'base_link', 'gps_link']
    #     ),
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher',
    #         arguments=['0', '0', '0.19', '0', '0', '0', '1', 'base_link', 'imu_link']
    #     ),
    # ])

    urdf_pkg_prefix = get_package_share_directory('svl_robot_bringup')
    urdf_path = os.path.join(urdf_pkg_prefix, 'f1tenth.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()
    
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       parameters=[os.path.join(urdf_pkg_prefix, 'ekf_svl_test.yaml'), {'use_sim_time': True}]
    )

    lgsvl_interface = Node(
        package='lgsvl_interface',
        executable='lgsvl_interface_exe',
        namespace='vehicle',
        output='screen',
        parameters=[
          LaunchConfiguration('lgsvl_interface_param_file'),
          {"lgsvl.publish_tf": True}
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "/vehicle/state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ]
    )

    pure_pursuit = Node(
        package='pure_pursuit_nodes',
        executable='pure_pursuit_node_exe',
        name='pure_pursuit_node',
        output="screen",
        parameters=[
            LaunchConfiguration('pure_pursuit_param_file'),
        ],
        remappings=[
            ("current_pose", "/vehicle/vehicle_kinematic_state"),
            ("trajectory", "/planning/trajectory"),
            ("ctrl_cmd", "/vehicle/vehicle_command"),
            ("ctrl_diag", "/control/control_diagnostic"),
        ]
    )

    recordreplay_planner = Node(
        package='recordreplay_planner_nodes',
        executable='recordreplay_planner_node_exe',
        name='recordreplay_planner',
        namespace='planning',
        parameters=[
            LaunchConfiguration('recordreplay_planner_param_file')
        ],
        remappings=[
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('planned_trajectory', '/planning/trajectory')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(sim)
    ld.add_action(lgsvl_interface_param)
    ld.add_action(pure_pursuit_param)
    ld.add_action(recordreplay_planner_param)
    ld.add_action(lgsvl_bridge)
    ld.add_action(urdf_publisher)
    # ld.add_action(robot_localization_node)
    # ld.add_action(odom_tf_node)
    ld.add_action(lgsvl_interface)
    ld.add_action(pure_pursuit)
    ld.add_action(recordreplay_planner)
    # ld.add_action(declare_pointcloud)
    # ld.add_action(pointcloud_to_laserscan_node)
    
    return ld