# slam_mapping.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # start.sh 에서 slam_params, slam_params_file 모두 지원하도록 처리
    slam_params = LaunchConfiguration("slam_params", default="slam_toolbox_params.yaml")
    slam_params_file = LaunchConfiguration("slam_params_file", default=slam_params)

    return LaunchDescription([
        DeclareLaunchArgument("slam_params", default_value="slam_toolbox_params.yaml"),
        DeclareLaunchArgument("slam_params_file", default_value=slam_params),
        
        # [핵심 추가] SLAM을 위한 base_link -> laser (라이다 센서) 간의 좌표계 연결
        # 라이다가 로봇 중심과 동일한 위치에 있다고 가정 (필요시 x,y,z 좌표 수정 가능)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params_file],
            remappings=[("/scan", "/scan_filtered")], # 필터링된 라이다만 활용
        ),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen"),
    ])