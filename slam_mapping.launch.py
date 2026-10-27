# slam_mapping.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # start.sh 에서 slam_params, slam_params_file 모두 지원하도록 처리
    slam_params = LaunchConfiguration("slam_params", default="slam_toolbox_params.yaml")
    slam_params_file = LaunchConfiguration("slam_params_file", default=slam_params)
    # [REVIEW #20] rviz2는 YOLO와 자원을 나눠 쓰므로 기본 OFF (필요 시 use_rviz:=true)
    use_rviz = LaunchConfiguration("use_rviz", default="false")

    return LaunchDescription([
        DeclareLaunchArgument("slam_params", default_value="slam_toolbox_params.yaml"),
        DeclareLaunchArgument("slam_params_file", default_value=slam_params),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        
        # [핵심 추가] SLAM을 위한 base_link -> laser (라이다 센서) 간의 좌표계 연결
        # 라이다가 로봇 중심과 동일한 위치에 있다고 가정 (필요시 x,y,z 좌표 수정 가능)
        # [REVIEW #21] LD19 드라이버의 frame_id(ld19.launch.py 기본값 'base_laser')와 이름이 다르면 TF가 연결되지 않음 -> 확인 필요
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
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen", condition=IfCondition(use_rviz)),
    ])