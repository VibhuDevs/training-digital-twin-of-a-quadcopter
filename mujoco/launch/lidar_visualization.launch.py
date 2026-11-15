"""Launch RViz2 with the MuJoCo LiDAR display configuration."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    config = Path(__file__).resolve().parents[1] / 'config/drone_lidar.rviz'
    return LaunchDescription([
        ExecuteProcess(cmd=['rviz2', '-d', str(config)], output='screen'),
    ])

if __name__ == '__main__':
    from launch import LaunchService
    service = LaunchService()
    service.include_launch_description(generate_launch_description())
    raise SystemExit(service.run())
