from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    This launch file launches the aruco_detect and fiducial_slam nodes.. 
    """
    launch_arguments = []
    launch_executables = []

    aruco_detect_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('aruco_detect'),
                'launch/aruco_detect.launch.py'
            ])
        ])
    )
    launch_executables.append(aruco_detect_launch_description)

    fiducial_slam_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('fiducial_slam'),
                'launch/fiducial_slam.launch.py'
            ])
        ])
    )
    launch_executables.append(fiducial_slam_launch_description)

    return LaunchDescription([

        # Launch arguments
        *launch_arguments,

        # Nodes, executables and event handlers
        *launch_executables,
    ])