from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    """
    This launch file launches the aruco_detect node. 
    """

    fiducial_slam_share_dir = Path(get_package_share_directory("fiducial_slam"))


    g_default_true = LaunchConfiguration('default_true', default=True)
    g_default_false = LaunchConfiguration('default_false', default=False)


    # -------------------------------------
    # Launch arguments
    # -------------------------------------

    launch_arguments = []

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Whether to use simulation time",
    )
    launch_arguments.append(use_sim_time_arg)

    map_file_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="map_file",
        default_value=[str(fiducial_slam_share_dir / "resources/map.txt")],
        description="Path to the file containing the generated map."
    )
    launch_arguments.append(map_file_arg)

    # I'm not certain how this is different from the map_file argument.
    initial_map_file_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="initial_map_file",
        default_value="",
        description="Path to the file containing initial map file"
    )
    launch_arguments.append(initial_map_file_arg)

    odom_frame_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="odom_frame",
        default_value="odom",
        description="If this is set to a non-empty string, then the result of the localization is published as a correction to odometry. For example, the odometry publishes the tf from map to odom, and this node publishes the tf from odom to base_link, with the tf from map to odom removed."
    )
    launch_arguments.append(odom_frame_arg)

    map_frame_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="map_frame",
        default_value="map",
        description="The name of the map (world) frame."
    )
    launch_arguments.append(map_frame_arg)

    base_frame_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="base_frame",
        default_value="base_link",
        description="The child frame of the tf we publish."
    )
    launch_arguments.append(base_frame_arg)

    # Currently not used, as duration does not accept a double, only two integers.
    # Now hard coded values are used at line 382 of map.cpp.
    future_date_transforms_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="future_date_transforms",
        default_value="0.0",
        description="Time with which to post-date the transform that is published, to indicate that this transform is valid into the future. "
    )
    launch_arguments.append(future_date_transforms_arg)

    publish_6dof_pose_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="publish_6dof_pose",
        default_value=g_default_false,
        description="If true, unmodified poses are published, otherwise they are constrained to have zero z translation and only the yaw component of rotation."
    )
    launch_arguments.append(publish_6dof_pose_arg)

    read_only_map_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="read_only_map",
        default_value=g_default_false,
        description="if true, the map is not modified, and only localization is performed. "
    )
    launch_arguments.append(read_only_map_arg)

    tf_publish_interval_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="tf_publish_interval",
        default_value="0.2",
        description="Specifies an interval at which poses are published, even if no fiducials are observed. This is useful in cases where the fiducial pose is used as a correction to odometry. "
    )
    launch_arguments.append(tf_publish_interval_arg)

    # These next parameters don't have an official description. 
    # So I'm guessing what they change based on the code i read.
    
    use_fiducial_area_as_weight_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="use_fiducial_area_as_weight",
        default_value=g_default_false,
        description="If set, use the fiducial area in pixels^2 as an indication of the 'goodness' of it. This will favor fiducials that are close to the camera and center of the image. The reciprical of the area is actually used, in place of reprojection error as the estimate's variance"
    )
    launch_arguments.append(use_fiducial_area_as_weight_arg)

    weighting_scale_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="weighting_scale",
        default_value="1e9",
        description="Scaling factor for weighing, only used if 'use_fiducial_area_as_weight' is set True."
    )
    launch_arguments.append(weighting_scale_arg)
    
    # Not completely sure whay this tf is, but it seems to be the pose of the robot.
    publish_tf_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="publish_tf",
        default_value=g_default_false,
        description="If set to True, publishes map -> odom transform"
    )
    launch_arguments.append(publish_tf_arg)

    # this error is added to the covariance of the pose. Line 382 of map.cpp
    systematic_error_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="systematic_error",
        default_value=LaunchConfiguration("sys_error", default=0.01),
        description="error added to the covariance, not certain what this error describes"
    )
    launch_arguments.append(systematic_error_arg)

    covariance_diagonal_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="covariance_diagonal",
        default_value="",
        description="If added overides the standard covariance matrix, needs 6 values."
    )
    launch_arguments.append(covariance_diagonal_arg)
    
    # Not certain what this does, as it doesn't seem to be used except for the initialization of the parameter.
    multi_error_theshold_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="multi_error_theshold",
        default_value=LaunchConfiguration("mul_err_tresh", default=-1.0),
        description="threshold of object error for using multi-fidicial pose. set -ve to never use"
    )
    launch_arguments.append(multi_error_theshold_arg)

    # -------------------------------------
    # Launch executables
    # -------------------------------------

    launch_executables = []

    fiducial_slam_node: Node = Node(
        package='fiducial_slam',
        namespace='fiducial_slam',
        executable='fiducial_slam_node',
        # name='fiducial_slam',
        parameters=[{
            map_frame_arg.name: LaunchConfiguration(map_frame_arg.name),
            odom_frame_arg.name: LaunchConfiguration(odom_frame_arg.name),
            base_frame_arg.name: LaunchConfiguration(base_frame_arg.name),
            publish_6dof_pose_arg.name: LaunchConfiguration(publish_6dof_pose_arg.name),
            map_file_arg.name: LaunchConfiguration(map_file_arg.name),
            publish_tf_arg.name: LaunchConfiguration(publish_tf_arg.name),
            # systematic_error_arg.name: LaunchConfiguration(systematic_error_arg.name),
        }]
    )
    launch_executables.append(fiducial_slam_node)

    return LaunchDescription([

        # Launch arguments
        *launch_arguments,

        # Nodes, executables and event handlers
        *launch_executables,
    ])