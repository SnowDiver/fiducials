from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    This launch file launches the aruco_detect node. 
    """

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

    dictionary_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="dictionary",
        default_value="7",
        description="The aruco dictionary to use. Possible values: 0 (DICT_4X4_50), 1 (DICT_4X4_100), 2 (DICT_4X4_250), 3 (DICT_4X4_1000), 4 (DICT_5X5_50), 5 (DICT_5X5_100), 6 (DICT_5X5_250), 7 (DICT_5X5_1000), 8 (DICT_6X6_50), 9 (DICT_6X6_100), 10 (DICT_6X6_250), 11 (DICT_6X6_1000), 12 (DICT_7X7_50), 13 (DICT_7X7_100), 14 (DICT_7X7_250), 15 (DICT_7X7_1000), 16 (DICT_ARUCO_ORIGINAL)."
    )
    launch_arguments.append(dictionary_arg)

    fiducial_len_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="fiducial_len",
        default_value="0.14",
        description="The length of one side of a fiducial in meters, used by the pose estimation."
    )
    launch_arguments.append(fiducial_len_arg)

    fiducial_len_override_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="fiducial_len_override",
        default_value="",
        description="A string expressing exceptions to fiducial_len. This can contain individual fiducial IDs, or ranges of them. Example: '1-10: 0.05, 12: 0.06' sets the length of fiducials 1 to 10 to 5cm and the length of fiducial 12 to 6cm."
    )
    launch_arguments.append(fiducial_len_override_arg)

    ignore_fiducials_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="ignore_fiducials",
        default_value="",
        description="A string expressing fiducials to be ignored. This can contain individual fiducial IDs, or ranges of them. Example: '1-10, 12' ignores fiducials 1 to 10 and 12. "
    )
    launch_arguments.append(ignore_fiducials_arg)

    publish_images_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="publish_images",
        default_value=g_default_true,
        description="If set, images will be published with the detected markers shown."
    )
    launch_arguments.append(publish_images_arg)

    # These next parameters don't have an official description. 
    # So I'm guessing what they change based on the code i read.
    
    # Not sure why one wouldn't want to execute the pose estimation,
    # as this seems the purpose of the node.
    do_pose_estimation_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="do_pose_estimation",
        default_value=g_default_true,
        description="If set, pose estimation will be executed"
    )
    launch_arguments.append(do_pose_estimation_arg)

    vis_msgs_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="vis_msgs",
        default_value=g_default_true,
        description="If set, the node will publish messages from vision_msgs (vision_msgs/msg/Detection2DArray), these are standardised msgs instead of the custom fiducials_msgs"
    )
    launch_arguments.append(vis_msgs_arg)

    topic_sub_cam_image_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="topic_sub_cam_image",
        default_value="/cam/color/image_raw",
        description="subscribed camera image topic"
    )
    launch_arguments.append(topic_sub_cam_image_arg)

    topic_sub_cam_info_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="topic_sub_cam_info",
        default_value="/cam/color/camera_info",
        description="subscribed camera info topic"
    )
    launch_arguments.append(topic_sub_cam_info_arg)

    intrinsics_override_enable_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="intrinsics_override.enable",
        default_value=g_default_false,
        description="if set to true, will overide the camera info msg. Will use the intrinsics_override.w, intrinsics_override.h and intrinsics_override.fov params."
    )
    launch_arguments.append(intrinsics_override_enable_arg)

    intrinsics_override_fov_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="intrinsics_override.fov",
        default_value=LaunchConfiguration("fov", default=0.9799),
        description="Horizontal Field of View, used if intrinsics_override.enable is true."
    )
    launch_arguments.append(intrinsics_override_fov_arg)

    intrinsics_override_w_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="intrinsics_override.w",
        default_value=LaunchConfiguration("w", default=1920),
        description="Width, used if intrinsics_override.enable is true."
    )
    launch_arguments.append(intrinsics_override_w_arg)

    intrinsics_override_h_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="intrinsics_override.h",
        default_value=LaunchConfiguration("h", default=1080),
        description="Heigth, used if intrinsics_override.enable is true."
    )
    launch_arguments.append(intrinsics_override_h_arg)


    # -------------------------------------
    # Launch executables
    # -------------------------------------

    launch_executables = []

    aruco_detect_node: Node = Node(
        package='aruco_detect',
            namespace='aruco_detect',
            executable='aruco_detect_node',
            name='aruco_detect',
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                topic_sub_cam_image_arg.name: LaunchConfiguration(topic_sub_cam_image_arg.name),
                topic_sub_cam_info_arg.name: LaunchConfiguration(topic_sub_cam_info_arg.name),
                dictionary_arg.name: LaunchConfiguration(dictionary_arg.name),
                fiducial_len_arg.name: LaunchConfiguration(fiducial_len_arg.name),
                publish_images_arg.name: LaunchConfiguration(publish_images_arg.name),
                intrinsics_override_enable_arg.name: LaunchConfiguration(intrinsics_override_enable_arg.name),
                intrinsics_override_fov_arg.name: LaunchConfiguration(intrinsics_override_fov_arg.name),
                intrinsics_override_w_arg.name: LaunchConfiguration(intrinsics_override_w_arg.name),
                intrinsics_override_h_arg.name: LaunchConfiguration(intrinsics_override_h_arg.name),
                # "ignore_fiducials": "2, 21, 25, 0, 13, 8, 11, 3, 7, 219, 20, 12"
            }]
    )
    launch_executables.append(aruco_detect_node)

    return LaunchDescription([

        # Launch arguments
        *launch_arguments,

        # Nodes, executables and event handlers
        *launch_executables,
    ])