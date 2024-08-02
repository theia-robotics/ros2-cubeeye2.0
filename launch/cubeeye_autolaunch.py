import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

# launch camera node and select the camera [index] and run it with [type]
# ex) ros2 launch cubeeye_camera cubeeye_autolaunch.py index:='1' type:='6'
def generate_launch_description():
    start_index = LaunchConfiguration("index")
    start_type = LaunchConfiguration("type")

    # connect first source by default
    start_index_arg = DeclareLaunchArgument('index', default_value='0')
    # run amplitude and depth by default
    start_type_arg = DeclareLaunchArgument('type', default_value='6')

    cubeeye_node = Node(
        package='cubeeye_camera',
        executable='cubeeye_camera_node',
        output='screen'
    )

    scan_cubeeye = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                "/cubeeye_camera_node/scan ",
                "cubeeye_camera/srv/Scan "
            ],
        ],
        shell=True,
    )

    connect_cubeeye = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                "/cubeeye_camera_node/connect ",
                "cubeeye_camera/srv/Connect ",
                "\"{index: ", start_index, "}\""
            ],
        ],
        shell=True,
    )

    run_cubeeye = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                "/cubeeye_camera_node/run ",
                "cubeeye_camera/srv/Run ",
                "\"{type: ", start_type, "}\""
            ],
        ],
        shell=True,
    )

    return LaunchDescription(
        [
            start_index_arg,
            start_type_arg,
            cubeeye_node,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=cubeeye_node,
                    on_start=[
                        LogInfo(msg="scan cubeeye_camera"),
                        scan_cubeeye,
                    ]
                )
            ),
            RegisterEventHandler(            
                OnProcessExit(
                    target_action=scan_cubeeye,
                    on_exit=[
                        LogInfo(msg="connect cubeeye_camera"),
                        connect_cubeeye,
                    ]
                )
            ),
            RegisterEventHandler(            
                OnProcessExit(
                    target_action=connect_cubeeye,
                    on_exit=[
                        LogInfo(msg="run cubeeye_camera"),
                        run_cubeeye,
                    ]
                )
            ),
        ])
