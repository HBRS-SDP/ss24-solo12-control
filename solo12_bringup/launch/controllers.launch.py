from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def launch_args(context):

    declared_args = []

    declared_args.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        )
    )

    return declared_args


def launch_setup(context):

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("solo12_description"),
            "config",
            "solo12_controllers.yaml"
        ]
    )
    
    solo12_urdf = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("solo12_description"),
                        "urdf",
                        "solo12.urdf.xacro"
                    ]
                )
            ]
        ),
        value_type=str
    )
    

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            # {'use_sim_time': LaunchConfiguration("use_sim_time")},
            # {'robot_description': solo12_urdf},
            {'controller_manager': robot_controllers}
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        # emulate_tty=True
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    return [
        ros2_control_node,
        joint_state_broadcaster_spawner
    ]


def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
