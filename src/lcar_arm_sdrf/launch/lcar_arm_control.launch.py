# 文件路径: lcar_arm_sdrf/launch/lcar_arm_control.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    """
    这是一个用于启动真实（或模拟的 printf）lcar_arm 硬件控制的启动文件。
    它会依次启动：
    1. ros2_control_node: 加载并运行我们的硬件接口插件。
    2. robot_state_publisher: 根据 /joint_states 话题发布TF变换。
    3. controller_manager spawner: 启动 joint_state_broadcaster 和 mainpulator_controller。
    4. move_group: MoveIt的核心规划节点。
    5. rviz2: 用于可视化和交互。
    """

    # 使用MoveItConfigsBuilder来加载所有配置，这是ROS 2 MoveIt的标准做法
    # 它会自动找到我们所有的 .srdf, .yaml, .xacro 文件
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="lcar_arm_urdf", 
            package_name="lcar_arm_sdrf"
        )
        .robot_description(file_path="config/lcar_arm_urdf.urdf.xacro")
        .robot_description_semantic(file_path="config/lcar_arm_urdf.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # =================================================================================
    # 1. 启动 ros2_control_node，这是硬件控制的核心
    #    它会使用URDF中定义的插件 (我们的 lcar_arm_driver)
    # =================================================================================
    # 加载由MoveIt Setup Assistant生成的 ros2_controllers.yaml 文件
    ros2_controllers_path = os.path.join(
        get_package_share_directory("lcar_arm_sdrf"),
        "config",
        "ros2_controllers.yaml",
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description, 
            ros2_controllers_path
        ],
        output="screen",
        # remappings=[ # 如果您有多个机器人，可能需要这个
        #     ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        # ],
    )

    # =================================================================================
    # 2. 启动 robot_state_publisher
    #    它订阅 /joint_states 话题 (由joint_state_broadcaster发布)
    #    并根据URDF模型发布机器人所有连杆的TF坐标变换
    # =================================================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # =================================================================================
    # 3. 启动 Controller Spawner
    #    这是一个工具，用于请求 controller_manager 启动特定的控制器
    # =================================================================================
    # 首先启动 joint_state_broadcaster，它负责发布 /joint_states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 然后启动 mainpulator_controller，这是MoveIt用来发送轨迹的控制器
    # 注意：这里的控制器名称 "mainpulator_controller" 必须与 ros2_controllers.yaml 中定义的一致
    mainpulator_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mainpulator_controller", "--controller-manager", "/controller_manager"],
    )

    # =================================================================================
    # 4. 启动 MoveIt 的 move_group 节点
    # =================================================================================
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # =================================================================================
    # 5. 启动 RViz 可视化界面
    # =================================================================================
    rviz_config_file = os.path.join(
        get_package_share_directory("lcar_arm_sdrf"), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # 将所有要启动的节点组合成一个列表
    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        mainpulator_controller_spawner,
        move_group_node,
        rviz_node,
    ]

    return LaunchDescription(nodes_to_start)