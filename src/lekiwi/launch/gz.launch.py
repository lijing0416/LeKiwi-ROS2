import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue 
from launch_ros.substitutions import FindPackageShare


def get_robot_description(context):
    """
    读取 Xacro 文件，将其转换为 URDF 字符串，并将 package:// 路径替换为绝对路径。
    """
    package_name = 'lekiwi'
    pkg_share = get_package_share_directory(package_name)
    
    # Xacro 和 YAML 路径
    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'config', 'lekiwi.urdf.xacro'
    ])
    initial_positions_path = PathJoinSubstitution([
        FindPackageShare(package_name), 'config', 'initial_positions.yaml'
    ])
    
    # 1. 执行 Xacro 命令行并获取 URDF 字符串
    # 假设使用 use_fake_hardware:=true 宏参数
    robot_description_command = Command([
        'xacro ', xacro_file,
        ' initial_positions_file:=', initial_positions_path, 
        ' use_fake_hardware:=', 'true'
    ])
    
    # 必须在 OpaqueFunction 内部执行 Command
    urdf_content = context.perform_substitution(robot_description_command)

    # 2. 强制路径替换（以兼容 Gazebo）
    absolute_mesh_path_prefix = pkg_share + os.sep
    processed_urdf_content = urdf_content.replace(f'package://{package_name}/', absolute_mesh_path_prefix)

    # 3. 将处理后的字符串包装为 ParameterValue
    robot_description = ParameterValue(processed_urdf_content, value_type=str)
    
    return {'robot_description': robot_description}


def launch_setup(context, *args, **kwargs):
    
    descriptions = get_robot_description(context)
    package_name = 'lekiwi'
    pkg_share = get_package_share_directory(package_name)

    # --- 1. 节点参数 ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 控制器配置文件路径
    controller_config_path = os.path.join(
        pkg_share, 'config', 'controllers.yaml'
    )
    with open(controller_config_path, 'r') as f:
        config_dict = yaml.safe_load(f)

    try:
        controller_params = config_dict['controller_manager']['ros__parameters']
    except KeyError:
        print("YAML 结构不正确：缺少 'controller_manager' 或 'ros__parameters' 键。")
        controller_params = {}
    print(f"\n--- DEBUG PATH: Controller Config Path is: {controller_config_path} ---\n")
    # 注意：这里我们不再使用 Python 读取和解包 YAML

    # --- 2. 节点定义 ---
    
    # 2.1 发布 URDF 模型
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': descriptions['robot_description'], 
            'use_sim_time': use_sim_time,
            'publish_frequency': 100.0 
        }]
    )
    
    # 2.2 ros2_control_node (控制器管理器)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        remappings=[
            ('~/robot_description', '/robot_description') 
        ],
        parameters=[controller_params], # 恢复传递字典
    )

    # 2.3 在 Gazebo 中创建实体 (create node)
    create_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'lekiwi_robot',
            '-topic', 'robot_description', 
            '-z', '0.0',
        ],
        output='screen',
    )

    # 2.4 控制器加载器 (使用 spawner Node)
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )
    
    lekiwi_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['lekiwi_arm_controller', '-c', '/controller_manager'],
        output='screen',
    )

    lekiwi_wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['lekiwi_wheel_controller', '-c', '/controller_manager'],
        output='screen',
    )
    
    # 3. 时序控制
    
    # 延迟启动实体创建和 Controller Manager，确保 RSP 启动
    delayed_nodes_launch = TimerAction(
        period=1.5, 
        actions=[
            create_entity,      
            control_node        
        ]
    )
    
    # 延迟启动控制器加载 (在 Controller Manager 启动后再次延迟)
    controller_spawners = TimerAction(
        period=3.0, 
        actions=[
            joint_state_broadcaster_spawner,
            lekiwi_arm_controller_spawner,
            lekiwi_wheel_controller_spawner
        ]
    )

    return [
        robot_state_publisher_node, 
        delayed_nodes_launch,       
        controller_spawners         
    ]


def generate_launch_description():
    # --- 启动 Gazebo ---
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    return LaunchDescription([
        # 启动参数定义
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        
        # 启动 Gazebo
        gz_launch,
        
        # OpaqueFunction 包装所有节点和逻辑
        OpaqueFunction(function=launch_setup)
    ])