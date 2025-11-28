import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart 
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory 

# --------------------------------------------------------------------------
# --- 1. 获取机器人描述 (使用 XACRO 展开) ---
# --------------------------------------------------------------------------
def get_robot_description(context, *args, **kwargs):
    pkg_share = get_package_share_directory('lekiwi')
    
    # 修正点：路径从 'urdf' 更改为 'config'
    xacro_file_path = os.path.join(pkg_share, 'config', 'lekiwi.urdf.xacro') 

    # 控制器配置文件路径
    controller_path = os.path.join(pkg_share, 'config', 'controllers_5dof.yaml')

    # 使用 Command 函数调用 XACRO 展开器
    robot_description_content = Command([
        'xacro ', xacro_file_path,
        ' initial_positions_file:=', LaunchConfiguration('initial_positions_file', default='initial_positions.yaml'),
        ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware', default='false'),
    ])
    
    urdf_content = context.perform_substitution(robot_description_content)
    absolute_mesh_path_prefix = pkg_share + os.sep
    urdf_content = urdf_content.replace('package://lekiwi/', absolute_mesh_path_prefix)
    robot_description = ParameterValue(urdf_content, value_type=str)
     
    return {
        'robot_description': robot_description,
        # Gazebo 使用和 ROS 2 Control 相同的描述
        'gazebo_description': robot_description, 
        'controller_path': controller_path
    }
# --------------------------------------------------------------------------
# --- 2. 启动逻辑函数 --- (修正后的 RegisterEventHandler)
# --------------------------------------------------------------------------
def launch_setup(context, *args, **kwargs):
    descriptions = get_robot_description(context)
    gz_sim = kwargs['gz_sim'] 

    # 1. 定义 control_node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': descriptions['robot_description']},
            descriptions['controller_path']
        ],
        output='screen',
        name='ros2_control_node'
    )
    
    # 2. 定义 spawn_robot 
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        arguments=[
            '-string', descriptions['gazebo_description'].value,
            '-name', 'lekiwi',
            '-allow_renaming', 'true',
            '-x', '0',
            '-y', '0',
            '-z', '0'
        ],
        output='screen'
    )
    RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=gz_sim, 
        on_start=[
            # 添加 3 秒延迟
            TimerAction(
                period=3.0, 
                actions=[spawn_robot]
            )
        ]
        )
    ),
    # 3. 定义控制器加载
    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], 
        output='screen'
    )

    # 完整节点列表
    nodes = [
        control_node, 
        # 延迟 spawn_robot，等待 Gazebo 启动
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=gz_sim, 
                on_start=[spawn_robot]
            )
        ),
        # 延迟控制器加载，等待 control_node 启动
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node, 
                on_start=[joint_state_broadcaster_spawner]
            )
        )
    ]
    return nodes
# --------------------------------------------------------------------------
# --- 3. 必需的 Launch 入口函数 --- (修正 GZ_SIM_RESOURCE_PATH 构造)
# --------------------------------------------------------------------------
def generate_launch_description():
    # 缺失的 Launch Arguments 声明
    initial_positions_file_arg = DeclareLaunchArgument(
        'initial_positions_file',
        default_value='initial_positions.yaml',
        description='YAML file with initial joint positions'
    )
    #是否使用假硬件接口参数
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Whether to use fake hardware interface'
    )

    #获取 lekiwi 包的安装共享目录的绝对路径字符串
    lekiwi_share_dir_str = get_package_share_directory('lekiwi')
    
    # 获取操作系统的路径分隔符 (通常是 ':')
    path_sep = os.pathsep
    
    # 获取当前 GZ_SIM_RESOURCE_PATH 的值
    new_gz_path = lekiwi_share_dir_str + path_sep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    #构造 GZ_SIM_RESOURCE_PATH 的新值 (字符串拼接)
    # 将新路径添加到现有路径的前面，并使用 os.pathsep 连接
    set_gz_resource_path = SetEnvironmentVariable(
                name='GZ_SIM_RESOURCE_PATH',
                value=new_gz_path
            )
    # 启动 Gazebo/Ignition 仿真器
    gz_sim = ExecuteProcess(
        cmd=['gz', 'gazebo', '-r', 'empty.sdf'],
        output='screen',
        name='gz'
    )

    return LaunchDescription([
        # 添加参数声明
        set_gz_resource_path,
        initial_positions_file_arg,
        use_fake_hardware_arg,
        gz_sim, 
        OpaqueFunction(function=launch_setup, kwargs={'gz_sim': gz_sim}) 
    ])