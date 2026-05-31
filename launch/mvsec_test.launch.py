import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WS_ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

def generate_launch_description():
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(WS_ROOT, 'config', 'mvsec_vins_config.yaml'),
    )
    vins_folder_arg = DeclareLaunchArgument(
        'vins_folder',
        default_value=WS_ROOT + '/',
    )

    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker',
        name='feature_tracker',
        namespace='feature_tracker',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_path'),
            'vins_folder': LaunchConfiguration('vins_folder'),
        }],
    )

    vio_node = Node(
        package='gtsam_backend',
        executable='vio',
        name='vio',
        output='screen',
        additional_env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/libtbbmalloc_proxy.so.2'},
        remappings=[
            ('/vio/data_imu', '/visensor/imu'),
            ('/vio/data_uv',  '/feature_tracker/feature'),
            # ('/visensor/imu', '/vio/data_imu'),
            # ('/feature_tracker/feature', '/vio/data_uv'),
        ],
        parameters=[{
            'fixedId':  'world',
            'gravity':  [0.0, 0.0, 9.81007],
            'imuWait':  1096,
            'featWait': 5,

            'R_C0toI': [
                 0.9999717314190615,   -0.007438121416209933,  0.001100323844221122,
                 0.00743200596269379,   0.9999574688631824,    0.005461295826837418,
                -0.0011408988276470173,-0.0054529638303831614, 0.9999844816472552,
            ],
            'p_IinC0': [-0.03921656415229387, 0.00621263233002485, 0.0012210059575531885],

            # 'prior_qGtoI': [0.716147, 0.051158, 0.133778, 0.683096],
            # 'prior_pIinG': [0.925493, -6.214668, 0.422872],
            # 'prior_vIinG': [1.128364, -2.280640, 0.213326],
            # 'prior_ba':    [0.00489771688759235973,  0.00800897351104824969,  0.03020588299505782420],
            # 'prior_bg':    [-0.00041196751646696610,  0.00992948457018005999,  0.02188282212555122189],

            'prior_qGtoI': [-0.013186, 0.017696, 0.999626, 0.016153],
            'prior_pIinG': [-0.187971, 0.214207, 1.098604],
            'prior_vIinG': [-0.376705, 0.408789, 2.792471],
            'prior_ba':    [0.0, 0.0, 0.0],
            'prior_bg':    [0.0, 0.0, 0.0],

            'sigma_camera':                0.306555403,
            'accelerometer_noise_density': 0.08,
            'gyroscope_noise_density':     0.004,
            'accelerometer_random_walk':   0.00004,
            'gyroscope_random_walk':       2.0e-6,

            'sigma_prior_rotation':    0.1,
            'sigma_prior_translation': 0.3,
            'sigma_velocity':          0.1,
            'sigma_bias':              0.15,
            'sigma_pose_rotation':     0.1,
            'sigma_pose_translation':  0.2,
        }],
        # prefix=['xterm -e gdb -ex run --args'], # Opens a separate terminal window for GDB
    )

    # Publish a static TF making 'world' the root frame so rviz can render all displays
    world_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='log',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(WS_ROOT, 'rviz', 'mvsec_test.rviz')],
        output='log',
    )

    return LaunchDescription([
        config_path_arg,
        vins_folder_arg,
        world_tf_node,
        feature_tracker_node,
        vio_node,
        rviz_node,
    ])
