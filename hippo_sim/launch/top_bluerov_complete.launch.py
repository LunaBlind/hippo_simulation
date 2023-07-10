from ament_index_python.packages import get_package_share_path
import launch
import launch_ros

def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    spawner_path = str(package_path / "launch/spawn_bluerov.launch.py")
    low_level_ctrl_package_path = get_package_share_path('hippo_control')
    mixer_path = str(low_level_ctrl_package_path / "launch/node_actuator_mixer.launch.py")
    mixer_config_file_path = str(low_level_ctrl_package_path /('config/actuator_mixer_bluerov_default.yaml'))
    ctrl_package_path = get_package_share_path('bluerov_ctrl')
    velocity_ctrl_path = str(ctrl_package_path / "launch/node_velocity_control.launch.py")
    ctrl_config_path = str(ctrl_package_path / "config/ctrl_params_sim.yaml")
    pose_ctrl_module_path = str(ctrl_package_path / "launch/node_pose_control_module.launch.py")
    estimation_package_path = get_package_share_path('bluerov_estimation')
    estimation_path = str(estimation_package_path / "launch/estimation.launch.py")
    traj_gen_path = str(ctrl_package_path / "launch/node_trajectory_gen_eight.launch.py")
    visualization_package_path = get_package_share_path('uvms_visualization')
    visualization_path = str(visualization_package_path / "launch/visualization.launch.py")
    rviz_path = str(package_path / "launch/rviz.launch.py")
    model_path_rviz = str(package_path / 'models/bluerov/urdf/bluerov_rviz.xacro')


    vehicle_name = 'klopsi00'
    use_sim_time = True
    start_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(package_path / "launch/start_gazebo.launch.py")
        )
    )
    paths = [spawner_path, traj_gen_path]
    launch_files = []

    for path in paths:

        launch_files.append(launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
            path),
            launch_arguments=dict(use_sim_time=str(use_sim_time),
                                  vehicle_name=vehicle_name).items()
            )
        )
    launch_files.append(launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            velocity_ctrl_path
        ),
        launch_arguments=dict(use_sim_time=str(use_sim_time),
                              controller_type=str(2),
                              config_file=ctrl_config_path).items()
        )
    )
    launch_files.append(launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            pose_ctrl_module_path
        ),
        launch_arguments=dict(use_sim_time=str(use_sim_time),
                              config_file=ctrl_config_path).items()
        )
    )

    launch_files.append(launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                rviz_path
            ),
        launch_arguments=dict(use_sim_time=str(use_sim_time),
                              ).items()
        )
    )

    launch_files.append(launch_ros.actions.Node(package='robot_state_publisher',
                                              executable='robot_state_publisher',
                                              name='robot_state_publisher',
                                              namespace=vehicle_name,
                                              output='screen',
                                              parameters=[{'use_sim_time' : use_sim_time,
                                                           'robot_description': launch_ros.descriptions.ParameterValue(
    launch.substitutions.Command(['xacro ', model_path_rviz, " vehicle_name:=", vehicle_name]), value_type=str)}])) # pi: 3.14159265359


    launch_files.append(launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            mixer_path),
        launch_arguments=dict(use_sim_time=str(use_sim_time),
                              vehicle_name=vehicle_name,
                              mixer_path=mixer_config_file_path).items()
    )
    )

    launch_files.append(launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            estimation_path
        ),
        launch_arguments=dict(use_sim_time=str(use_sim_time),
                              vehicle_name=vehicle_name).items()
    ))
    launch_files.append(launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            visualization_path
        ),
        launch_arguments=dict(visualization_modules="[2, 3, 4, 5]",
                                use_sim_time=str(use_sim_time),
                              vehicle_name=vehicle_name).items()
    ))

    launch_files.append(start_gazebo)
    return launch.LaunchDescription([
        *launch_files,
        #tf_map_body_publisher

    ])


