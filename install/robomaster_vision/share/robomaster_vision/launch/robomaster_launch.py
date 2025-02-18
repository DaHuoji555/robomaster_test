import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # 视频发布节点
        launch_ros.actions.Node(
            package='robomaster_vision',
            executable='video_publisher',
            name='video_publisher',
            output='screen'
        ),

        # 图像处理节点
        launch_ros.actions.Node(
            package='robomaster_vision',
            executable='image_processor',
            name='image_processor',
            output='screen'
        ),

        # 预测节点
        launch_ros.actions.Node(
            package='robomaster_vision',
            executable='predictor_node',
            name='predictor_node',
            output='screen'
        ),

        # 结果显示节点
        launch_ros.actions.Node(
            package='robomaster_vision',
            executable='result_display',
            name='result_display',
            output='screen'
        )
    ])

