from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/face_mesh/output',
        description='Output image topic with face mesh overlay'
    )

    fd_model_type_arg = DeclareLaunchArgument(
        'fd_model_type',
        default_value='precise',
        description='Face detection model type: precise or fast'
    )

    # Create the face mesh node
    face_mesh_node = Node(
        package='cvml_ros',
        executable='face_mesh_node',
        name='face_mesh_node',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'fd_model_type': LaunchConfiguration('fd_model_type'),
        }],
        output='screen'
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        fd_model_type_arg,
        face_mesh_node
    ])
