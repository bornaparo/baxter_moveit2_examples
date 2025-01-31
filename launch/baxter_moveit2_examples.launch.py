import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from launch_param_builder import load_yaml, load_xacro


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="False",
            description="Is gazebo used"
        ),
    ]
    
    baxter_srdf_path = os.path.join(get_package_share_directory("baxter_moveit_config"), "config", "baxter.srdf.xacro")
    baxter_urdf_path = os.path.join(get_package_share_directory("baxter_description"), "urdf", "baxter2_include.urdf.xacro")
    baxter_share_config = os.path.join(get_package_share_directory("baxter_moveit_config"), "config")
    moveit_params = {
        "robot_description": {
            "robot_description": load_xacro(Path(baxter_urdf_path), mappings={"gazebo": "False"}),
        },
        "robot_description_semantic": {
            "robot_description_semantic": load_xacro(Path(baxter_srdf_path)),
        },
        "robot_description_kinematics": {
            "robot_description_kinematics": load_yaml(Path(os.path.join(baxter_share_config, "kinematics.yaml"))),
        },
    }
    
    return LaunchDescription(
        declared_arguments + [
        Node(
            package="baxter_moveit2_examples",
            executable="baxter_moveit2_examples",
            name="baxter_moveit2_examples_node",
            parameters=[
                moveit_params["robot_description"],
                moveit_params["robot_description_semantic"],
                moveit_params["robot_description_kinematics"],
                {
                    "use_sim_time": True,
                }
            ]
        )
    ])