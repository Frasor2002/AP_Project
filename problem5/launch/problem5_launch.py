# Adapted from: https://github.com/PlanSys2/ros2_planning_system_examples/blob/rolling/plansys2_simple_example/launch/plansys2_simple_example_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  # Get the launch directory
  example_dir = get_package_share_directory('problem5')
  namespace = LaunchConfiguration('namespace')

  declare_namespace_cmd = DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='Namespace')

  plansys2_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('plansys2_bringup'),
      'launch',
      'plansys2_bringup_launch_monolithic.py')),
    launch_arguments={
      'model_file': example_dir + '/pddl/domain.pddl',
      'namespace': namespace
      }.items())
  
  stdout_line_buffer = SetEnvironmentVariable(
    'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

  # Specify the actions
  nodes = []
  package_name = 'problem5'
  output_to = 'screen'
  action_names = [
    'cool_artifact_action_node',
    'end_seismic_action_node',
    'fly_action_node',
    'hook_action_node',
    'load_action_node',
    'load_pod_action_node',
    'move_sealed_action_node',
    'move_unsealed_action_node',
    'place_in_pod_action_node',
    'reset_pod_action_node',
    'seal_action_node',
    'unhook_action_node',
    'unload_action_node',
    'unload_pod_action_node',
    'unseal_action_node'
  ]

  for name in action_names:
    action_node = Node(
      package=package_name,
      executable=name,
      name=name,
      namespace=namespace,
      output=output_to,
      parameters=[])
    nodes.append(action_node)

    
  ld = LaunchDescription()

  ld.add_action(declare_namespace_cmd)
  ld.add_action(plansys2_cmd)
  ld.add_action(stdout_line_buffer)

  # Specify actions
  for action_node in nodes:
    ld.add_action(action_node)

  return ld