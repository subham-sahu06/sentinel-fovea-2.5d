import importlib.util
import os
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node


def test_master_launch_description():
    launch_path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'master.launch.py')
    spec = importlib.util.spec_from_file_location('master_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    ld = module.generate_launch_description()
    assert isinstance(ld, LaunchDescription)

    # Verify all 6 expected nodes are present in launch actions
    node_actions = [action for action in ld.entities if isinstance(action, Node)]
    assert len(node_actions) == 6

    packages = {action.node_package for action in node_actions}
    assert 'demo_pipeline' in packages
    assert 'safety_gateway' in packages
    assert 'adaptive_grid' in packages
    assert 'rosbridge_server' in packages

    executables = {action.node_executable for action in node_actions}
    assert 'synthetic_lidar' in executables
    assert 'semantic_segmentation' in executables
    assert 'ground_filter' in executables
    assert 'safety_gateway' in executables
    assert 'adaptive_grid' in executables
    assert 'rosbridge_websocket' in executables

