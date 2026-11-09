# Copyright 2026 diting
# Licensed under the Apache License, Version 2.0 (the "License");

"""Check ownership and transport setup without starting a process or changing the OS env."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription,
    SetEnvironmentVariable, Shutdown,
)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import pytest


WORKSPACE = Path(__file__).resolve().parents[4]
SCENE = WORKSPACE / 'sentry_nav/src/sentry_sim_bringup/launch/dual_sentry_simulation_launch.py'
GAZEBO = WORKSPACE / 'src/rmu_gazebo_simulator/rmu_gazebo_simulator/launch/gazebo.launch.py'


def _module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(module, overrides=None):
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def _text(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


@pytest.fixture
def scene(monkeypatch, tmp_path):
    module = _module(SCENE, 'lifecycle_scene')
    original = module.tempfile.NamedTemporaryFile

    def temporary_file(*args, **kwargs):
        kwargs['dir'] = str(tmp_path)
        return original(*args, **kwargs)

    monkeypatch.setattr(module.tempfile, 'NamedTemporaryFile', temporary_file)
    return module


def _partition_actions(context, actions):
    return [action for action in actions if isinstance(action, SetEnvironmentVariable)
            and _text(context, action.name) == 'IGN_PARTITION']


def test_owned_world_gets_a_unique_partition_before_any_children(scene):
    partitions = []
    for _ in range(2):
        context = _context(scene)
        context.environment.pop('IGN_PARTITION', None)
        actions = scene._launch_setup(context)
        assignments = _partition_actions(context, actions)
        assert len(assignments) == 1
        assert actions[0] is assignments[0]
        partition = _text(context, assignments[0].value)
        assert partition.startswith('sentry_sim_')
        assert len(partition) == len('sentry_sim_') + 32
        assert 'IGN_PARTITION' not in context.environment
        assignments[0].execute(context)
        assert context.environment['IGN_PARTITION'] == partition
        assert any(isinstance(action, GroupAction) for action in actions)
        partitions.append(partition)
    assert partitions[0] != partitions[1]


@pytest.mark.parametrize('owns_server', ['True', 'False'])
def test_explicit_partition_and_address_are_preserved(scene, owns_server):
    context = _context(scene, {'start_simulator': owns_server})
    context.environment['IGN_PARTITION'] = 'caller_owned_world'
    context.environment['IGN_IP'] = '192.0.2.10'
    actions = scene._launch_setup(context)
    assert not _partition_actions(context, actions)
    for action in actions:
        if isinstance(action, SetEnvironmentVariable):
            action.execute(context)
    assert context.environment['IGN_PARTITION'] == 'caller_owned_world'
    assert context.environment['IGN_IP'] == '192.0.2.10'


@pytest.mark.parametrize('external_partition', [None, ''])
def test_external_world_does_not_get_a_new_partition_or_a_server(scene, external_partition):
    context = _context(scene, {'start_simulator': 'False'})
    context.environment.pop('IGN_PARTITION', None)
    if external_partition is not None:
        context.environment['IGN_PARTITION'] = external_partition
    actions = scene._launch_setup(context)
    assert not _partition_actions(context, actions)
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == 1  # The navigation include is inside its existing GroupAction.
    assert includes[0].condition.evaluate(context) is False
    assert context.environment.get('IGN_PARTITION') == external_partition
    assert ('IGN_PARTITION' in context.environment) is (external_partition is not None)
    # ROS Node derives from ExecuteProcess and remains valid in external mode.
    assert not any(type(action) is ExecuteProcess for action in actions)


def test_gazebo_tracks_server_gui_and_clock_without_a_shell(monkeypatch):
    gazebo = _module(GAZEBO, 'lifecycle_gazebo')
    processes = []
    nodes = []

    def observe_process(**kwargs):
        processes.append(kwargs)
        return object()

    def observe_node(**kwargs):
        nodes.append(kwargs)
        return object()

    exports = {'IGN_GAZEBO_RESOURCE_PATH': '/caller/resources:/export/resources',
               'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': '/caller/plugins:/export/plugins'}
    monkeypatch.setattr(gazebo, 'ExecuteProcess', observe_process)
    monkeypatch.setattr(gazebo, 'Node', observe_node)
    monkeypatch.setattr(gazebo, '_gazebo_environment', lambda context: exports)
    context = _context(gazebo, {'world_sdf_path': '/tmp/world with spaces.sdf',
                               'ign_config_path': '/tmp/gui with spaces.config'})
    gazebo._launch_setup(context)
    assert len(processes) == 2
    by_name = {process['name']: process for process in processes}
    assert set(by_name) == {'gazebo_server', 'gazebo_gui'}
    for process in processes:
        assert process['shell'] is False
        assert process['additional_env'] == exports
        assert isinstance(process['on_exit'], Shutdown)
        assert float(process['sigterm_timeout']) > 0
        assert float(process['sigkill_timeout']) > 0
    commands = {name: [_text(context, argument) for argument in process['cmd']]
                for name, process in by_name.items()}
    for command in commands.values():
        assert Path(command[0]).name == 'ruby'
        assert Path(command[1]).name == 'ign'
        assert command[2] == 'gazebo'
        assert command[-2:] == ['--force-version', '6']
        assert '/tmp/world with spaces.sdf' in command
        assert '--headless-rendering' not in command
        assert not any('render-engine' in argument for argument in command)
    assert '-s' in commands['gazebo_server'] and '-g' not in commands['gazebo_server']
    assert '-r' in commands['gazebo_server']
    assert '-g' in commands['gazebo_gui'] and '-s' not in commands['gazebo_gui']
    assert commands['gazebo_gui'][commands['gazebo_gui'].index('--gui-config') + 1] == (
        '/tmp/gui with spaces.config')
    assert len(nodes) == 1
    assert nodes[0]['package'] == 'ros_gz_bridge'
    assert nodes[0]['executable'] == 'parameter_bridge'
    assert nodes[0]['arguments'] == ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']


def test_direct_launcher_keeps_package_exported_resource_and_plugin_paths():
    """Replacing the upstream process launcher must preserve its discovery paths."""
    gazebo = _module(GAZEBO, 'lifecycle_gazebo_exports')
    context = LaunchContext()
    context.environment.update({
        'GZ_SIM_SYSTEM_PLUGIN_PATH': '/caller/gz_plugins',
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': '/caller/ign_plugins',
        'GZ_SIM_RESOURCE_PATH': '/caller/gz_resources',
        'IGN_GAZEBO_RESOURCE_PATH': '/caller/ign_resources',
        'LD_LIBRARY_PATH': '/caller/libraries',
    })
    exports = gazebo._gazebo_environment(context)
    assert set(exports) == {'GZ_SIM_SYSTEM_PLUGIN_PATH', 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                            'GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH'}
    assert exports['GZ_SIM_SYSTEM_PLUGIN_PATH'].startswith('/caller/gz_plugins:/caller/libraries:')
    assert exports['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'].startswith('/caller/ign_plugins:/caller/libraries:')
    assert exports['GZ_SIM_RESOURCE_PATH'].startswith('/caller/gz_resources:')
    assert exports['IGN_GAZEBO_RESOURCE_PATH'].startswith('/caller/ign_resources:')
