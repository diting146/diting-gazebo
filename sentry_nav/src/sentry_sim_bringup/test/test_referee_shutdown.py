# Copyright 2026 diting
# Licensed under the Apache License, Version 2.0 (the "License");

"""Only normal ROS/keyboard shutdown should be swallowed by the HP node entrypoint."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from rclpy.executors import ExternalShutdownException
import pytest


SOURCE = (Path(__file__).resolve().parents[4] /
          'src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/referee_system/referee_hp.py')


@pytest.fixture
def referee(monkeypatch):
    spec = importlib.util.spec_from_file_location('referee_shutdown_contract', SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls = []
    node = SimpleNamespace(destroy_node=lambda: calls.append('destroy'))
    monkeypatch.setattr(module, 'RefereeHp', lambda: node)
    monkeypatch.setattr(module.rclpy, 'init', lambda **kwargs: calls.append('init'))
    monkeypatch.setattr(module.rclpy, 'try_shutdown', lambda: calls.append('try_shutdown'))

    def unexpected_shutdown():
        raise AssertionError('Non-idempotent shutdown must not be used')

    monkeypatch.setattr(module.rclpy, 'shutdown', unexpected_shutdown)
    return module, calls


@pytest.mark.parametrize('normal_exit', [None, KeyboardInterrupt, ExternalShutdownException])
def test_referee_normal_exit_destroys_node_and_uses_idempotent_shutdown(
        monkeypatch, referee, normal_exit):
    module, calls = referee

    def spin(node):
        calls.append('spin')
        if normal_exit:
            raise normal_exit()

    monkeypatch.setattr(module.rclpy, 'spin', spin)
    module.main()
    assert calls == ['init', 'spin', 'destroy', 'try_shutdown']


def test_referee_unexpected_failure_is_not_hidden(monkeypatch, referee):
    module, calls = referee

    def spin(node):
        calls.append('spin')
        raise RuntimeError('unexpected HP failure')

    monkeypatch.setattr(module.rclpy, 'spin', spin)
    with pytest.raises(RuntimeError, match='unexpected HP failure'):
        module.main()
    assert calls == ['init', 'spin', 'destroy', 'try_shutdown']
