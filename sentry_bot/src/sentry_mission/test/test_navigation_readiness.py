"""Navigation activation must precede command ownership, with stop/cancel honored."""
from types import SimpleNamespace

import pytest
from sentry_mission.opening_mission import OpeningMissionNode


class ReadyProbe:
    def __init__(self, active=False, cancel=False, generation=4):
        self.generation = 4
        self.motion = SimpleNamespace(generation=generation)
        self.handle = SimpleNamespace(is_cancel_requested=cancel)
        self.stages = []
        self.calls = 0
        self.active = active
        self.nav_ready = SimpleNamespace(
            service_is_ready=lambda: True, call_async=self.call
        )

    def call(self, _):
        self.calls += 1
        return SimpleNamespace(success=self.active or self.calls > 1)

    def wait(self, future, _):
        return future

    def feedback(self, _, stage):
        self.stages.append(stage)


def test_waits_for_lifecycle_active():
    p = ReadyProbe()
    OpeningMissionNode.wait_navigation_ready(p, p.handle)
    assert p.calls == 2
    assert p.stages == ["waiting_navigation"]


def test_cancel_and_operator_stop_prevent_activation():
    for p, reason in [
        (ReadyProbe(cancel=True), "Mission canceled"),
        (ReadyProbe(generation=5), "Motion stopped"),
    ]:
        with pytest.raises(RuntimeError, match=reason):
            OpeningMissionNode.wait_navigation_ready(p, p.handle)
        assert p.calls == 0
