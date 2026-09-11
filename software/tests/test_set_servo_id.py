"""dogv3-set-id: guarded single-servo ID assignment (D1 prerequisite)."""
import pytest

from dogv3.driver.feetech import FeetechDriver
from dogv3.driver.mux import Bus
from dogv3.setup_program.set_servo_id import set_servo_id

from sim import SimTransport


def _driver(ids_by_bus):
    return FeetechDriver(SimTransport(ids_by_bus), reply_timeout=0.1)


def test_write_id_changes_and_verifies():
    d = _driver({Bus.A: [1], Bus.B: []})
    assert d.write_id(Bus.A, 1, 7) is True
    assert d.ping(Bus.A, 7)
    assert not d.ping(Bus.A, 1)


def test_set_servo_id_single_servo():
    d = _driver({Bus.A: [1], Bus.B: []})
    bus, src, ok = set_servo_id(d, new_id=7)
    assert (bus, src, ok) == (Bus.A, 1, True)
    assert d.ping(Bus.A, 7)


def test_set_servo_id_refuses_multiple():
    d = _driver({Bus.A: [1, 2], Bus.B: []})
    with pytest.raises(RuntimeError, match="ONE servo"):
        set_servo_id(d, new_id=7)


def test_set_servo_id_force_allows_multiple():
    d = _driver({Bus.A: [1, 2], Bus.B: []})
    bus, src, ok = set_servo_id(d, new_id=7, force=True)
    assert ok and bus == Bus.A


def test_set_servo_id_no_servo():
    d = _driver({Bus.A: [], Bus.B: []})
    with pytest.raises(RuntimeError, match="no servo"):
        set_servo_id(d, new_id=7)


def test_set_servo_id_from_mismatch():
    d = _driver({Bus.A: [3], Bus.B: []})
    with pytest.raises(RuntimeError, match="not --from"):
        set_servo_id(d, new_id=7, expected_from=1)


def test_set_servo_id_already_target_is_noop():
    d = _driver({Bus.A: [7], Bus.B: []})
    bus, src, ok = set_servo_id(d, new_id=7)
    assert ok and src == 7


def test_new_id_out_of_range_rejected():
    d = _driver({Bus.A: [1], Bus.B: []})
    with pytest.raises(ValueError):
        set_servo_id(d, new_id=999)
