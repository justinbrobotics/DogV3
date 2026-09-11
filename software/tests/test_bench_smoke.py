"""Bench smoke-test path for flashed onboard ESP32 firmware."""

from dogv3.config.loader import default_config
from dogv3.config.schema import ServoSpec
from dogv3.driver.mux import Bus
from dogv3.setup_program.bench_smoke import expected_ids_by_bus, run_bench_smoke
from dogv3.setup_program.verify_firmware import verify_transport

from sim import SimTransport


def commissioned_config():
    cfg = default_config()
    sid = 1
    for leg in ("BL", "BR", "FL", "FR"):
        for joint in ("hip", "femur", "tibia"):
            setattr(cfg.legs[leg].ids, joint, sid)
            cfg.servos[str(sid)] = ServoSpec(
                slot=f"{leg}_{joint}",
                verified={"assigned": True, "center": True, "direction": True, "range": True},
            )
            sid += 1
    return cfg


def test_sim_transport_answers_ascii_verify():
    t = SimTransport({Bus.A: [1], Bus.B: [7]})
    assert verify_transport(t, "DOGV3-MUX v1.0", do_scan=True)


def test_bench_smoke_passes_and_torque_offs_found_servos():
    cfg = commissioned_config()
    t = SimTransport({Bus.A: [1, 2, 3, 4, 5, 6], Bus.B: [7, 8, 9, 10, 11, 12]})
    for bus_servos in t.servos.values():
        for servo in bus_servos.values():
            servo.torque = True

    report = run_bench_smoke(t, config=cfg)
    assert report.ok
    assert report.firmware_ok is True
    assert report.found[Bus.A] == [1, 2, 3, 4, 5, 6]
    assert report.found[Bus.B] == [7, 8, 9, 10, 11, 12]
    assert all(not servo.torque for bus_servos in t.servos.values() for servo in bus_servos.values())


def test_bench_smoke_reports_missing_and_unexpected_ids():
    cfg = commissioned_config()
    t = SimTransport({Bus.A: [1, 2, 99], Bus.B: [7, 8, 9, 10, 11, 12]})
    report = run_bench_smoke(t, config=cfg, id_range=range(1, 100), torque_off=False)
    assert not report.ok
    assert report.missing[Bus.A] == [3, 4, 5, 6]
    assert report.unexpected[Bus.A] == [99]


def test_expected_ids_by_bus_uses_config_leg_buses():
    cfg = commissioned_config()
    expected = expected_ids_by_bus(cfg)
    assert expected[Bus.A] == [1, 2, 3, 4, 5, 6]
    assert expected[Bus.B] == [7, 8, 9, 10, 11, 12]
