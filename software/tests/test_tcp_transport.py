"""End-to-end test of the WiFi (TCP) transport.

A localhost TCP server stands in for the ESP32 SoftAP bridge: it speaks the same
two protocols the firmware does — mux frames (routed to the servo simulator) and
ASCII control lines (VERSION/PING). Confirms the whole driver stack + verify work
unchanged over TCP.
"""
import socket
import threading
import time

import pytest

from dogv3.driver.feetech import FeetechDriver, TcpTransport, make_transport
from dogv3.driver.mux import Bus
from dogv3.setup_program.verify_firmware import verify_transport

from sim import SimTransport


class FakeEsp32Server:
    """Minimal stand-in for the esp32-wifi firmware over TCP."""

    def __init__(self):
        self.sim = SimTransport({Bus.A: [1, 2, 3], Bus.B: [31, 32]})
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(1)
        self.port = self._srv.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self):
        self._srv.settimeout(0.2)
        try:
            conn, _ = self._srv.accept()
        except socket.timeout:
            return
        conn.settimeout(0.05)
        with conn:
            while not self._stop.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    out = self.sim.read()
                    if out:
                        conn.sendall(out)
                    continue
                if not data:
                    break
                if data[:1] == b"\xfe":
                    self.sim.write(data)
                    out = self.sim.read()
                    if out:
                        conn.sendall(out)
                else:
                    line = data.strip().upper()
                    if b"VERSION" in line:
                        conn.sendall(b"DOGV3-MUX v1.0\r\n")
                    elif b"PING" in line:
                        conn.sendall(b"PONG\r\n")

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._srv.close()
        except Exception:
            pass


@pytest.fixture
def esp32():
    s = FakeEsp32Server()
    yield s
    s.close()


def test_tcp_transport_ping_and_move(esp32):
    drv = FeetechDriver(TcpTransport("127.0.0.1", esp32.port, read_timeout=0.05), reply_timeout=0.5)
    assert drv.ping(Bus.A, 1)
    assert not drv.ping(Bus.A, 99)
    assert drv.write_pos_ex(Bus.A, 2, 1500, 800, 0)
    assert drv.read_present_position_raw(Bus.A, 2) == 1500


def _wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def test_tcp_sync_write(esp32):
    drv = FeetechDriver(TcpTransport("127.0.0.1", esp32.port, read_timeout=0.05), reply_timeout=0.5)
    drv.sync_write_positions({Bus.A: [(1, 1000, 500, 0)], Bus.B: [(31, 2500, 500, 0)]})
    # Broadcast sync writes are fire-and-forget; wait for the server to apply them.
    assert _wait_until(lambda: esp32.sim.servos[Bus.A][1].position == 1000)
    assert _wait_until(lambda: esp32.sim.servos[Bus.B][31].position == 2500)


def test_verify_over_tcp(esp32):
    t = TcpTransport("127.0.0.1", esp32.port, read_timeout=0.05)
    assert verify_transport(t, "DOGV3-MUX v1.0", do_scan=False)


def test_make_transport_selects_tcp(esp32):
    t = make_transport(tcp=f"127.0.0.1:{esp32.port}")
    assert isinstance(t, TcpTransport)
    t.close()


def test_make_transport_none_is_dry():
    assert make_transport() is None
