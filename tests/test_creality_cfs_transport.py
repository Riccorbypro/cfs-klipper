"""Behavioral tests for the Klipper extra's reactor-safe serial transport."""

from contextlib import AbstractContextManager
from types import SimpleNamespace

import pytest

from klipper_extra import creality_cfs


class FakeCompletion:
    def __init__(self, reactor=None):
        self.reactor = reactor
        self.value = None
        self.completed = False

    def wait(self, deadline, default=None):
        if self.reactor is not None and self.reactor.on_wait is not None:
            self.reactor.on_wait()
        if (self.reactor is not None and not self.completed
                and hasattr(self.reactor, "now")):
            self.reactor.now = deadline
        return self.value if self.completed else default

    def test(self):
        return self.completed

    def complete(self, value):
        self.value = value
        self.completed = True


class FakeMutex(AbstractContextManager):
    def __init__(self):
        self.entries = 0

    def __enter__(self):
        self.entries += 1
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return False


class FakeReactor:
    def __init__(self):
        self.on_wait = None
        self.on_pause = None
        self.now = 100.0
        self.pause_deadlines = []
        self.mutex_instance = FakeMutex()
        self.registered = []
        self.unregistered = []

    def monotonic(self):
        return self.now

    def pause(self, deadline):
        self.pause_deadlines.append(deadline)
        if self.on_pause is not None:
            self.on_pause()
        self.now = deadline

    def completion(self):
        return FakeCompletion(self)

    def mutex(self):
        return self.mutex_instance

    def register_fd(self, fd, callback):
        handle = (fd, callback)
        self.registered.append(handle)
        return handle

    def unregister_fd(self, handle):
        self.unregistered.append(handle)


class FakeSerial:
    def __init__(self, chunks=(), fd=17, read_error=None):
        self.chunks = list(chunks)
        self.fd = fd
        self.read_error = read_error
        self.allow_read = False
        self.closed = False
        self.writes = []
        self.reset_count = 0

    def fileno(self):
        return self.fd

    def reset_input_buffer(self):
        self.reset_count += 1

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def flush(self):
        pass

    def read(self, size):
        if not self.allow_read:
            raise AssertionError("serial reads must only run from the reactor fd callback")
        if self.read_error is not None:
            raise self.read_error
        return self.chunks.pop(0) if self.chunks else b""

    def close(self):
        self.closed = True


def make_transport(serial_port=None):
    cfs = object.__new__(creality_cfs.CrealityCFS)
    cfs.reactor = FakeReactor()
    cfs.serial_path = "/dev/ttyUSB-test"
    cfs.baud = 230400
    cfs.ser = serial_port
    cfs._serial_fd_handle = None
    cfs._rx_buffer = bytearray()
    cfs._pending_response = None
    cfs._pending_match = None
    cfs._bus_lock = cfs.reactor.mutex()
    cfs.addressed = True
    return cfs


def install_raw_write(monkeypatch):
    writes = []

    def write(fd, data):
        writes.append((fd, data))
        return len(data)

    monkeypatch.setattr(
        creality_cfs, "os", SimpleNamespace(write=write), raising=False)
    return writes


def test_send_reads_only_from_reactor_fd_callback(monkeypatch):
    response = creality_cfs.build_frame(
        0x01, 0x00, creality_cfs.FN["GET_BOX_STATE"], b"\x12")
    serial_port = FakeSerial([response])
    cfs = make_transport(serial_port)
    raw_writes = install_raw_write(monkeypatch)

    def deliver_response():
        serial_port.allow_read = True
        try:
            cfs._handle_serial_read(100.0)
        finally:
            serial_port.allow_read = False

    cfs.reactor.on_wait = deliver_response

    received = cfs._send(
        0x01, 0xFF, creality_cfs.FN["GET_BOX_STATE"], timeout=0.25)

    assert received == response
    assert raw_writes == [(
        serial_port.fd,
        creality_cfs.build_frame(
            0x01, 0xFF, creality_cfs.FN["GET_BOX_STATE"]))]
    assert serial_port.writes == []
    assert cfs.reactor.mutex_instance.entries == 1


def test_timeout_quarantines_late_reply_before_next_same_function_send(
        monkeypatch):
    """A late stage-5 reply must not complete the next EXTRUDE_PROCESS call."""
    serial_port = FakeSerial()
    cfs = make_transport(serial_port)
    install_raw_write(monkeypatch)
    function_code = creality_cfs.FN["EXTRUDE_PROCESS"]

    assert cfs._send(0x01, 0xFF, function_code, timeout=0.25) == b""

    late = creality_cfs.build_frame(
        0x01, 0x00, function_code, b"late-stage-5")
    current = creality_cfs.build_frame(
        0x01, 0x00, function_code, b"current-stage-6")

    cfs.reactor.on_pause = lambda: cfs._dispatch_serial_frame(late)
    cfs.reactor.on_wait = lambda: cfs._dispatch_serial_frame(current)

    received = cfs._send(0x01, 0xFF, function_code, timeout=0.25)

    assert received == current
    assert cfs.reactor.pause_deadlines == [100.5]


def test_serial_callback_assembles_a_frame_across_partial_reads():
    response = creality_cfs.build_frame(
        0x01, 0x00, creality_cfs.FN["GET_BUFFER_STATE"], b"\x02")
    serial_port = FakeSerial([response[:3], response[3:]])
    serial_port.allow_read = True
    cfs = make_transport(serial_port)
    pending = FakeCompletion()
    cfs._pending_response = pending
    cfs._pending_match = (0x01, creality_cfs.FN["GET_BUFFER_STATE"])

    cfs._handle_serial_read(100.0)
    assert not pending.test()

    cfs._handle_serial_read(100.1)
    assert pending.test()
    assert pending.value == response


def test_serial_callback_ignores_an_unmatched_frame_before_the_reply():
    unrelated = creality_cfs.build_frame(
        0x01, 0x00, creality_cfs.FN["GET_VERSION_SN"], b"v1")
    response = creality_cfs.build_frame(
        0x01, 0x00, creality_cfs.FN["GET_BOX_STATE"], b"\x12")
    serial_port = FakeSerial([unrelated + response])
    serial_port.allow_read = True
    cfs = make_transport(serial_port)
    pending = FakeCompletion()
    cfs._pending_response = pending
    cfs._pending_match = (0x01, creality_cfs.FN["GET_BOX_STATE"])

    cfs._handle_serial_read(100.0)

    assert pending.value == response


def test_serial_callback_rejects_bad_crc_before_accepting_the_reply():
    response = creality_cfs.build_frame(
        0x01, 0x00, creality_cfs.FN["GET_BOX_STATE"], b"\x12")
    corrupt = response[:-1] + bytes([response[-1] ^ 0xFF])
    serial_port = FakeSerial([corrupt + response])
    serial_port.allow_read = True
    cfs = make_transport(serial_port)
    pending = FakeCompletion()
    cfs._pending_response = pending
    cfs._pending_match = (0x01, creality_cfs.FN["GET_BOX_STATE"])

    cfs._handle_serial_read(100.0)

    assert pending.value == response


def test_serial_callback_resynchronizes_after_implausible_length():
    response = creality_cfs.build_frame(
        0x01, 0x00, creality_cfs.FN["GET_BOX_STATE"], b"\x12")
    serial_port = FakeSerial([b"\xF7\x01\xFF" + response])
    serial_port.allow_read = True
    cfs = make_transport(serial_port)
    pending = FakeCompletion()
    cfs._pending_response = pending
    cfs._pending_match = (0x01, creality_cfs.FN["GET_BOX_STATE"])

    cfs._handle_serial_read(100.0)

    assert pending.value == response


def test_open_registers_a_nonblocking_serial_fd(monkeypatch):
    created = {}
    serial_port = FakeSerial(fd=23)

    def open_serial(path, **kwargs):
        created["path"] = path
        created.update(kwargs)
        return serial_port

    monkeypatch.setattr(creality_cfs, "serial", SimpleNamespace(Serial=open_serial))
    cfs = make_transport()

    cfs._open()

    assert created == {
        "path": "/dev/ttyUSB-test",
        "baudrate": 230400,
        "timeout": 0,
        "write_timeout": 0,
    }
    assert cfs.reactor.registered[0][0] == 23
    callback = cfs.reactor.registered[0][1]
    assert callback.__self__ is cfs
    assert callback.__func__ is creality_cfs.CrealityCFS._handle_serial_read


def test_disconnect_unregisters_fd_closes_port_and_wakes_waiter():
    serial_port = FakeSerial()
    cfs = make_transport(serial_port)
    fd_handle = (17, cfs._handle_serial_read)
    cfs._serial_fd_handle = fd_handle
    pending = FakeCompletion()
    cfs._pending_response = pending

    cfs._handle_disconnect()

    assert cfs.reactor.unregistered == [fd_handle]
    assert serial_port.closed
    assert pending.test()
    assert pending.value == b""
    assert cfs.ser is None
    assert cfs._serial_fd_handle is None
    assert not cfs.addressed


def test_fatal_read_error_closes_transport_and_wakes_waiter():
    serial_port = FakeSerial(read_error=OSError("USB device disconnected"))
    serial_port.allow_read = True
    cfs = make_transport(serial_port)
    fd_handle = (17, cfs._handle_serial_read)
    cfs._serial_fd_handle = fd_handle
    pending = FakeCompletion()
    cfs._pending_response = pending

    cfs._handle_serial_read(100.0)

    assert cfs.reactor.unregistered == [fd_handle]
    assert serial_port.closed
    assert pending.value == b""
    assert cfs.ser is None
    assert not cfs.addressed


@pytest.mark.parametrize("broadcast_addr", [0xFE, 0xFF])
def test_broadcast_send_accepts_a_unicast_reply(broadcast_addr, monkeypatch):
    response = creality_cfs.build_frame(
        0x01, 0x00, creality_cfs.FN["CMD_GET_SLAVE_INFO"], bytes(range(14)))
    serial_port = FakeSerial([response])
    cfs = make_transport(serial_port)
    install_raw_write(monkeypatch)

    def deliver_response():
        serial_port.allow_read = True
        try:
            cfs._handle_serial_read(100.0)
        finally:
            serial_port.allow_read = False

    cfs.reactor.on_wait = deliver_response

    received = cfs._send(
        broadcast_addr, 0x00, creality_cfs.FN["CMD_GET_SLAVE_INFO"], timeout=0.25)

    assert received == response
