"""Behavior tests for the hidden box.cfg-compatible direct G-code API."""

from contextlib import nullcontext
import os
import struct
import sys
from contextlib import nullcontext

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "klipper_extra"))
import creality_cfs  # noqa: E402


class FakeReactor:
    NOW = 0.0

    def monotonic(self):
        return 0.0

    def mutex(self):
        return nullcontext()


class FakeGcode:
    def __init__(self):
        self.commands = {}
        self.descriptions = {}

    def register_command(self, name, handler, desc=None):
        self.commands[name] = handler
        self.descriptions[name] = desc


class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()
        self.gcode = FakeGcode()
        self.objects = {}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        if name == "gcode":
            return self.gcode
        return self.objects.get(name, default)

    def add_object(self, name, value):
        self.objects[name] = value

    def register_event_handler(self, event, handler):
        pass


class FakeConfig:
    def __init__(self):
        self.printer = FakePrinter()

    def get_printer(self):
        return self.printer

    def get(self, name, default=None):
        return default

    def getint(self, name, default=None, **kwargs):
        return default

    def getfloat(self, name, default=None, **kwargs):
        return default

    def error(self, message):
        return RuntimeError(message)


class FakeGcmd:
    _MISSING = object()

    def __init__(self, **params):
        self.params = {key: str(value) for key, value in params.items()}
        self.responses = []

    def get(self, name, default=_MISSING):
        if name in self.params:
            return self.params[name]
        if default is not self._MISSING:
            return default
        raise self.error("missing %s" % name)

    def get_int(self, name, default=None, minval=None, maxval=None):
        raw = self.params.get(name)
        if raw is None:
            if default is None:
                raise self.error("missing %s" % name)
            value = default
        else:
            value = int(raw, 0)
        if minval is not None and value < minval:
            raise self.error("%s below minimum" % name)
        if maxval is not None and value > maxval:
            raise self.error("%s above maximum" % name)
        return value

    def get_float(self, name, default=None, minval=None, maxval=None):
        raw = self.params.get(name)
        value = default if raw is None else float(raw)
        if value is None:
            raise self.error("missing %s" % name)
        if minval is not None and value < minval:
            raise self.error("%s below minimum" % name)
        if maxval is not None and value > maxval:
            raise self.error("%s above maximum" % name)
        return value

    def error(self, message):
        return ValueError(message)

    def respond_info(self, message):
        self.responses.append(message)


@pytest.fixture
def direct_api(monkeypatch):
    monkeypatch.setattr(creality_cfs, "serial", object())
    config = FakeConfig()
    cfs = creality_cfs.CrealityCFS(config)
    calls = []

    def send(addr, status, function_code, data=b"", timeout=2.0, debug=False):
        calls.append((addr, status, function_code, data, timeout))
        if function_code == creality_cfs.FN["GET_MEASURING_WHEEL"]:
            return bytes.fromhex("f70107000e") + struct.pack(">f", -42.5) + b"\x00"
        if function_code == creality_cfs.FN["GET_BUFFER_STATE"]:
            return bytes.fromhex("f70104000502aa")
        if function_code == creality_cfs.FN["GET_FILAMENT_SENSOR_STATE"]:
            return bytes.fromhex("f70104000805aa")
        return bytes([0xF7, addr, 0x03, 0x00, function_code, 0x00])

    cfs._send = send
    return cfs, config.printer.gcode.commands, calls


def test_hidden_box_commands_are_registered_without_public_aliases(direct_api):
    """Dropping an underscore or a command registration breaks the hidden API."""
    _, commands, _ = direct_api
    expected = {
        "_BOX_GET_BOX_STATE",
        "_BOX_GET_VERSION_SN",
        "_BOX_GET_RFID",
        "_BOX_GET_REMAIN_LEN",
        "_BOX_GET_BUFFER_STATE",
        "_BOX_GET_FILAMENT_SENSOR_STATE",
        "_BOX_SET_BOX_MODE",
        "_BOX_SET_PRE_LOADING",
        "_BOX_CTRL_CONNECTION_MOTOR_ACTION",
        "_BOX_MEASURING_WHEEL",
        "_BOX_TIGHTEN_UP_ENABLE",
        "_BOX_EXTRUDE_PROCESS",
        "_CFS_EXTRUDE_UNTIL_SENSOR",
        "_BOX_RETRUDE_PROCESS",
        "_BOX_MOVE_DISTANCE",
        "_BOX_SEND_DATA",
    }
    assert expected <= set(commands)
    assert not {name.removeprefix("_") for name in expected} & set(commands)
    assert not {"_CFS_%s" % name for name in creality_cfs.FN} & set(commands)
    assert all(
        direct_api[0].gcode.descriptions[name] is None for name in expected)


@pytest.mark.parametrize(
    "command,params,want_fn,want_data,want_timeout",
    [
        ("_BOX_GET_RFID", {"SLOT": "A"},
         "GET_RFID", b"\x01", 2.0),
        ("_BOX_GET_REMAIN_LEN", {"SLOT": "D"},
         "GET_REMAIN_LEN", b"\x08", 2.0),
        ("_BOX_SET_BOX_MODE", {"ADDR": 2, "SLOT": "B", "MODE": "PRINT"},
         "SET_BOX_MODE", b"\x02\x00", 2.0),
        ("_BOX_SET_PRE_LOADING", {"SLOT": "D", "ACTION": "OPEN"},
         "SET_PRE_LOADING", b"\x08\x01", 2.0),
        ("_BOX_CTRL_CONNECTION_MOTOR_ACTION", {"ACTION": "RETRUDE"},
         "CTRL_CONNECTION_MOTOR_ACTION", b"\x02", 2.0),
        ("_BOX_TIGHTEN_UP_ENABLE", {"ENABLE": "ENABLE"},
         "TIGHTEN_UP_ENABLE", b"\x01", 2.0),
        ("_BOX_EXTRUDE_PROCESS", {"SLOT": "C", "STAGE": 7, "AMOUNT": 3},
         "EXTRUDE_PROCESS", b"\x04\x07\x03", 2.0),
        ("_BOX_RETRUDE_PROCESS", {"SLOT": "D", "TRIGGER": "MATERIAL"},
         "RETRUDE_PROCESS", b"\x08\x01", 2.0),
        ("_BOX_MOVE_DISTANCE", {"DIRECTION": "RETRUDE", "DIST": 50},
         "MOVE_DISTANCE", b"\x01\x32", 2.0),
    ],
)
def test_named_parameters_translate_to_validated_local_payloads(
        direct_api, command, params, want_fn, want_data, want_timeout):
    """Wrong enum polarity, slot mask, or byte order sends a dangerous command."""
    _, commands, calls = direct_api
    commands[command](FakeGcmd(**params))
    assert calls[-1] == (
        int(params.get("ADDR", 1)), 0xFF, creality_cfs.FN[want_fn],
        want_data, want_timeout)


def test_stage_seven_defaults_to_hardware_validated_amount(direct_api):
    """Omitting AMOUNT at stage 7 must not restore the known-wrong zero byte."""
    _, commands, calls = direct_api
    commands["_BOX_EXTRUDE_PROCESS"](FakeGcmd(SLOT="A", STAGE=7))
    assert calls[-1][3] == b"\x01\x07\x03"


@pytest.mark.parametrize(
    "command,params",
    [
        ("_BOX_GET_RFID", {"NUM": 1}),
        ("_BOX_GET_REMAIN_LEN", {"NUM": 1}),
        ("_BOX_SET_BOX_MODE", {"NUM": "A", "MODE": "PRINT"}),
        ("_BOX_SET_PRE_LOADING", {"NUM": 1, "ACTION": "OPEN"}),
        ("_BOX_EXTRUDE_PROCESS", {"NUM": "A", "STAGE": 0}),
        ("_BOX_RETRUDE_PROCESS", {"NUM": "A"}),
    ],
)
def test_legacy_num_parameter_is_rejected_without_serial_write(
        direct_api, command, params):
    """Accepting NUM could silently target a different slot after the rename."""
    _, commands, calls = direct_api
    with pytest.raises(ValueError, match="SLOT"):
        commands[command](FakeGcmd(**params))
    assert calls == []


def test_read_commands_decode_known_response_fields(direct_api):
    """Returning only opaque frames would lose useful stable sensor semantics."""
    _, commands, _ = direct_api

    wheel = FakeGcmd(ACTION="GET")
    commands["_BOX_MEASURING_WHEEL"](wheel)
    assert "-42.500mm" in wheel.responses[-1]

    buffer_state = FakeGcmd()
    commands["_BOX_GET_BUFFER_STATE"](buffer_state)
    assert "EMPTY" in buffer_state.responses[-1]

    sensors = FakeGcmd(POSITION="CONNECTIONS")
    commands["_BOX_GET_FILAMENT_SENSOR_STATE"](sensors)
    assert "A,C" in sensors.responses[-1]


def test_raw_sender_accepts_separated_hex_bytes_without_decimal_digit_splitting(direct_api):
    """DATA=0f,01 must mean bytes 0f01, unlike the upstream digit parser."""
    _, commands, calls = direct_api
    commands["_BOX_SEND_DATA"](
        FakeGcmd(ADDR=1, CMD="0x0d", STATE="0xff", TIMEOUT=3.5, DATA="0f,01"))
    assert calls[-1] == (1, 0xFF, 0x0D, b"\x0f\x01", 3.5)


@pytest.mark.parametrize(
    "command,params,message",
    [
        ("_BOX_SET_BOX_MODE", {"SLOT": "E", "MODE": "PRINT"}, "SLOT"),
        ("_BOX_EXTRUDE_PROCESS", {"SLOT": "A", "STAGE": 8}, "STAGE"),
        ("_BOX_RETRUDE_PROCESS", {"SLOT": "A", "TRIGGER": "NOPE"}, "TRIGGER"),
        ("_BOX_SEND_DATA", {"CMD": 13, "DATA": "abc"}, "DATA"),
    ],
)
def test_invalid_direct_parameters_fail_before_serial_write(
        direct_api, command, params, message):
    """Malformed direct parameters must never reach the physical serial link."""
    _, commands, calls = direct_api
    with pytest.raises(ValueError, match=message):
        commands[command](FakeGcmd(**params))
    assert calls == []
