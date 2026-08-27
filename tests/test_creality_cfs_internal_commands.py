"""Contract tests for the hidden, low-level CFS G-code commands."""

from contextlib import nullcontext
from types import SimpleNamespace
import struct

import pytest

from klipper_extra import creality_cfs


class RecordingGCode:
    def __init__(self):
        self.commands = {}

    def register_command(self, name, handler, desc=None):
        self.commands[name] = (handler, desc)


class FakePrinter:
    def __init__(self, gcode):
        self.gcode = gcode
        self.reactor = SimpleNamespace(mutex=lambda: nullcontext())

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name):
        assert name == "gcode"
        return self.gcode

    def add_object(self, name, obj):
        pass

    def register_event_handler(self, event, handler):
        pass


class FakeConfig:
    def __init__(self, printer):
        self.printer = printer

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


class FakeGCmd:
    def __init__(self, **params):
        self.params = {name.upper(): str(value) for name, value in params.items()}
        self.responses = []

    def get(self, name, default=None):
        return self.params.get(name, default)

    def get_int(self, name, default=None, minval=None, maxval=None):
        raw = self.params.get(name)
        if raw is None:
            if default is None:
                raise self.error("%s is required" % name)
            value = default
        else:
            try:
                value = int(raw, 0)
            except ValueError:
                raise self.error("%s must be an integer" % name)
        if minval is not None and value < minval:
            raise self.error("%s must be at least %s" % (name, minval))
        if maxval is not None and value > maxval:
            raise self.error("%s must be at most %s" % (name, maxval))
        return value

    def error(self, message):
        return RuntimeError(message)

    def respond_info(self, message):
        self.responses.append(message)


def make_cfs():
    gcode = RecordingGCode()
    printer = FakePrinter(gcode)
    cfs = creality_cfs.CrealityCFS(FakeConfig(printer))
    cfs.box_addr = 1
    sent = []
    responses = {}

    def send(addr, status, function_code, data=b"", **kwargs):
        sent.append((addr, status, function_code, data, kwargs))
        return responses.get(
            function_code,
            creality_cfs.build_frame(addr, 0x00, function_code))

    cfs._send = send
    return cfs, gcode, sent, responses


def invoke(gcode, command_name, **params):
    gcmd = FakeGCmd(**params)
    gcode.commands[command_name][0](gcmd)
    return gcmd


def test_registers_every_protocol_function_as_a_hidden_gcode_command():
    cfs, gcode, sent, responses = make_cfs()
    expected_names = {"_CFS_%s" % name for name in creality_cfs.FN}

    assert expected_names <= set(gcode.commands)
    assert all(gcode.commands[name][1] is None for name in expected_names)


@pytest.mark.parametrize(
    "command,params,expected_addr,expected_status,expected_function,expected_data,expected_kwargs",
    [
        ("_CFS_GET_RFID", {"SLOT_INDEX": 2}, 1, 0xFF, 0x02, b"\x02", {}),
        ("_CFS_GET_REMAIN_LEN", {"SLOT_INDEX": 3}, 1, 0xFF, 0x03, b"\x03", {}),
        ("_CFS_SET_BOX_MODE", {"SLOT": "C", "MODE": "PRINT"},
         1, 0xFF, 0x04, b"\x04\x00", {}),
        ("_CFS_GET_BUFFER_STATE", {}, 1, 0xFF, 0x05, b"", {}),
        ("_CFS_CTRL_CONNECTION_MOTOR_ACTION", {"ACTION": "RETRUDE"},
         1, 0xFF, 0x07, b"\x02", {}),
        ("_CFS_GET_FILAMENT_SENSOR_STATE", {"BANK": "CONNECTIONS"},
         1, 0xFF, 0x08, b"\x01", {}),
        ("_CFS_GET_BOX_STATE", {}, 1, 0xFF, 0x0A, b"", {}),
        ("_CFS_SET_PRE_LOADING", {"SLOTS": "AC", "ACTION": "RUN"},
         1, 0xFF, 0x0D, b"\x05\x02", {"timeout": 45.0}),
        ("_CFS_GET_MEASURING_WHEEL", {}, 1, 0xFF, 0x0E, b"\x01", {}),
        ("_CFS_TIGHTEN_UP_ENABLE", {"ENABLED": "TRUE"},
         1, 0xFF, 0x0F, b"\x01", {}),
        ("_CFS_EXTRUDE_PROCESS", {"SLOT": "D", "STAGE": 7, "AMOUNT": 3},
         1, 0xFF, 0x10, b"\x08\x07\x03", {}),
        ("_CFS_RETRUDE_PROCESS", {"SLOT": "B", "STAGE": 1},
         1, 0xFF, 0x11, b"\x02\x01", {}),
        ("_CFS_GET_VERSION_SN", {}, 1, 0xFF, 0x14, b"", {}),
        ("_CFS_MOVE_DISTANCE", {"SLOT": "C", "DIRECTION": "REVERSE", "DISTANCE": 50},
         1, 0xFF, 0x31, b"\x04\x01\x32", {}),
        ("_CFS_CMD_SET_SLAVE_ADDR",
         {"NEW_ADDRESS": 5, "UID": "00112233445566778899aabb"},
         0xFE, 0x00, 0xA0,
         b"\x05\x00\x11\x22\x33\x44\x55\x66\x77\x88\x99\xaa\xbb", {}),
        ("_CFS_CMD_GET_SLAVE_INFO", {}, 0xFE, 0x00, 0xA1, b"\xfe\xfe", {}),
        ("_CFS_CMD_ONLINE_CHECK", {}, 1, 0x00, 0xA2, b"", {}),
    ],
)
def test_hidden_commands_encode_readable_parameters_as_protocol_bytes(
        command, params, expected_addr, expected_status, expected_function,
        expected_data, expected_kwargs):
    cfs, gcode, sent, responses = make_cfs()

    invoke(gcode, command, **params)

    assert sent == [(
        expected_addr, expected_status, expected_function, expected_data,
        expected_kwargs)]


@pytest.mark.parametrize(
    "command,response_status,response_data,params,expected_text",
    [
        ("_CFS_GET_BUFFER_STATE", 0x00, b"\x02", {}, "buffer=EMPTY"),
        ("_CFS_GET_FILAMENT_SENSOR_STATE", 0x00, b"\x05", {},
         "bitmask=0x05 slots=A,C"),
        ("_CFS_GET_MEASURING_WHEEL", 0x00, struct.pack(">f", -12.5), {},
         "distance=-12.500mm"),
        ("_CFS_GET_VERSION_SN", 0x00, b"113100008730633225CMPN", {},
         "text=113100008730633225CMPN"),
        ("_CFS_GET_RFID", 0x00, b"A:none;", {"SLOT_INDEX": 1},
         "text=A:none;"),
        ("_CFS_GET_BOX_STATE", 0x0C, b"", {},
         "status=0x0c(EXTRUDE_ERR8)"),
        ("_CFS_CMD_GET_SLAVE_INFO", 0x00,
         b"\xfe\xfe\x00\x11\x22\x33\x44\x55\x66\x77\x88\x99\xaa\xbb",
         {}, "uid=00112233445566778899aabb"),
        ("_CFS_CMD_SET_SLAVE_ADDR", 0x00,
         b"\x00\x11\x22\x33\x44\x55\x66\x77\x88\x99\xaa\xbb",
         {"NEW_ADDRESS": 5, "UID": "00112233445566778899aabb"},
         "uid=00112233445566778899aabb"),
        ("_CFS_CMD_ONLINE_CHECK", 0x00,
         b"\x00\x11\x22\x33\x44\x55\x66\x77\x88\x99\xaa\xbb",
         {}, "uid=00112233445566778899aabb"),
    ],
)
def test_hidden_query_commands_report_decoded_and_raw_responses(
        command, response_status, response_data, params, expected_text):
    cfs, gcode, sent, responses = make_cfs()
    function_name = command[len("_CFS_"):]
    function_code = creality_cfs.FN[function_name]
    response = creality_cfs.build_frame(
        1, response_status, function_code, response_data)
    responses[function_code] = response

    gcmd = invoke(gcode, command, **params)

    assert expected_text in gcmd.responses[0]
    assert "status=%#04x" % response_status in gcmd.responses[0]
    assert "raw=%s" % response.hex() in gcmd.responses[0]


def test_hidden_command_reports_explicit_empty_fields_when_box_does_not_reply():
    cfs, gcode, sent, responses = make_cfs()
    cfs._send = lambda *args, **kwargs: b""

    gcmd = invoke(gcode, "_CFS_GET_BOX_STATE")

    assert gcmd.responses == [
        "_CFS_GET_BOX_STATE: status=unknown data=(empty) raw=(empty)"]


def test_hidden_command_rejects_unknown_readable_value_before_sending():
    cfs, gcode, sent, responses = make_cfs()

    with pytest.raises(RuntimeError, match="ACTION must be one of"):
        invoke(gcode, "_CFS_CTRL_CONNECTION_MOTOR_ACTION", ACTION="FEED")

    assert sent == []


def test_set_slave_address_rejects_a_malformed_uid_before_sending():
    cfs, gcode, sent, responses = make_cfs()

    with pytest.raises(RuntimeError, match="UID must contain exactly 12 bytes"):
        invoke(gcode, "_CFS_CMD_SET_SLAVE_ADDR", NEW_ADDRESS=5, UID="1234")

    assert sent == []


@pytest.mark.parametrize(
    "command,params,error",
    [
        ("_CFS_SET_BOX_MODE", {"MODE": "IDLE", "SLOT": "E"},
         "SLOT must be one of"),
        ("_CFS_SET_PRE_LOADING", {"ACTION": "OPEN", "SLOTS": "AE"},
         "SLOTS must be ALL"),
        ("_CFS_EXTRUDE_PROCESS", {"SLOT": "A", "STAGE": 256},
         "STAGE must be between"),
        ("_CFS_CMD_SET_SLAVE_ADDR",
         {"NEW_ADDRESS": 254, "UID": "00112233445566778899aabb"},
         "NEW_ADDRESS must be between"),
        ("_CFS_RETRUDE_PROCESS", {"STAGE": 0}, "SLOT is required"),
    ],
)
def test_hidden_commands_reject_invalid_or_missing_parameters_before_sending(
        command, params, error):
    cfs, gcode, sent, responses = make_cfs()

    with pytest.raises(RuntimeError, match=error):
        invoke(gcode, command, **params)

    assert sent == []


@pytest.mark.parametrize(
    "command,params,expected_data",
    [
        ("_CFS_SET_BOX_MODE", {"MODE": "IDLE"}, b"\x00\x01"),
        ("_CFS_GET_FILAMENT_SENSOR_STATE", {}, b"\x00"),
        ("_CFS_SET_PRE_LOADING", {"ACTION": "OPEN"}, b"\x0f\x01"),
        ("_CFS_EXTRUDE_PROCESS", {"SLOT": "A", "STAGE": 5},
         b"\x01\x05\x00"),
        ("_CFS_MOVE_DISTANCE", {"DIRECTION": "FORWARD", "DISTANCE": 10},
         b"\x00\x0a"),
    ],
)
def test_hidden_commands_apply_their_documented_readable_defaults(
        command, params, expected_data):
    cfs, gcode, sent, responses = make_cfs()

    invoke(gcode, command, **params)

    assert sent[0][3] == expected_data
