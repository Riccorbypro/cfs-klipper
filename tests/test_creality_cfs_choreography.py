"""Behavioral tests for safe purge-bucket entry and exit choreography."""

from types import SimpleNamespace

import pytest

from klipper_extra import creality_cfs


class RecordingGCode:
    def __init__(self, events=None):
        self.commands = []
        self.events = events

    def run_script_from_command(self, command):
        self.commands.append(command)
        if self.events is not None:
            self.events.append(("gcode", command))


class FakeToolhead:
    def __init__(self, z, max_accel=3000.0, gcode_z=None):
        self.position = [100.0, 100.0, z, 0.0]
        self.max_accel = max_accel
        self.gcode_z = z if gcode_z is None else gcode_z

    def get_position(self):
        return list(self.position)

    def get_status(self, eventtime):
        return {"homed_axes": "xyz", "max_accel": self.max_accel}


class FakeGCodeMove:
    def __init__(self, z):
        self.z = z

    def get_status(self, eventtime):
        return {"gcode_position": [100.0, 100.0, self.z, 0.0]}


class FakeGCmd:
    def __init__(self):
        self.responses = []

    def get(self, name, default=None):
        return "A" if name == "SLOT" else default

    def get_int(self, name, default, **kwargs):
        return 1 if name == "POLLS" else default

    def error(self, message):
        return RuntimeError(message)

    def respond_info(self, message):
        self.responses.append(message)


def make_cfs():
    cfs = object.__new__(creality_cfs.CrealityCFS)
    cfs.gcode = RecordingGCode()
    cfs.purge_min_z = 30.0
    cfs.purge_z_hop = 1.0
    cfs.purge_entry_x = 140.0
    cfs.purge_entry_y = 220.0
    cfs.purge_x = None
    cfs.purge_y = 225.0
    cfs.purge_move_speed = 1500.0
    cfs.purge_wipe_accel = 15000.0
    cfs.purge_wipe_speed = 12000.0
    cfs.purge_wipe_repetitions = 3
    return cfs


def make_command_cfs(gcode_z=35.0):
    cfs = make_cfs()
    events = []
    cfs.gcode = RecordingGCode(events)
    toolhead = FakeToolhead(z=35.0, gcode_z=gcode_z)
    gcode_move = FakeGCodeMove(gcode_z)
    cfs.reactor = SimpleNamespace(monotonic=lambda: 100.0)

    def lookup_object(name):
        return {"toolhead": toolhead, "gcode_move": gcode_move}.get(name)

    cfs.printer = SimpleNamespace(lookup_object=lookup_object)
    cfs.box_addr = 1
    cfs._reset_pre_loading = lambda: None
    cfs._pause = lambda seconds: None
    sent = []

    def record_send(addr, status, function_code, data=b"", **kwargs):
        sent.append((function_code, data))
        events.append(("send", function_code, data))
        return b""

    cfs._send = record_send
    return cfs, events, sent


def test_bucket_entry_raises_low_toolhead_to_minimum_before_xy_travel():
    cfs = make_cfs()

    cfs._approach_purge_bucket(20.0)
    cfs._move_into_purge_bucket(cfs.purge_move_speed)
    cfs.gcode.run_script_from_command("M400")

    assert cfs.gcode.commands == [
        "G90",
        "G1 Z30.00 F1500",
        "M400",
        "G1 X140.00 Y220.00 F1500",
        "M400",
        "G1 Y225.00 F1500",
        "M400",
    ]


def test_bucket_entry_adds_zhop_when_toolhead_is_already_high_enough():
    cfs = make_cfs()

    cfs._approach_purge_bucket(35.0)

    assert cfs.gcode.commands[1] == "G1 Z36.00 F1500"


def test_bucket_entry_uses_gcode_z_when_coordinate_offset_is_active():
    cfs, events, sent = make_command_cfs(gcode_z=15.0)
    cfs._load_filament_in_bucket = lambda gcmd, slot, polls: True

    cfs.cmd_CFS_EXTRUDE(FakeGCmd())

    assert "G1 Z30.00 F1500" in cfs.gcode.commands
    assert "G1 Z36.00 F1500" not in cfs.gcode.commands


def test_extrude_reports_missing_required_bucket_position():
    cfs, events, sent = make_command_cfs()
    cfs.purge_entry_x = None

    with pytest.raises(RuntimeError, match="bucket geometry"):
        cfs.cmd_CFS_EXTRUDE(FakeGCmd())

    assert sent == []


def test_bucket_entry_only_moves_configured_axis_when_entering_bucket():
    cfs = make_cfs()
    cfs.purge_x = 150.0
    cfs.purge_y = None

    cfs._approach_purge_bucket(35.0)
    cfs._move_into_purge_bucket(cfs.purge_move_speed)
    cfs.gcode.run_script_from_command("M400")

    assert cfs.gcode.commands[-2:] == [
        "G1 X150.00 F1500",
        "M400",
    ]


def test_bucket_exit_breaks_strands_three_times_and_ends_outside():
    cfs = make_cfs()

    cfs._exit_purge_bucket(original_accel=3000.0)

    assert cfs.gcode.commands == [
        "SET_VELOCITY_LIMIT ACCEL=15000",
        "G1 X140.00 Y220.00 F12000",
        "G1 Y225.00 F12000",
        "G1 X140.00 Y220.00 F12000",
        "G1 Y225.00 F12000",
        "G1 X140.00 Y220.00 F12000",
        "G1 Y225.00 F12000",
        "G1 X140.00 Y220.00 F12000",
        "M400",
        "SET_VELOCITY_LIMIT ACCEL=3000",
        "RESTORE_GCODE_STATE NAME=CFS_EXTRUDE",
    ]


def test_bucket_exit_restores_acceleration_and_state_when_wipe_move_fails():
    cfs = make_cfs()
    commands = []

    def fail_inside_bucket(command):
        if command == "G1 Y225.00 F12000":
            raise RuntimeError("simulated move failure")
        commands.append(command)

    cfs.gcode.run_script_from_command = fail_inside_bucket

    with pytest.raises(RuntimeError, match="simulated move failure"):
        cfs._exit_purge_bucket(original_accel=3000.0)

    assert commands[-2:] == [
        "SET_VELOCITY_LIMIT ACCEL=3000",
        "RESTORE_GCODE_STATE NAME=CFS_EXTRUDE",
    ]


def test_bucket_exit_preserves_fractional_acceleration():
    cfs = make_cfs()

    cfs._exit_purge_bucket(original_accel=3000.123456)

    assert cfs.gcode.commands[-2:] == [
        "SET_VELOCITY_LIMIT ACCEL=3000.123456",
        "RESTORE_GCODE_STATE NAME=CFS_EXTRUDE",
    ]


def test_bucket_exit_attempts_evacuation_when_acceleration_setup_fails():
    cfs = make_cfs()

    def fail_acceleration_once(command):
        if command == "SET_VELOCITY_LIMIT ACCEL=15000":
            raise RuntimeError("simulated acceleration failure")
        cfs.gcode.commands.append(command)

    cfs.gcode.run_script_from_command = fail_acceleration_once

    with pytest.raises(RuntimeError, match="simulated acceleration failure"):
        cfs._exit_purge_bucket(original_accel=3000.0)

    assert cfs.gcode.commands[-2:] == [
        "M400",
        "RESTORE_GCODE_STATE NAME=CFS_EXTRUDE",
    ]
    assert "G1 X140.00 Y220.00 F12000" in cfs.gcode.commands


def test_bucket_exit_retries_final_evacuation_after_wipe_entry_fails():
    cfs = make_cfs()
    entry_move = "G1 X140.00 Y220.00 F12000"
    entry_calls = 0

    def fail_second_entry_once(command):
        nonlocal entry_calls
        if command == entry_move:
            entry_calls += 1
            if entry_calls == 2:
                raise RuntimeError("simulated wipe entry failure")
        cfs.gcode.commands.append(command)

    cfs.gcode.run_script_from_command = fail_second_entry_once

    with pytest.raises(RuntimeError, match="simulated wipe entry failure"):
        cfs._exit_purge_bucket(original_accel=3000.0)

    assert entry_calls == 3
    assert cfs.gcode.commands[-3:] == [
        "M400",
        "SET_VELOCITY_LIMIT ACCEL=3000",
        "RESTORE_GCODE_STATE NAME=CFS_EXTRUDE",
    ]


def test_extrude_exits_bucket_and_restores_state_when_loading_fails():
    cfs, events, sent = make_command_cfs()

    def fail_handoff(gcmd, slot):
        raise RuntimeError("simulated load failure")

    cfs._extrude_material_handoff = fail_handoff

    with pytest.raises(RuntimeError, match="simulated load failure"):
        cfs.cmd_CFS_EXTRUDE(FakeGCmd())

    assert cfs.gcode.commands[-2:] == [
        "SET_VELOCITY_LIMIT ACCEL=3000",
        "RESTORE_GCODE_STATE NAME=CFS_EXTRUDE",
    ]
    assert cfs.gcode.commands[-4] == "G1 X140.00 Y220.00 F12000"
    assert sent[-3:] == [
        (creality_cfs.FN["TIGHTEN_UP_ENABLE"], bytes([0x00])),
        (creality_cfs.FN["CTRL_CONNECTION_MOTOR_ACTION"], bytes([0x00])),
        (creality_cfs.FN["SET_BOX_MODE"], bytes([0x00, 0x01])),
    ]
    bucket_entry = events.index(("gcode", "G1 Y225.00 F1500"))
    load_start = events.index((
        "send", creality_cfs.FN["CTRL_CONNECTION_MOTOR_ACTION"],
        bytes([0x01])))
    cleanup_done = len(events) - 1 - events[::-1].index((
        "send", creality_cfs.FN["SET_BOX_MODE"], bytes([0x00, 0x01])))
    wipe_start = events.index(("gcode", "SET_VELOCITY_LIMIT ACCEL=15000"))
    assert bucket_entry < load_start
    assert cleanup_done < wipe_start


def test_extrude_restores_state_without_wiping_if_safe_approach_fails():
    cfs, events, sent = make_command_cfs()

    def fail_z_move(command):
        if command == "G1 Z36.00 F1500":
            raise RuntimeError("simulated approach failure")
        cfs.gcode.commands.append(command)

    cfs.gcode.run_script_from_command = fail_z_move

    with pytest.raises(RuntimeError, match="simulated approach failure"):
        cfs.cmd_CFS_EXTRUDE(FakeGCmd())

    assert cfs.gcode.commands[-1] == "RESTORE_GCODE_STATE NAME=CFS_EXTRUDE"
    assert not any(command.startswith("SET_VELOCITY_LIMIT ACCEL=15000")
                   for command in cfs.gcode.commands)


def test_extrude_exits_bucket_when_entry_completion_wait_fails():
    cfs, events, sent = make_command_cfs()
    m400_calls = 0

    def fail_inside_m400_once(command):
        nonlocal m400_calls
        if command == "M400":
            m400_calls += 1
            if m400_calls == 3:
                raise RuntimeError("simulated bucket entry wait failure")
        cfs.gcode.commands.append(command)

    cfs.gcode.run_script_from_command = fail_inside_m400_once

    with pytest.raises(RuntimeError, match="simulated bucket entry wait failure"):
        cfs.cmd_CFS_EXTRUDE(FakeGCmd())

    assert cfs.gcode.commands[-2:] == [
        "SET_VELOCITY_LIMIT ACCEL=3000",
        "RESTORE_GCODE_STATE NAME=CFS_EXTRUDE",
    ]


def test_box_cleanup_attempts_all_commands_and_preserves_load_error():
    cfs, events, sent = make_command_cfs()

    def fail_handoff(gcmd, slot):
        raise RuntimeError("original load failure")

    cfs._extrude_material_handoff = fail_handoff
    original_send = cfs._send

    def fail_first_cleanup(addr, status, function_code, data=b"", **kwargs):
        if (function_code == creality_cfs.FN["TIGHTEN_UP_ENABLE"]
                and data == bytes([0x00])):
            sent.append((function_code, data))
            raise RuntimeError("cleanup failure")
        return original_send(addr, status, function_code, data, **kwargs)

    cfs._send = fail_first_cleanup

    with pytest.raises(RuntimeError, match="original load failure"):
        cfs.cmd_CFS_EXTRUDE(FakeGCmd())

    assert (creality_cfs.FN["CTRL_CONNECTION_MOTOR_ACTION"], bytes([0x00])) in sent
    assert (creality_cfs.FN["SET_BOX_MODE"], bytes([0x00, 0x01])) in sent


def test_extrude_preserves_load_error_when_bucket_exit_also_fails():
    cfs, events, sent = make_command_cfs()

    def fail_load(gcmd, slot, polls):
        raise RuntimeError("original load failure")

    def fail_exit(original_accel):
        raise RuntimeError("exit failure")

    cfs._load_filament_in_bucket = fail_load
    cfs._exit_purge_bucket = fail_exit

    with pytest.raises(RuntimeError, match="original load failure"):
        cfs.cmd_CFS_EXTRUDE(FakeGCmd())


def test_extrude_rejects_zero_distance_bucket_entry():
    cfs, events, sent = make_command_cfs()
    cfs.purge_y = cfs.purge_entry_y

    with pytest.raises(RuntimeError, match="bucket geometry"):
        cfs.cmd_CFS_EXTRUDE(FakeGCmd())

    assert sent == []


def test_extrude_rejects_bucket_entry_that_rounds_to_zero_distance():
    cfs, events, sent = make_command_cfs()
    cfs.purge_entry_y = 220.001
    cfs.purge_y = 220.002

    with pytest.raises(RuntimeError, match="bucket geometry"):
        cfs.cmd_CFS_EXTRUDE(FakeGCmd())

    assert sent == []
