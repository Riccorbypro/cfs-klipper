"""Rendered-behavior tests for the configurable direct CFS workflow."""

import configparser
import os
from types import SimpleNamespace

import jinja2


MACRO_PATH = os.path.join(
    os.path.dirname(__file__), "..", "macros", "direct_toolchange.cfg")


def _load_template(section):
    with open(MACRO_PATH, encoding="utf-8") as config_file:
        raw = config_file.read()
    stripped = "\n".join(line.split("#", 1)[0] for line in raw.splitlines())
    parser = configparser.RawConfigParser(
        strict=False, inline_comment_prefixes=(";", "#"))
    parser.read_string(stripped)
    env = jinja2.Environment("{%", "%}", "{", "}")
    return env.from_string(parser.get(section, "gcode"))


def test_direct_load_renders_named_stages_and_configured_poll_count():
    """Replacing named direct stages with a wrapped load breaks custom control."""
    template = _load_template("gcode_macro _CFS_DIRECT_LOAD")
    config = SimpleNamespace(
        extrude_x=159.0,
        extrude_y=217.5,
        travel_speed=1500.0,
        extrude_poll_count=3,
        poll_delay_ms=400,
        prime_e1=10.0,
        prime_e1_speed=35.0,
        prime_e2=5.0,
        prime_e2_speed=10.0,
        handoff_push=9.0,
        handoff_speed=12000.0,
        purge_length=0.0,
        purge_speed=500.0,
    )
    rendered = template.render(
        params={"TO": "C", "SAFE_Z": "82.0"},
        printer={
            "gcode_macro CFS_DIRECT_CONFIG": config,
            "extruder": SimpleNamespace(can_extrude=True),
        },
        action_raise_error=lambda message: "ERROR %s" % message,
        action_respond_info=lambda message: "INFO %s" % message,
    )

    assert "_BOX_EXTRUDE_PROCESS NUM=C STAGE=0 AMOUNT=0" in rendered
    assert "G1 Z82.0 F1500.0" in rendered
    assert "_BOX_EXTRUDE_PROCESS NUM=C STAGE=4 AMOUNT=0" in rendered
    assert rendered.count("_BOX_EXTRUDE_PROCESS NUM=C STAGE=5 AMOUNT=0") == 3
    assert "_BOX_EXTRUDE_PROCESS NUM=C STAGE=7 AMOUNT=3" in rendered
    assert "_BOX_SET_BOX_MODE NUM=C MODE=PRINT" in rendered


def test_direct_toolchange_rejects_uncalibrated_motion_coordinates():
    """Sentinel coordinates must stop the workflow before any movement is emitted."""
    template = _load_template("gcode_macro CFS_DIRECT_TOOLCHANGE")
    config = SimpleNamespace(
        cut_x=-1.0,
        cut_y=-1.0,
        retreat_x=-1.0,
        retreat_y=-1.0,
        extrude_x=-1.0,
        extrude_y=-1.0,
        minimum_z=-1.0,
        toolhead_sensor_name="filament_sensor_2",
        home_command="G28",
    )
    rendered = template.render(
        params={"TO": "A"},
        printer={
            "gcode_macro CFS_DIRECT_CONFIG": config,
            "save_variables": SimpleNamespace(variables={}),
            "toolhead": SimpleNamespace(homed_axes="xyz"),
        },
        action_raise_error=lambda message: "ERROR %s" % message,
        action_respond_info=lambda message: "INFO %s" % message,
    )

    assert "ERROR CFS_DIRECT_TOOLCHANGE: calibrate cut, retreat, extrude, and minimum-Z settings" in rendered
    assert "_CFS_DIRECT_SEQUENCE" not in rendered


def test_direct_load_stops_before_box_motion_when_hotend_is_cold():
    """A first load must not start CFS motors before cold extrusion is rejected."""
    template = _load_template("gcode_macro _CFS_DIRECT_LOAD")
    config = SimpleNamespace(
        extrude_x=159.0,
        extrude_y=217.5,
        travel_speed=1500.0,
        extrude_poll_count=3,
        poll_delay_ms=400,
        prime_e1=10.0,
        prime_e1_speed=35.0,
        prime_e2=5.0,
        prime_e2_speed=10.0,
        handoff_push=9.0,
        handoff_speed=12000.0,
        purge_length=0.0,
        purge_speed=500.0,
    )
    rendered = template.render(
        params={"TO": "A", "TEMP": "0", "SAFE_Z": "52.0"},
        printer={
            "gcode_macro CFS_DIRECT_CONFIG": config,
            "extruder": SimpleNamespace(can_extrude=False),
        },
        action_raise_error=lambda message: "ERROR %s" % message,
        action_respond_info=lambda message: "INFO %s" % message,
    )

    assert "ERROR _CFS_DIRECT_LOAD: hotend is too cold" in rendered
    assert "_BOX_" not in rendered


def test_direct_unload_rejects_non_positive_buffer_chunk():
    """Invalid chunk tuning must raise a useful error instead of dividing by zero."""
    template = _load_template("gcode_macro _CFS_DIRECT_UNLOAD")
    config = SimpleNamespace(
        pre_cut_retract=27.0,
        buffer_chunk=0.0,
        pre_cut_retract_speed=1980.0,
        cut_x=36.0,
        cut_y=227.0,
        retreat_x=36.0,
        retreat_y=200.0,
        travel_speed=1500.0,
        cut_speed=1200.0,
        post_cut_retract=60.0,
        post_cut_retract_speed=500.0,
    )
    rendered = template.render(
        params={"FROM": "A", "TEMP": "220", "SAFE_Z": "52.0"},
        printer={
            "gcode_macro CFS_DIRECT_CONFIG": config,
            "extruder": SimpleNamespace(can_extrude=True),
        },
        action_raise_error=lambda message: "ERROR %s" % message,
        action_respond_info=lambda message: "INFO %s" % message,
    )

    assert "ERROR _CFS_DIRECT_UNLOAD: buffer_chunk must be greater than zero" in rendered
    assert "_BOX_" not in rendered


def _render_direct_sequence(current_z, minimum_z=50.0, z_hop=2.0,
                            position_max=250.0):
    template = _load_template("gcode_macro _CFS_DIRECT_SEQUENCE")
    config = SimpleNamespace(
        minimum_z=minimum_z,
        z_hop=z_hop,
        purge_length=0.0,
    )
    return template.render(
        params={"FROM": "A", "TO": "B", "TEMP": "220", "PURGE": "0"},
        printer={
            "gcode_macro CFS_DIRECT_CONFIG": config,
            "toolhead": SimpleNamespace(
                position=SimpleNamespace(z=current_z)),
            "configfile": SimpleNamespace(
                settings=SimpleNamespace(
                    stepper_z=SimpleNamespace(position_max=position_max))),
        },
        action_raise_error=lambda message: "ERROR %s" % message,
        action_respond_info=lambda message: "INFO %s" % message,
    )


def test_direct_sequence_uses_minimum_z_plus_hop_when_current_z_is_lower():
    """A low toolhead must rise above the configured minimum before XY travel."""
    rendered = _render_direct_sequence(current_z=10.0)
    assert "_CFS_DIRECT_UNLOAD FROM=A TEMP=220 SAFE_Z=52.0" in rendered
    assert "_CFS_DIRECT_LOAD TO=B TEMP=220 PURGE=0 SAFE_Z=52.0" in rendered


def test_direct_sequence_hops_above_current_z_when_already_above_minimum():
    """Starting high must never cause a downward move to the configured minimum."""
    rendered = _render_direct_sequence(current_z=80.0)
    assert "_CFS_DIRECT_UNLOAD FROM=A TEMP=220 SAFE_Z=82.0" in rendered
    assert "_CFS_DIRECT_LOAD TO=B TEMP=220 PURGE=0 SAFE_Z=82.0" in rendered


def test_direct_sequence_rejects_clearance_above_z_axis_limit():
    """The required hop must fail before motion rather than exceed Z maximum."""
    rendered = _render_direct_sequence(current_z=249.0, position_max=250.0)
    assert "ERROR _CFS_DIRECT_SEQUENCE: required clearance Z251.000 exceeds Z maximum 250.000" in rendered
    assert "_CFS_DIRECT_UNLOAD" not in rendered
    assert "_CFS_DIRECT_LOAD" not in rendered
