# Hidden direct commands and configurable workflow

This repository exposes a hidden, low-level G-code surface for users who want
to build their own CFS process instead of relying on the wrapped `CFS_EXTRUDE`
and `CFS_RETRUDE` commands. The command names follow the shape documented by
FrederickAlt's
[`serial-protocol.md`](https://github.com/FrederickAlt/CREALITY-K1-AND-K1-MAX-CFS-RETRUDE-BEFORE-CUT-MOD/blob/master/docs/serial-protocol.md),
but slot-selecting parameters consistently use `SLOT=A|B|C|D` in this
repository. `macros/direct_toolchange.cfg` provides a configurable workflow
parallel to that project's
[`box.cfg`](https://github.com/FrederickAlt/CREALITY-K1-AND-K1-MAX-CFS-RETRUDE-BEFORE-CUT-MOD/blob/master/box.cfg).

## Compatibility boundary

Compatibility is at the command-intent level, not exact parameter spelling or
the raw byte level.

The referenced `serial_485` wrapper describes application packets beginning
with `ADDR`, and command IDs such as `GET_BOX_STATE=0x09`. This repository's
live-captured hardware protocol has an `0xf7` head byte, CRC-8 trailer, and
different command IDs, including `GET_BOX_STATE=0x0a`. The extra therefore
translates named parameters to the local `FN` table and `build_frame()` format.
Do not copy numeric packets from one implementation into the other.

There are two other deliberate translations:

- `_BOX_TIGHTEN_UP_ENABLE ENABLE=ENABLE` sends `0x01`, matching this
  repository's live-captured protocol. The referenced wrapper documents the
  opposite polarity.
- `_BOX_EXTRUDE_PROCESS ... STAGE=7` defaults `AMOUNT` to `3`, matching the
  independently corroborated local implementation. Other stages default it
  to `0`.

All commands start with `_` and are registered without help descriptions, so
they are omitted from normal `HELP` output. They remain callable from macros
and the console.

## Direct command reference

`ADDR` is optional on every command and defaults to `box_addr` from
`[creality_cfs]`. An explicit address is limited to `1..4`.

| Command | Parameters | Result |
|---|---|---|
| `_BOX_GET_BOX_STATE` | `[ADDR=<1..4>]` | Raw state response. |
| `_BOX_GET_VERSION_SN` | `[ADDR=<1..4>]` | ASCII version/serial when decodable, plus raw response. |
| `_BOX_GET_RFID` | `[ADDR=<1..4>] [SLOT=A\|B\|C\|D]` | RFID text when decodable. Omitting `SLOT` queries the all-slot mask. |
| `_BOX_GET_REMAIN_LEN` | `[ADDR=<1..4>] [SLOT=A\|B\|C\|D]` | Raw remaining-length bytes. Omitting `SLOT` queries the all-slot mask. |
| `_BOX_GET_BUFFER_STATE` | `[ADDR=<1..4>]` | Decodes `MIDDLE=0`, `FULL=1`, or `EMPTY=2`. |
| `_BOX_GET_FILAMENT_SENSOR_STATE` | `[ADDR=<1..4>] [POSITION=MATERIAL\|CONNECTIONS]` | Decodes the A-D sensor mask. |
| `_BOX_SET_BOX_MODE` | `[ADDR=<1..4>] MODE=PRINT\|IDLE [SLOT=A\|B\|C\|D]` | Sets per-slot print mode or generic idle mode when `SLOT` is omitted. |
| `_BOX_SET_PRE_LOADING` | `[ADDR=<1..4>] [SLOT=A\|B\|C\|D] ACTION=CLOSE\|OPEN\|RUN\|TIGHT` | Controls one slot, or all slots when `SLOT` is omitted. `RUN` and `TIGHT` use a 45-second timeout. |
| `_BOX_CTRL_CONNECTION_MOTOR_ACTION` | `[ADDR=<1..4>] ACTION=STOP\|EXTRUDE\|RETRUDE` | Controls the shared-path connection motor. |
| `_BOX_MEASURING_WHEEL` | `[ADDR=<1..4>] [ACTION=GET\|CLEAN]` | `GET` decodes the big-endian float distance. |
| `_BOX_TIGHTEN_UP_ENABLE` | `[ADDR=<1..4>] ENABLE=ENABLE\|DISABLE` | Controls box tensioning using local wire polarity. |
| `_BOX_EXTRUDE_PROCESS` | `[ADDR=<1..4>] SLOT=A\|B\|C\|D STAGE=0\|3\|4\|5\|6\|7 [AMOUNT=<0..255>]` | Sends exactly one staged load command. |
| `_BOX_RETRUDE_PROCESS` | `[ADDR=<1..4>] [SLOT=A\|B\|C\|D] [TRIGGER=BUFFER\|MATERIAL]` | Sends exactly one staged unload command; omitting `SLOT` uses the generic no-slot form. |
| `_BOX_MOVE_DISTANCE` | `[ADDR=<1..4>] [DIRECTION=FORWARD\|EXTRUDE\|RETRUDE\|REVERSE] DIST=<1..255> [TIMEOUT=<0.05..120>]` | Moves the box feed motor directly. |
| `_BOX_SEND_DATA` | `[ADDR=<1..4>] CMD=<0..255> [STATE=<0..255>] [TIMEOUT=<0.05..120>] [DATA=<hex>]` | Escape hatch for local command frames. Data is real hexadecimal: `DATA=0f01` and `DATA=0f,01` both send bytes `0f 01`. |

Every command reports the response status and raw frame. Known stable fields
are decoded in addition to the raw frame; uncertain fields are not guessed.

The referenced wrapper's `BOX_CREATE_CONNECT`, `BOX_EXTRUDE_2_PROCESS`, and
`BOX_COMMUNICATION_TEST` commands are not aliased. This repository uses a
different discovery/addressing exchange and has no live-validated equivalent
for the latter two commands. `_BOX_SEND_DATA` remains available for supervised
protocol work without pretending those command IDs are interchangeable.

## Installing the direct workflow

Install the Python extra normally, then copy and include the macro file:

```ini
[include direct_toolchange.cfg]
```

The file contains these sections:

- `CFS_DIRECT_CONFIG`: configuration-only macro holding XY coordinates,
  minimum travel Z, Z-hop, speeds, distances, stage polling, purge length,
  bucket-exit wipe tuning, sensor name, and homing command.
- `CFS_DIRECT_TOOLCHANGE`: public entry point.
- `_CFS_DIRECT_SEQUENCE`: post-home safe-Z calculation and workflow dispatch.
- `_CFS_DIRECT_UNLOAD`: buffer-aware pre-cut retract, cutter movement, and
  material-triggered unload.
- `_CFS_DIRECT_LOAD`: explicit connection, tension, stages 0/4/5/6/7,
  toolhead handoff, optional purge, print mode, and a high-acceleration
  outside-inside-final-out bucket exit.
- `_CFS_DIRECT_FINISH`: post-load sensor check and active-slot persistence.

`CFS_DIRECT_CONFIG` ships with every movement coordinate and `minimum_z` set
to `-1`. Copy the file to the printer's config directory, calibrate every
value on that printer, and change them there. `CFS_DIRECT_TOOLCHANGE` refuses
to run while any required value is negative.

Before any cutter or loading-position XY move, the post-home sequence computes:

```text
clearance_z = max(current_z, minimum_z) + z_hop
```

This means a toolchange that starts above `minimum_z` rises from its current
height instead of descending to the configured minimum. The same clearance is
used by unload and load, so the hop is not applied twice. If the required
height exceeds `[stepper_z] position_max`, the workflow aborts before either
helper is dispatched.

A `[save_variables]` section is required:

```ini
[save_variables]
filename: ~/printer_data/config/cfs_variables.cfg
```

Call the workflow directly while commissioning it:

```gcode
CFS_DIRECT_TOOLCHANGE FROM=A TO=B TEMP=220 PURGE=40
```

To unload without loading another slot, omit `TO`:

```gcode
CFS_DIRECT_TOOLCHANGE FROM=A TEMP=220
```

Unload-only mode clears the saved `cfs_active_slot` after the unload finishes.
It does not require calibrated extrude/bucket coordinates.

`FROM` may be omitted after the first successful change; the confirmed active
slot is saved as `cfs_active_slot`. A positive `TEMP` always sets and waits for
that exact hotend target before both unload and load, even when the hotend is
already above its cold-extrusion threshold. `PURGE` overrides the configured
purge length for one change.

`macros/direct_tool_aliases.cfg` optionally maps `T0` through `T3` to slots A
through D. Do not include it if another file already defines those commands.
It intentionally does not use `rename_existing`, so a conflict fails during
configuration instead of silently changing slicer behavior.

## Commissioning order

1. Verify `CFS_STATUS` and read-only `_BOX_GET_*` commands.
2. Calibrate cutter/retreat/extrude XY coordinates and `minimum_z` with
   supervised manual moves; choose a positive `z_hop` with adequate clearance.
3. Test individual motor and process commands with no print running.
4. Run `_CFS_DIRECT_LOAD` and `_CFS_DIRECT_UNLOAD` separately under direct
   supervision.
5. Only then use `CFS_DIRECT_TOOLCHANGE`, and only add T0-T3 aliases after the
   complete workflow is repeatable.

The extra still performs synchronous serial reads from Klipper's reactor.
Customising the macro reduces opaque process behavior but does not remove that
transport limitation.
