# Klipper extra

`creality_cfs.py` wraps the protocol validated elsewhere in this repo into
a real Klipper extra — gcode commands and a background status poll,
instead of standalone scripts you run by hand.

**Hardware status:** the earlier integration was confirmed end-to-end on
2026-08-16, including a slot switch, but the 2026-08-26 non-blocking serial
transport and purge-bucket choreography rewrites are regression-tested only.
Repeat the supervised checks below before trusting them unattended.
`cmd_CFS_TOOLCHANGE` (the combined swap macro) is still the next thing to
exercise as a whole.

The serial transport is now integrated with Klipper's reactor: pyserial is
non-blocking, its file descriptor is registered with `reactor.register_fd()`,
and command handlers wait on reactor completions rather than blocking the
single reactor thread. This is regression-tested without hardware; repeat the
supervised live checks below after installing the updated extra.

## Addressing bug - found and fixed (2026-08-16)

For a while, the box never got marked as addressed inside this extra at
all - `CFS_STATUS` always said "not addressed", even though `cfs_cli.py
status` reliably worked seconds apart on the same box. A long diagnostic
session (byte-level TX/RX logging, testing under klippy-env's own Python
standalone with Klipper fully stopped, testing the real `cfs_protocol.py`
code directly) ruled out every "execution context" theory - Klipper's
reactor, a persistent vs. fresh connection, even the interpreter/venv
itself all turned out to be red herrings.

**Real cause: this box remembers its RS-485 address across power
cycles.** Once addressed, it simply stops replying to broadcast discovery
(`CMD_GET_SLAVE_INFO` at the broadcast address) - confirmed by calling
`CFSClient.discover()` directly from a plain script and getting no reply
either. It answers a direct query at its already-known address (`0x01`
here) instantly. `cfs_cli.py status` always "worked" only because it
never does discovery in the first place - it just talks straight to the
known address. This extra's old `_discover_and_address()` insisted on
broadcast-first with no fallback, so it failed every single time on an
already-addressed box.

**Fix:** `_discover_and_address()` now tries a cheap direct probe
(`CMD_ONLINE_CHECK`) at `box_addr` first, and only falls back to full
broadcast discovery if that gets no reply (a genuinely fresh/unaddressed
box). Confirmed live: `klippy.log` now shows `box already addressed at
0x01, skipped broadcast discovery (direct probe replied)`, and
`CFS_STATUS` correctly reports `material loaded in slots: A, B, C, D`
through the real gcode command path.

## Two real Klipper gotchas found installing this, worth knowing regardless of this project

1. **Never use Jinja2 `{# comment #}` syntax inside a macro's `gcode:`
   block.** Klipper's config loader strips everything after the *first*
   `#` on every raw line - including inside a multi-line `gcode:` value -
   before Jinja2 ever sees it. A Jinja2 comment's own `#` characters trip
   this, silently truncating the line and producing a confusing
   `jinja2.exceptions.TemplateSyntaxError: unexpected 'end of template'`
   pointing nowhere near the real cause. Hit this in two of this repo's
   own macro drafts - both fixed by just removing the `{# #}` comments.
2. **On at least this printer's community firmware stack (Guilouz
   Helper-Script), Moonraker's `firmware_restart`/`restart` API calls did
   not reliably reload edited Python extras** - the `klippy.py` process
   ID stayed the same across "restarts", and code changes weren't picked
   up. What did work: `kill <klippy.py pid>` followed by
   `/etc/init.d/S55klipper_service start` (find the exact service name
   with `ls /etc/init.d/ | grep -i klip` on your unit). Worth checking
   for if your own extra edits don't seem to take effect after a normal
   restart.

## Install

```bash
cp creality_cfs.py ~/klipper/klippy/extras/creality_cfs.py
```

Add to `printer.cfg`:

```ini
[creality_cfs]
serial: /dev/ttyUSB0
baud: 230400
box_addr: 1

# Calibrate these on your printer; do not copy coordinates from another unit.
purge_min_z: <minimum safe bucket travel Z>
purge_entry_x: <X safely outside the bucket>
purge_entry_y: <Y safely outside the bucket>
# Configure one or both axes for the move from the entry point into the bucket.
purge_y: <Y inside the bucket>

# Name of the real toolhead filament switch used to confirm load/unload.
toolhead_sensor_name: filament_sensor_2

# Optional motion tuning defaults:
# purge_z_hop: 1
# purge_move_speed: 1500
# purge_wipe_accel: 15000
# purge_wipe_speed: 12000
# purge_wipe_repetitions: 3
```

Restart Klipper (see the gotcha above if changes don't seem to apply),
then test read-only first:

```
CFS_STATUS
```

For a lower-level, box.cfg-style workflow, this extra also registers hidden
underscore-prefixed `_BOX_*` commands with typed parameters for individual
box-mode, sensor, motor, preloading, staged extrude/retrude, measuring-wheel,
and raw-data operations. They translate the upstream-style command surface to
this repository's different, live-validated function IDs and F7/CRC8 framing.
See [`docs/DIRECT_COMMANDS.md`](../docs/DIRECT_COMMANDS.md) for the full command
reference and the separately installable `macros/direct_toolchange.cfg`
workflow. Both direct loading and `CFS_EXTRUDE` now keep stage 5 running until
the real toolhead sensor confirms arrival, with `POLLS` as a bounded failure
limit rather than a completion count. `CFS_EXTRUDE` also refuses to enter
PRINT mode unless its buffer/measuring-wheel handoff check confirms success.

If it says "not addressed" (e.g. a genuinely first-ever run with a fresh
box), try `CFS_RECONNECT`, which now falls back to full broadcast
discovery automatically. Only move on to `CFS_RETRUDE SLOT=A` /
`CFS_EXTRUDE SLOT=A` once status genuinely works and you're watching the
printer.

**`CFS_RETRUDE` ✅ confirmed working as a real Klipper extra command**
(2026-08-16) - full tip-form unload sequence, ended with "toolhead sensor
clear, unload complete" and "confirmed clear", no manual assist needed.

**`CFS_EXTRUDE` ✅ was confirmed working as a real Klipper extra command**
(2026-08-16) - `CFS_EXTRUDE SLOT=B` completed cleanly under the old fixed
20-poll behavior, a genuine slot switch through the actual gcode command path.
The current sensor-gated stage-5 behavior is regression-tested in the
repository but still needs supervised live confirmation. Getting the original
command working took finding and fixing two real problems live:

1. **A reactor-stall heater fault.** This extra's accumulated
   `time.sleep()` calls and blocking serial reads
   stalled Klipper's reactor long enough that `verify_heater` missed its
   update window and tripped a false "not heating at expected rate"
   shutdown mid-run. Fixed in two stages: every `time.sleep()` became
   `self._pause()` (`reactor.pause()`, cooperative), which was confirmed live,
   and `_send()`'s blocking `ser.read()` loop was replaced on 2026-08-26 by a
   `reactor.register_fd()` callback plus cooperative completion waits. The
   serial rewrite is regression-tested but still needs supervised confirmation
   on the printer.
2. **A real physical collision.** The "go to extrude position" move used
   to default to `X148 Y225.3 Z30`, copied verbatim (never physically
   tested by us) from a factory `box.cfg` found on a different K1C. Live
   on this printer, that move crashed the toolhead into the frame near an
   overhead camera mount - the user had to hit emergency stop.
   `CFS_EXTRUDE` now requires a calibrated `purge_min_z` and a distinct
   `purge_entry_x`/`purge_entry_y` safely outside the bucket. It first raises
   Z to that minimum (or adds a 1mm hop when already above it), travels to the
   entry point, and only then moves the configured `purge_x` and/or `purge_y`
   axis into the bucket. After loading, three high-acceleration in/out moves
   break nozzle strands and finish outside before acceleration and G-code
   state are restored. These coordinates remain printer-specific: calibrate
   your own and do not copy values from another machine.

`BOX_NOZZLE_CLEAN` and stage 7's exact 3rd payload byte remain
unconfirmed guesses, but didn't block the live result.

## Fluidd / Mainsail display

You don't need a custom panel for this. The extra registers one small
object per slot named `filament_switch_sensor CFS_A` (through `CFS_D`),
matching the exact shape Klipper's built-in filament sensor uses
(`filament_detected` + `enabled`). Fluidd and Mainsail already know how to
display any object with that name pattern in their normal filament-sensor
UI — so once this extra is loaded and polling, slots A–D should just show
up there like any other runout sensor, updating live as material is
loaded/unloaded. Change the `CFS_` prefix via `sensor_name_prefix:` in the
config if you want different names.

This part hasn't been visually confirmed in a real Fluidd session yet
(same "written, not yet tested" caveat as the rest of this file) — the
object shape and naming convention are correct per Klipper's own
`filament_switch_sensor` implementation, but it's worth a quick look the
first time you load this to confirm it renders as expected.
