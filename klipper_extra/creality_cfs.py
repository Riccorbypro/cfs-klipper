# Creality CFS support for Klipper
#
# Wraps the protocol validated in cfs_protocol.py / docs/PROTOCOL.md into
# a real Klipper extra: gcode commands + a periodic status poll.
#
# STATUS: the underlying protocol (framing, CRC8, command bytes) is
# live-validated against real hardware - see the repo root README and
# docs/PROTOCOL.md. This *file* - the Klipper integration itself (config
# parsing, gcode command registration, reactor timer) - has NOT yet been
# loaded into a running Klipper and tested. Review it, install it
# carefully, and test each command individually before relying on it.
#
# SERIAL TRANSPORT: CFS I/O is reactor-driven and non-blocking. Pyserial is
# opened with zero read/write timeouts and its file descriptor is registered
# with reactor.register_fd(). _send() parks only its calling greenlet on a
# reactor completion while the fd callback assembles the response, leaving
# MCU keepalives, motion, LEDs, heater watchdogs, and other reactor work free
# to run. A reactor.mutex() serializes the half-duplex bus so the periodic poll
# cannot overwrite a gcode command's pending response. This replaces the old
# blocking ser.read() loop that caused live verify_heater/reactor stalls.
#
# Installation: copy this file into klipper/klippy/extras/creality_cfs.py
# on the printer, add a [creality_cfs] section to printer.cfg (see example
# below), then restart Klipper.
#
# Example printer.cfg section:
#
#   [creality_cfs]
#   serial: /dev/ttyUSB0
#   baud: 230400
#   box_addr: 1
#   # REQUIRED before CFS_EXTRUDE will run. Calibrate these live on YOUR
#   # printer. The entry point is safely outside the purge bucket; purge_x
#   # and/or purge_y then moves inward to actuate the bucket mechanism:
#   purge_min_z: <minimum safe travel height>
#   purge_entry_x: <safe X outside bucket>
#   purge_entry_y: <safe Y outside bucket>
#   purge_y: <Y inside bucket; omit purge_x to retain entry X>
#   # Name of your real toolhead [filament_switch_sensor]. Sensor-gated
#   # CFS_EXTRUDE requires this to resolve; CFS_RETRUDE uses it too:
#   toolhead_sensor_name: filament_sensor_2

import logging
import math
import os
import struct

try:
    import serial
except ImportError:
    serial = None


FN = {
    "GET_RFID": 0x02,
    "GET_REMAIN_LEN": 0x03,
    "SET_BOX_MODE": 0x04,
    "GET_BUFFER_STATE": 0x05,
    "CTRL_CONNECTION_MOTOR_ACTION": 0x07,
    "GET_FILAMENT_SENSOR_STATE": 0x08,
    "GET_BOX_STATE": 0x0A,
    "SET_PRE_LOADING": 0x0D,
    "GET_MEASURING_WHEEL": 0x0E,
    "TIGHTEN_UP_ENABLE": 0x0F,
    "EXTRUDE_PROCESS": 0x10,
    "RETRUDE_PROCESS": 0x11,
    "GET_VERSION_SN": 0x14,
    "MOVE_DISTANCE": 0x31,
    "CMD_SET_SLAVE_ADDR": 0xA0,
    "CMD_GET_SLAVE_INFO": 0xA1,
    "CMD_ONLINE_CHECK": 0xA2,
}

SLOT_BYTES = {"A": 0x01, "B": 0x02, "C": 0x04, "D": 0x08}
BROADCAST_ALL_BOXES = 0xFE
# The reference implementation caps CFS payloads at 100 bytes. Responses in
# this protocol are much smaller in practice; bounding the length also lets
# the stream parser recover from a stray 0xF7 followed by a bogus length byte.
MAX_RESPONSE_DATA = 100


def format_gcode_number(value):
    """Serialize a numeric setting without losing meaningful precision."""
    value = float(value)
    return str(int(value)) if value.is_integer() else repr(value)

# "Tip-forming" toolhead move sequence for a clean, non-jamming unload -
# see the identical table (and its full rationale) in cfs_protocol.py's
# TIP_FORM_STEPS. Duplicated here rather than imported since this file
# is meant to be self-contained when copied into klippy/extras/.
TIP_FORM_STEPS = [
    (0.5, 600), (-5, 600), (2.5, 600), (-1.25, 600), (1.75, 600), (1, 60),
    (-15, 90), (-15, 90), (-15, 500), (-15, 500), (-15, 500), (-15, 500),
]


class CFSSlotSensor:
    """A minimal object matching the shape of Klipper's built-in
    filament_switch_sensor (filament_detected + enabled). Registered under
    the name "filament_switch_sensor CFS_<slot>" so Fluidd/Mainsail pick it
    up in their normal filament-sensor UI automatically - no custom
    frontend needed. See klipper_extra/README.md for what this looks like.
    """

    def __init__(self):
        self.filament_detected = False
        self.enabled = True

    def get_status(self, eventtime):
        return {"filament_detected": self.filament_detected, "enabled": self.enabled}


def crc8(data):
    crc = 0
    for byte in bytearray(data):
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07 if crc & 0x80 else crc << 1) & 0xFF
    return crc


def decode_measuring_wheel(data):
    """Decode a 4-byte measuring-wheel/odometer reading - big-endian
    IEEE-754 float, mm, negative and growing in magnitude while material
    actively feeds. Duplicated from cfs_protocol.py (same reasoning as
    TIP_FORM_STEPS - self-contained file). Confirmed correct against our
    own real EXTRUDE_PROCESS telemetry, and independently re-derived
    2026-08-17 from decompiled reference firmware's own
    get_measuring_wheel() - its convoluted big-endian-int-then-repack-
    little-endian-float dance is mathematically identical to a plain
    big-endian float unpack, see FINDINGS.md."""
    if len(data) != 4:
        return None
    return struct.unpack(">f", data)[0]


def build_frame(slave_addr, status, function_code, data=b""):
    length = 1 + 1 + len(data) + 1
    body = bytes([length, status, function_code]) + bytes(data)
    crc = crc8(body)
    return bytes([0xF7, slave_addr]) + body + bytes([crc])


class CrealityCFS:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")

        if serial is None:
            raise config.error(
                "creality_cfs: pyserial is not importable in this Klipper "
                "environment - install it in klippy-env before using this extra")

        self.serial_path = config.get("serial", "/dev/ttyUSB0")
        self.baud = config.getint("baud", 230400)
        self.box_addr = config.getint("box_addr", 1)
        self.poll_interval = config.getfloat("poll_interval", 5.0, above=0.0)

        # Safe purge-bucket entry before EXTRUDE_PROCESS.
        #
        # SAFETY INCIDENT 2026-08-16: this used to default to X148/Y225.3/
        # Z30, copied verbatim (never physically tested by us) from a
        # factory box.cfg found on a different K1C. Live on THIS printer,
        # that move crashed the toolhead into the frame/enclosure near
        # where an overhead camera is mounted - the user had to hit
        # emergency stop. No injury/damage beyond a startled camera mount,
        # but this is a real collision hazard, not a theoretical one.
        # Positions have no defaults: the bucket's location and safe travel
        # height are printer-specific. purge_x/purge_y are individually
        # optional because a cardinally-mounted bucket usually needs motion
        # on only one axis; CFS_EXTRUDE requires at least one of them.
        self.purge_min_z = config.getfloat("purge_min_z", None, minval=0.0)
        self.purge_entry_x = config.getfloat("purge_entry_x", None)
        self.purge_entry_y = config.getfloat("purge_entry_y", None)
        self.purge_x = config.getfloat("purge_x", None)
        self.purge_y = config.getfloat("purge_y", None)
        self.purge_z_hop = config.getfloat("purge_z_hop", 1.0, above=0.0)
        # Keep entry travel conservative; the strand-breaking exit has its
        # own faster configured speed.
        self.purge_move_speed = config.getfloat(
            "purge_move_speed", 1500.0, above=0.0)
        self.purge_wipe_accel = config.getfloat(
            "purge_wipe_accel", 15000.0, above=0.0)
        self.purge_wipe_speed = config.getfloat(
            "purge_wipe_speed", 12000.0, above=0.0)
        self.purge_wipe_repetitions = config.getint(
            "purge_wipe_repetitions", 3, minval=1, maxval=10)

        # EXTRUDE stage 5->6->7 handoff tuning - see _extrude_material_handoff()
        # below for the full sequence this drives. UPDATED 2026-08-17: an
        # earlier version of this file just raised the toolhead priming
        # distances (prime_e1/e2, 10mm/5mm -> 20mm/15mm) after live testing
        # showed filament reaching the toolhead sensor but the extruder gear
        # never actually grabbing it - that was a reasonable guess at the
        # time, but decompiling the real firmware's extrude_material()
        # function (previously a decompyle3 parse failure, only readable via
        # raw bytecode disassembly - see FINDINGS.md) revealed the ACTUAL
        # missing piece is architectural, not distance: the real firmware
        # runs a bounded RETRY LOOP here, verified against the box's own
        # measuring-wheel/odometer reading (not just a toolhead sensor
        # check) after each attempt, using a short but FAST push
        # (default 9mm at F12000 - 200mm/s, not our old slow F35/F10) -
        # repeated up to extrude_material_times times, with a toolhead+box
        # retreat-and-retry recovery step if a given attempt's stage 7
        # doesn't come back OK. prime_e1/prime_e2 (the fixed 10mm/5mm moves
        # immediately around stage 6 and inside the retry loop's stage 7
        # branch) turned out to be correct in the real firmware after all -
        # reverted to their original values here now that the real
        # bottleneck is understood to be architecture, not raw distance.
        self.prime_e1 = config.getfloat("prime_e1", 10.0)
        self.prime_e2 = config.getfloat("prime_e2", 5.0)
        self.extrude_material_len_for_extruder = config.getfloat(
            "extrude_material_len_for_extruder", 9.0, minval=0.0, maxval=60.0)
        self.extrude_material_times = config.getint("extrude_material_times", 6, minval=1)

        # Name of the real toolhead filament sensor (a plain Klipper
        # [filament_switch_sensor], NOT one of this extra's own virtual
        # CFS_A..CFS_D sensors). Loading uses it as the stage-5 completion
        # signal; tip-form unload uses it to know when it is safe to stop.
        self.toolhead_sensor_name = config.get("toolhead_sensor_name", "filament_sensor_2")

        self.ser = None
        self._serial_fd_handle = None
        self._rx_buffer = bytearray()
        self._pending_response = None
        self._pending_match = None
        self._response_quarantine_until = 0.0
        # The CFS link is half-duplex. Klipper's mutex parks competing
        # greenlets cooperatively, so polling and gcode commands cannot race
        # for the one pending response without blocking the reactor.
        self._bus_lock = self.reactor.mutex()
        self.addressed = False
        self.last_status = {}

        # Register one virtual filament_switch_sensor per slot so Fluidd/
        # Mainsail show CFS material presence in their normal filament
        # sensor panel, without any custom frontend work. Sensor names are
        # "filament_switch_sensor CFS_A".."CFS_D".
        name_prefix = config.get("sensor_name_prefix", "CFS_")
        self.slot_sensors = {}
        for slot_letter in SLOT_BYTES:
            sensor = CFSSlotSensor()
            self.printer.add_object(
                "filament_switch_sensor %s%s" % (name_prefix, slot_letter), sensor)
            self.slot_sensors[slot_letter] = sensor

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)

        gcode = self.gcode
        gcode.register_command("CFS_STATUS", self.cmd_CFS_STATUS,
                                desc="Report CFS box status and sensor state")
        gcode.register_command("CFS_RETRUDE", self.cmd_CFS_RETRUDE,
                                desc="CFS_RETRUDE SLOT=<A|B|C|D> - reel filament back onto the spool")
        gcode.register_command("CFS_EXTRUDE", self.cmd_CFS_EXTRUDE,
                                desc="CFS_EXTRUDE SLOT=<A|B|C|D> [POLLS=<n>] - feed filament from the spool")
        gcode.register_command("CFS_SET_PRE_LOADING", self.cmd_CFS_SET_PRE_LOADING,
                                desc="CFS_SET_PRE_LOADING ACTION=<CLOSE|OPEN|RUN|TIGHT> [MASK=<0-15>] "
                                     "- mostly for diagnostics; CLOSE is run automatically as a reset "
                                     "step at the start of CFS_RETRUDE/CFS_EXTRUDE. RUN/TIGHT are slow "
                                     "(~38s/slot) and untested by us - use supervised.")
        gcode.register_command("CFS_RECONNECT", self.cmd_CFS_RECONNECT,
                                desc="CFS_RECONNECT - retry discovery/addressing manually. The "
                                     "automatic attempt at klippy:connect can lose a race with "
                                     "the USB device settling - if CFS_STATUS says "
                                     "'not addressed' after startup even though the box is known "
                                     "good, run this instead of a full restart.")
        gcode.register_command("CFS_SYNC_FEED", self.cmd_CFS_SYNC_FEED,
                                desc="CFS_SYNC_FEED [DIST=<mm, default 100>] - DIAGNOSTIC, not "
                                     "part of normal use. Fires a raw box feed-motor move and a "
                                     "toolhead G1 E move of the same distance as close to "
                                     "simultaneously as possible, to test whether genuinely "
                                     "synchronized box+toolhead feeding pushes filament through "
                                     "where the staged EXTRUDE_PROCESS sequence hasn't. Requires "
                                     "a hot nozzle and homed axes are not needed.")
        gcode.register_command("CFS_BOX_FEED", self.cmd_CFS_BOX_FEED,
                                desc="CFS_BOX_FEED [DIST=<mm, default 50>] - DIAGNOSTIC, box-only "
                                     "isolation companion to CFS_SYNC_FEED. Fires just the box's "
                                     "raw feed-motor move (no toolhead move at all) to check "
                                     "whether the box pushes cleanly on its own.")
        gcode.register_command("CFS_SET_PRINT_MODE", self.cmd_CFS_SET_PRINT_MODE,
                                desc="CFS_SET_PRINT_MODE SLOT=<A|B|C|D> - DIAGNOSTIC. Sets the "
                                     "box's per-slot PRINT mode early/standalone, to test whether "
                                     "that's the real 'auto-feed on buffer demand' mode - see this "
                                     "command's own docstring.")
        # Hidden, box.cfg-compatible low-level commands. The leading
        # underscore follows Klipper's convention for implementation
        # commands that macros may call but users should not normally see
        # in HELP. These intentionally translate named parameters to this
        # project's live-validated F7/CRC8 protocol; the referenced
        # serial_485 wrapper uses different command ids and framing, so its
        # numeric packets must not be copied verbatim.
        direct_commands = {
            "_BOX_GET_BOX_STATE": self.cmd_BOX_GET_BOX_STATE,
            "_BOX_GET_VERSION_SN": self.cmd_BOX_GET_VERSION_SN,
            "_BOX_GET_RFID": self.cmd_BOX_GET_RFID,
            "_BOX_GET_REMAIN_LEN": self.cmd_BOX_GET_REMAIN_LEN,
            "_BOX_GET_BUFFER_STATE": self.cmd_BOX_GET_BUFFER_STATE,
            "_BOX_GET_FILAMENT_SENSOR_STATE": self.cmd_BOX_GET_FILAMENT_SENSOR_STATE,
            "_BOX_SET_BOX_MODE": self.cmd_BOX_SET_BOX_MODE,
            "_BOX_SET_PRE_LOADING": self.cmd_BOX_SET_PRE_LOADING,
            "_BOX_CTRL_CONNECTION_MOTOR_ACTION": self.cmd_BOX_CTRL_CONNECTION_MOTOR_ACTION,
            "_BOX_MEASURING_WHEEL": self.cmd_BOX_MEASURING_WHEEL,
            "_BOX_TIGHTEN_UP_ENABLE": self.cmd_BOX_TIGHTEN_UP_ENABLE,
            "_BOX_EXTRUDE_PROCESS": self.cmd_BOX_EXTRUDE_PROCESS,
            "_CFS_EXTRUDE_UNTIL_SENSOR": self.cmd_CFS_EXTRUDE_UNTIL_SENSOR,
            "_BOX_RETRUDE_PROCESS": self.cmd_BOX_RETRUDE_PROCESS,
            "_BOX_MOVE_DISTANCE": self.cmd_BOX_MOVE_DISTANCE,
            "_BOX_SEND_DATA": self.cmd_BOX_SEND_DATA,
        }
        for name, handler in direct_commands.items():
            # No desc: Klipper only adds described commands to HELP.
            gcode.register_command(name, handler)

    # -- low level transport -------------------------------------------

    def _open(self):
        if self.ser is None:
            ser = serial.Serial(
                self.serial_path, baudrate=self.baud, timeout=0, write_timeout=0)
            try:
                fd_handle = self.reactor.register_fd(
                    ser.fileno(), self._handle_serial_read)
            except Exception:
                ser.close()
                raise
            self.ser = ser
            self._serial_fd_handle = fd_handle
            self._rx_buffer = bytearray()

    def _close(self):
        """Wake any waiter, unregister the reactor fd, and close the port."""
        self.addressed = False
        pending, self._pending_response = self._pending_response, None
        self._pending_match = None
        if pending is not None and not pending.test():
            pending.complete(b"")

        if self._serial_fd_handle is not None:
            try:
                self.reactor.unregister_fd(self._serial_fd_handle)
            except Exception:
                logging.exception("creality_cfs: failed to unregister serial fd")
            self._serial_fd_handle = None

        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self._rx_buffer = bytearray()
        self._response_quarantine_until = 0.0

    def _pause(self, seconds):
        """Cooperative wait - use instead of a raw time.sleep() anywhere in
        this file. A plain time.sleep() hard-blocks Klipper's whole
        single-threaded reactor, including its own heater PID/watchdog
        timers; reactor.pause() yields to the reactor while waiting, so
        those keep running normally. Found live 2026-08-16 (see
        FINDINGS.md): a CFS_EXTRUDE run's accumulated time.sleep() calls
        stalled the reactor long enough that verify_heater's watchdog
        missed its update window and tripped a false "Heater extruder not
        heating at expected rate" shutdown, even though the hotend itself
        was fine. Serial response waits are cooperative too; see the file
        header and _send()."""
        self.reactor.pause(self.reactor.monotonic() + seconds)

    def _send(self, slave_addr, status, function_code, data=b"", timeout=2.0, debug=False):
        self._open()
        frame = build_frame(slave_addr, status, function_code, data)
        if debug:
            logging.info("creality_cfs: TX %s", frame.hex())

        # Broadcast requests are answered from a box's assigned unicast
        # address, so only match their function code. Normal requests must
        # match both address and function.
        match_addr = None if slave_addr in (0xFE, 0xFF) else slave_addr
        with self._bus_lock:
            quarantine_until = getattr(
                self, "_response_quarantine_until", 0.0)
            if quarantine_until > self.reactor.monotonic():
                # A reply that arrives after its transaction timed out cannot
                # be correlated with a later request using the same function
                # code. Keep the bus idle for one extra timeout window so the
                # reactor can read and discard that late frame while no
                # transaction is pending.
                self.reactor.pause(quarantine_until)
            self._response_quarantine_until = 0.0
            self.ser.reset_input_buffer()
            self._rx_buffer = bytearray()
            completion = self.reactor.completion()
            self._pending_response = completion
            self._pending_match = (match_addr, function_code)
            try:
                try:
                    # Pyserial's POSIX write wrapper can busy-loop on EAGAIN
                    # even with write_timeout=0. One raw write against its
                    # O_NONBLOCK fd either succeeds immediately or fails this
                    # transaction without monopolizing Klipper's reactor.
                    written = os.write(self.ser.fileno(), frame)
                except BlockingIOError:
                    logging.warning(
                        "creality_cfs: serial write would block; command not sent")
                    return b""
                except OSError:
                    logging.exception("creality_cfs: serial write failed")
                    self._close()
                    return b""
                if written != len(frame):
                    logging.error(
                        "creality_cfs: short non-blocking serial write (%s/%d bytes)",
                        written, len(frame))
                    self._close()
                    return b""
                response = completion.wait(
                    self.reactor.monotonic() + timeout, b"")
                if not response:
                    self._response_quarantine_until = max(
                        self._response_quarantine_until,
                        self.reactor.monotonic() + timeout)
                if debug:
                    logging.info(
                        "creality_cfs: RX %s",
                        response.hex() if response else "(nothing)")
                return response or b""
            finally:
                if self._pending_response is completion:
                    self._pending_response = None
                    self._pending_match = None

    def _handle_serial_read(self, eventtime):
        """Read available bytes without blocking and extract complete frames."""
        if self.ser is None:
            return
        try:
            chunk = self.ser.read(256)
        except Exception:
            logging.exception("creality_cfs: serial read failed")
            # A disconnected tty can remain level-triggered readable forever.
            # Unregister it immediately to avoid a callback/log storm, and
            # wake the active transaction so it need not wait for its timeout.
            self._close()
            return
        if not chunk:
            return
        self._rx_buffer.extend(chunk)
        self._parse_serial_frames()

    def _parse_serial_frames(self):
        """Parse [head, addr, length, ...] frames from the receive buffer."""
        while True:
            header_index = self._rx_buffer.find(0xF7)
            if header_index < 0:
                self._rx_buffer = bytearray()
                return
            if header_index:
                del self._rx_buffer[:header_index]
            if len(self._rx_buffer) < 3:
                return

            length = self._rx_buffer[2]
            if length < 3 or length > MAX_RESPONSE_DATA + 3:
                # Invalid length: discard this apparent header and resync.
                del self._rx_buffer[0]
                continue
            frame_length = 3 + length
            if len(self._rx_buffer) < frame_length:
                return

            response = bytes(self._rx_buffer[:frame_length])
            del self._rx_buffer[:frame_length]
            self._dispatch_serial_frame(response)

    def _dispatch_serial_frame(self, response):
        """Complete the active transaction when address and function match."""
        if len(response) < 6 or crc8(response[2:-1]) != response[-1]:
            logging.warning(
                "creality_cfs: discarded serial frame with invalid CRC: %s",
                response.hex())
            return
        pending = self._pending_response
        if pending is None or pending.test():
            return
        response_addr = response[1] if len(response) >= 2 else None
        response_function = response[4] if len(response) >= 5 else None
        match_addr, match_function = self._pending_match
        if ((match_addr is None or match_addr == response_addr)
                and match_function == response_function):
            self._pending_response = None
            self._pending_match = None
            pending.complete(response)

    # -- lifecycle --------------------------------------------------------

    def _handle_connect(self):
        try:
            self._discover_and_address()
        except Exception:
            logging.exception("creality_cfs: discovery/addressing failed at klippy:connect")
            return
        self.reactor.register_timer(self._poll_timer, self.reactor.NOW)

    def _handle_disconnect(self):
        self._close()

    def _discover_and_address(self, attempts=3, retry_pause=1.5):
        # ROOT CAUSE FOUND 2026-08-16 (see FINDINGS.md for the full
        # diagnostic trail): this box remembers its RS-485 address across
        # power cycles. Once it has been addressed once, it simply stops
        # replying to broadcast discovery (CMD_GET_SLAVE_INFO at the
        # broadcast address) - it's not listening there any more, not a
        # timing/reactor/environment problem. It answers a direct query at
        # its already-known address instantly and reliably. This is
        # exactly why cfs_cli.py's `status` command always worked: it
        # never does discovery either, it just talks straight to
        # box_addr=1. This extra's old code insisted on broadcast-first
        # with no fallback, so it failed every time on an
        # already-addressed box - confirmed identically whether run
        # inside Klipper, under klippy-env's own Python standalone, or
        # under the system Python: the execution context was never the
        # actual variable.
        #
        # Fix: try a cheap direct probe at box_addr first. Only fall back
        # to full broadcast discovery (for a genuinely fresh/unaddressed
        # box - e.g. first-ever run, or after replacing the box) if that
        # direct probe gets no reply.
        resp = self._send(self.box_addr, 0xFF, FN["CMD_ONLINE_CHECK"])
        if resp:
            self.addressed = True
            logging.info("creality_cfs: box already addressed at %#04x, "
                         "skipped broadcast discovery (direct probe replied)",
                         self.box_addr)
            return
        logging.info("creality_cfs: no reply from a direct probe at %#04x, "
                     "falling back to broadcast discovery (box may be "
                     "genuinely unaddressed)", self.box_addr)
        for attempt in range(1, attempts + 1):
            # debug=True here logs the raw TX/RX bytes at INFO level (always
            # visible in klippy.log, unlike logging.debug which Klipper's
            # default log setup may filter out) - added specifically to
            # diagnose the still-open addressing bug documented above: is
            # the write actually reaching the box, or is the read side
            # never getting a reply that's really there? Next physical
            # session, trigger CFS_RECONNECT (or a restart) and grep
            # klippy.log for "creality_cfs: TX"/"creality_cfs: RX" to see
            # exactly what went out and what (if anything) came back.
            resp = self._send(BROADCAST_ALL_BOXES, 0x00, FN["CMD_GET_SLAVE_INFO"],
                               bytes([BROADCAST_ALL_BOXES, BROADCAST_ALL_BOXES]),
                               debug=True)
            if len(resp) >= 20:
                uid = resp[7:19]
                self._send(BROADCAST_ALL_BOXES, 0x00, FN["CMD_SET_SLAVE_ADDR"],
                            bytes([self.box_addr]) + uid)
                self.addressed = True
                logging.info("creality_cfs: addressed box at %#04x, uid=%s "
                             "(attempt %d/%d)", self.box_addr, uid.hex(),
                             attempt, attempts)
                return
            logging.warning("creality_cfs: no CFS box responded to discovery "
                            "(attempt %d/%d)", attempt, attempts)
            if attempt < attempts:
                self._close()
                self._pause(retry_pause)

    def _poll_timer(self, eventtime):
        if self.addressed:
            try:
                resp = self._send(self.box_addr, 0xFF, FN["GET_BOX_STATE"])
                self.last_status["box_state"] = resp.hex() if resp else None
                sensor = self._send(self.box_addr, 0xFF, FN["GET_FILAMENT_SENSOR_STATE"], bytes([0x00]))
                if len(sensor) >= 6:
                    bitmask = sensor[5]
                    self.last_status["material_bitmask"] = bitmask
                    for slot_letter, slot_byte in SLOT_BYTES.items():
                        self.slot_sensors[slot_letter].filament_detected = bool(bitmask & slot_byte)
            except Exception:
                logging.exception("creality_cfs: poll failed")
        return eventtime + self.poll_interval

    # -- gcode commands -----------------------------------------------

    def _direct_addr(self, gcmd):
        return gcmd.get_int("ADDR", self.box_addr, minval=1, maxval=4)

    def _direct_enum(self, gcmd, name, values, default=None):
        value = gcmd.get(name, default)
        if value is None:
            raise gcmd.error("%s is required" % name)
        value = value.upper()
        if value not in values:
            raise gcmd.error("%s must be one of %s" % (
                name, ", ".join(values)))
        return value, values[value]

    def _direct_slot(self, gcmd, default=None):
        if gcmd.get("NUM", None) is not None:
            raise gcmd.error("NUM is no longer supported; use SLOT=A, B, C, or D")
        value = gcmd.get("SLOT", None)
        if value is None:
            if default is not None:
                return None, default
            raise gcmd.error("SLOT is required")
        value = value.upper()
        if value not in SLOT_BYTES:
            raise gcmd.error("SLOT must be one of A, B, C, D")
        return value, SLOT_BYTES[value]

    def _direct_byte(self, gcmd, name, default=None):
        raw = gcmd.get(name, None)
        if raw is None:
            if default is None:
                raise gcmd.error("%s is required" % name)
            return default
        try:
            value = int(raw, 0)
        except (TypeError, ValueError):
            raise gcmd.error("%s must be a byte (decimal or 0x-prefixed hex)" % name)
        if not 0 <= value <= 0xFF:
            raise gcmd.error("%s must be between 0 and 255" % name)
        return value

    def _direct_send(self, gcmd, label, function_code, data=b"", timeout=2.0,
                     decoder=None):
        addr = self._direct_addr(gcmd)
        resp = self._send(addr, 0xFF, function_code, data, timeout=timeout)
        status = resp[3] if len(resp) >= 4 else None
        detail = decoder(resp) if decoder is not None else None
        message = "%s ADDR=%d status=%s" % (
            label, addr, hex(status) if status is not None else "no reply")
        if detail:
            message += " %s" % detail
        message += " raw=%s" % (resp.hex() if resp else "(none)")
        gcmd.respond_info(message)
        return resp

    @staticmethod
    def _direct_hex_data(gcmd):
        raw = gcmd.get("DATA", "").strip()
        if not raw:
            return b""
        try:
            if " " in raw or "," in raw:
                tokens = raw.replace(",", " ").split()
                data = bytes(int(token, 16) for token in tokens)
            else:
                compact = raw[2:] if raw.lower().startswith("0x") else raw
                if len(compact) % 2:
                    raise ValueError
                data = bytes.fromhex(compact)
        except (TypeError, ValueError):
            raise gcmd.error(
                "DATA must be hex, for example DATA=0f01 or DATA=0f,01")
        return data

    def cmd_BOX_GET_BOX_STATE(self, gcmd):
        self._direct_send(gcmd, "_BOX_GET_BOX_STATE", FN["GET_BOX_STATE"])

    def cmd_BOX_GET_VERSION_SN(self, gcmd):
        def decode(resp):
            if len(resp) < 7:
                return None
            try:
                value = resp[5:-1].decode("ascii")
            except UnicodeDecodeError:
                return None
            return "version_sn=%s" % value

        self._direct_send(gcmd, "_BOX_GET_VERSION_SN", FN["GET_VERSION_SN"],
                          decoder=decode)

    def cmd_BOX_GET_RFID(self, gcmd):
        _, slot = self._direct_slot(gcmd, default=0x0F)

        def decode(resp):
            if len(resp) < 7:
                return None
            try:
                return "rfid=%s" % resp[5:-1].decode("ascii")
            except UnicodeDecodeError:
                return None

        self._direct_send(gcmd, "_BOX_GET_RFID", FN["GET_RFID"], bytes([slot]),
                          decoder=decode)

    def cmd_BOX_GET_REMAIN_LEN(self, gcmd):
        _, slot = self._direct_slot(gcmd, default=0x0F)

        def decode(resp):
            return "remain=%s" % resp[5:-1].hex() if len(resp) >= 7 else None

        self._direct_send(gcmd, "_BOX_GET_REMAIN_LEN", FN["GET_REMAIN_LEN"],
                          bytes([slot]), decoder=decode)

    def cmd_BOX_GET_BUFFER_STATE(self, gcmd):
        def decode(resp):
            if len(resp) < 6:
                return None
            value = resp[5]
            name = {0: "MIDDLE", 1: "FULL", 2: "EMPTY"}.get(value, "UNKNOWN")
            return "buffer=%s(%d)" % (name, value)

        self._direct_send(gcmd, "_BOX_GET_BUFFER_STATE", FN["GET_BUFFER_STATE"],
                          decoder=decode)

    def cmd_BOX_GET_FILAMENT_SENSOR_STATE(self, gcmd):
        position, bank = self._direct_enum(
            gcmd, "POSITION", {"MATERIAL": 0x00, "CONNECTIONS": 0x01},
            default="MATERIAL")

        def decode(resp):
            if len(resp) < 6:
                return None
            mask = resp[5]
            slots = [name for name, bit in SLOT_BYTES.items() if mask & bit]
            return "%s=%#04x slots=%s" % (
                position.lower(), mask, ",".join(slots) or "none")

        self._direct_send(gcmd, "_BOX_GET_FILAMENT_SENSOR_STATE",
                          FN["GET_FILAMENT_SENSOR_STATE"], bytes([bank]),
                          decoder=decode)

    def cmd_BOX_SET_BOX_MODE(self, gcmd):
        _, slot = self._direct_slot(gcmd, default=0x00)
        _, mode = self._direct_enum(
            gcmd, "MODE", {"PRINT": 0x00, "IDLE": 0x01})
        self._direct_send(gcmd, "_BOX_SET_BOX_MODE", FN["SET_BOX_MODE"],
                          bytes([slot, mode]))

    def cmd_BOX_SET_PRE_LOADING(self, gcmd):
        _, slot = self._direct_slot(gcmd, default=0x0F)
        action_name, action = self._direct_enum(
            gcmd, "ACTION", {"CLOSE": 0x00, "OPEN": 0x01,
                              "RUN": 0x02, "TIGHT": 0x03},
            default="CLOSE")
        timeout = 45.0 if action_name in ("RUN", "TIGHT") else 2.0
        self._direct_send(gcmd, "_BOX_SET_PRE_LOADING", FN["SET_PRE_LOADING"],
                          bytes([slot, action]), timeout=timeout)

    def cmd_BOX_CTRL_CONNECTION_MOTOR_ACTION(self, gcmd):
        _, action = self._direct_enum(
            gcmd, "ACTION", {"STOP": 0x00, "EXTRUDE": 0x01, "RETRUDE": 0x02})
        self._direct_send(gcmd, "_BOX_CTRL_CONNECTION_MOTOR_ACTION",
                          FN["CTRL_CONNECTION_MOTOR_ACTION"], bytes([action]))

    def cmd_BOX_MEASURING_WHEEL(self, gcmd):
        action_name, action = self._direct_enum(
            gcmd, "ACTION", {"CLEAN": 0x00, "GET": 0x01}, default="GET")

        def decode(resp):
            if action_name != "GET" or len(resp) < 10:
                return None
            value = decode_measuring_wheel(resp[5:-1])
            return "distance=%.3fmm" % value if value is not None else None

        self._direct_send(gcmd, "_BOX_MEASURING_WHEEL",
                          FN["GET_MEASURING_WHEEL"], bytes([action]), decoder=decode)

    def cmd_BOX_TIGHTEN_UP_ENABLE(self, gcmd):
        # This project's live protocol uses 1=enable, 0=disable. The
        # referenced serial_485 wrapper documents the opposite polarity.
        # Keep the human-readable API, translate to the validated bytes.
        _, enable = self._direct_enum(
            gcmd, "ENABLE", {"ENABLE": 0x01, "DISABLE": 0x00})
        self._direct_send(gcmd, "_BOX_TIGHTEN_UP_ENABLE",
                          FN["TIGHTEN_UP_ENABLE"], bytes([enable]))

    def cmd_BOX_EXTRUDE_PROCESS(self, gcmd):
        _, slot = self._direct_slot(gcmd)
        stage = gcmd.get_int("STAGE", minval=0, maxval=7)
        if stage not in (0, 3, 4, 5, 6, 7):
            raise gcmd.error("STAGE must be one of 0, 3, 4, 5, 6, 7")
        amount = self._direct_byte(gcmd, "AMOUNT", 0x03 if stage == 7 else 0x00)
        self._direct_send(gcmd, "_BOX_EXTRUDE_PROCESS", FN["EXTRUDE_PROCESS"],
                          bytes([slot, stage, amount]))

    def cmd_CFS_EXTRUDE_UNTIL_SENSOR(self, gcmd):
        _, slot = self._direct_slot(gcmd)
        sensor_name = gcmd.get("SENSOR", self.toolhead_sensor_name)
        polls = gcmd.get_int("POLLS", 50, minval=1, maxval=200)
        delay_ms = gcmd.get_int("DELAY_MS", 400, minval=1, maxval=5000)
        recovery_x = gcmd.get_float("RECOVERY_X", None)
        recovery_y = gcmd.get_float("RECOVERY_Y", None)
        recovery_speed = gcmd.get_float(
            "RECOVERY_SPEED", None, above=0.0)
        restore_outer = bool(gcmd.get_int(
            "RESTORE_OUTER", 0, minval=0, maxval=1))
        try:
            self._extrude_until_toolhead_sensor(
                gcmd, slot, polls, delay_ms / 1000.0,
                sensor_name=sensor_name)
        except Exception:
            cleanup_error = self._cleanup_box_after_load()
            if cleanup_error is not None:
                logging.error(
                    "creality_cfs: cleanup failed after stage-5 sensor "
                    "timeout: %s", cleanup_error)
            self._recover_direct_load(
                recovery_x, recovery_y, recovery_speed, restore_outer)
            raise

    def cmd_BOX_RETRUDE_PROCESS(self, gcmd):
        _, slot = self._direct_slot(gcmd, default=0x00)
        _, trigger = self._direct_enum(
            gcmd, "TRIGGER", {"BUFFER": 0x00, "MATERIAL": 0x01},
            default="BUFFER")
        self._direct_send(gcmd, "_BOX_RETRUDE_PROCESS", FN["RETRUDE_PROCESS"],
                          bytes([slot, trigger]))

    def cmd_BOX_MOVE_DISTANCE(self, gcmd):
        _, direction = self._direct_enum(
            gcmd, "DIRECTION", {"FORWARD": 0x00, "EXTRUDE": 0x00,
                                 "RETRUDE": 0x01, "REVERSE": 0x01},
            default="FORWARD")
        dist = gcmd.get_int("DIST", minval=1, maxval=255)
        timeout = gcmd.get_float("TIMEOUT", 2.0, minval=0.05, maxval=120.0)
        self._direct_send(gcmd, "_BOX_MOVE_DISTANCE", FN["MOVE_DISTANCE"],
                          bytes([direction, dist]), timeout=timeout)

    def cmd_BOX_SEND_DATA(self, gcmd):
        command = self._direct_byte(gcmd, "CMD")
        status = self._direct_byte(gcmd, "STATE", 0xFF)
        timeout = gcmd.get_float("TIMEOUT", 2.0, minval=0.05, maxval=120.0)
        data = self._direct_hex_data(gcmd)
        addr = self._direct_addr(gcmd)
        resp = self._send(addr, status, command, data, timeout=timeout)
        response_status = resp[3] if len(resp) >= 4 else None
        gcmd.respond_info("_BOX_SEND_DATA ADDR=%d CMD=%#04x status=%s raw=%s" % (
            addr, command,
            hex(response_status) if response_status is not None else "no reply",
            resp.hex() if resp else "(none)"))

    def _reset_pre_loading(self):
        """CLOSE (disable) pre-loading on all 4 slots - a cheap, fast,
        non-motor call the real official sequence sends as a "reset to
        known state" step before every toolchange (see FINDINGS.md in
        the private research log). Best-effort - failures here shouldn't
        block the actual retrude/extrude that follows."""
        try:
            self._send(self.box_addr, 0xFF, FN["SET_PRE_LOADING"], bytes([0x0F, 0x00]))
        except Exception:
            logging.exception("creality_cfs: pre-loading reset failed (non-fatal)")

    def _get_measuring_wheel(self):
        """Read the box's measuring-wheel/odometer distance (fn 0x0E,
        data=[0x01] = the real firmware's "GET" action byte - confirmed
        2026-08-17 from decompiled reference, see FINDINGS.md). Returns
        None if the box didn't reply with a valid 4-byte reading - callers
        must handle that (treat as "can't verify", not "definitely 0")."""
        resp = self._send(self.box_addr, 0xFF, FN["GET_MEASURING_WHEEL"], bytes([0x01]))
        if len(resp) >= 10 and resp[3] == 0x00:
            value = decode_measuring_wheel(resp[5:9])
            if value is not None and math.isfinite(value):
                return value
        return None

    def _get_buffer_state(self):
        """Returns the raw buffer_state byte (0=middle, 1=full, 2=empty -
        see docs/PROTOCOL.md), or None if no valid reply."""
        resp = self._send(self.box_addr, 0xFF, FN["GET_BUFFER_STATE"])
        if len(resp) >= 7 and resp[3] == 0x00 and resp[5] in (0, 1, 2):
            return resp[5]
        return None

    def _extrude_material_handoff(self, gcmd, slot):
        """The real firmware's stage 6->7 handoff, ported 2026-08-17 from
        decompiled reference (extrude_material(), previously unreadable
        via decompyle3 - only recovered via raw bytecode disassembly, see
        FINDINGS.md). This is the moment the box has pushed filament up to
        the toolhead sensor and the toolhead extruder needs to actually
        grab it and pull the rest of the way, while the box keeps feeding
        - our earlier single-shot version (one slow toolhead move, trust
        the sensor) was missing this whole retry+verify structure.

        Sequence: a fixed priming move + stage 6, then up to
        extrude_material_times attempts of a short fast push
        (extrude_material_len_for_extruder mm at F12000), each verified
        against the ACTUAL distance the box's measuring wheel reports
        moving (not just "does the sensor still see filament") - stopping
        as soon as the buffer isn't full or the wheel confirms enough
        distance. Each attempt that doesn't look successful also tries
        stage 7 and, if that comes back bad, does a toolhead+box retreat
        before the next attempt.

        Returns True if the handoff has positive sensor plus buffer/wheel
        evidence. False is a hard failure for the wrapped load; it must not
        enter PRINT mode without that evidence."""
        if self._toolhead_filament_detected() is not True:
            gcmd.respond_info(
                "CFS_EXTRUDE: refusing handoff because the toolhead sensor "
                "has not confirmed filament")
            return False

        self.gcode.run_script_from_command("M83")
        self.gcode.run_script_from_command("G0 E%.2f F35" % self.prime_e1)
        self._pause(0.3)
        resp6 = self._send(self.box_addr, 0xFF, FN["EXTRUDE_PROCESS"], bytes([slot, 0x06, 0x00]))
        status6 = resp6[3] if len(resp6) >= 4 else None
        if status6 != 0x00:
            gcmd.respond_info("CFS_EXTRUDE: stage 6 status=%s (continuing - real "
                               "firmware doesn't hard-stop here either)" %
                               (hex(status6) if status6 is not None else "no reply"))

        initial_distance = self._get_measuring_wheel()
        retry_count = 0
        for attempt in range(self.extrude_material_times):
            self.gcode.run_script_from_command("M83")
            self.gcode.run_script_from_command(
                "G0 E%.2f F12000" % self.extrude_material_len_for_extruder)
            self.gcode.run_script_from_command("M400")

            buffer_state = self._get_buffer_state()
            new_distance = self._get_measuring_wheel()
            diff_length = None
            if initial_distance is not None and new_distance is not None:
                diff_length = abs(new_distance - initial_distance)
            gcmd.respond_info(
                "CFS_EXTRUDE: handoff attempt %d/%d - buffer_state=%s, "
                "measuring-wheel diff=%s" % (
                    attempt + 1, self.extrude_material_times, buffer_state,
                    ("%.2fmm" % diff_length) if diff_length is not None else "unknown"))

            buffer_released = buffer_state in (0, 2)
            wheel_moved = (diff_length is not None and
                           math.isfinite(diff_length) and
                           diff_length >= self.extrude_material_len_for_extruder)
            sensor_confirmed = self._toolhead_filament_detected() is True
            if sensor_confirmed and (buffer_released or wheel_moved):
                return True

            self._send(self.box_addr, 0xFF, FN["SET_BOX_MODE"], bytes([0x00, 0x01]))
            self.gcode.run_script_from_command("M83")
            self.gcode.run_script_from_command("G0 E%.2f F10" % self.prime_e2)
            # stage 7's 3rd byte: was 0x02 (our own decompiled-source guess,
            # never independently confirmed) - corrected to 0x03 2026-08-17
            # after finding github.com/Jacob10383/k2-plus-custom-firmware's
            # own independent open-source CFS protocol implementation
            # (different Creality printer, same underlying CFS protocol
            # family), whose load_stage() uses `argument = 3 if stage == 7
            # else 0`. Not yet independently live-verified against our own
            # hardware either way, but a second, publicly-visible source
            # agreeing is a real signal worth acting on.
            resp7 = self._send(self.box_addr, 0xFF, FN["EXTRUDE_PROCESS"], bytes([slot, 0x07, 0x03]))
            status7 = resp7[3] if len(resp7) >= 4 else None
            if status7 != 0x00:
                retry_count += 1
                if retry_count > 3:
                    gcmd.respond_info("CFS_EXTRUDE: handoff giving up - "
                                       "%d stage-7 failures" % retry_count)
                    return False
                # Recovery: check in with a generic retrude, then retreat
                # both the toolhead and the box before the next attempt -
                # matches the real firmware's own fallback here.
                recover = self._send(self.box_addr, 0xFF, FN["RETRUDE_PROCESS"],
                                      bytes([0x00, 0x00]))
                recover_status = recover[3] if len(recover) >= 4 else None
                if recover_status in (0x00, 0x12):
                    self.gcode.run_script_from_command("M83")
                    self.gcode.run_script_from_command("G0 E-10 F180")
                    self.gcode.run_script_from_command("M400")
                    self._send(self.box_addr, 0xFF, FN["MOVE_DISTANCE"], bytes([0x01, 50]))

        gcmd.respond_info("CFS_EXTRUDE: handoff did not confirm success after "
                           "%d attempts - check physically" % self.extrude_material_times)
        return False

    def cmd_CFS_SET_PRE_LOADING(self, gcmd):
        action = gcmd.get("ACTION", "CLOSE").upper()
        action_map = {"CLOSE": 0x00, "OPEN": 0x01, "RUN": 0x02, "TIGHT": 0x03}
        if action not in action_map:
            raise gcmd.error("ACTION must be one of CLOSE, OPEN, RUN, TIGHT")
        mask = gcmd.get_int("MASK", 0x0F, minval=0, maxval=15)
        timeout = 45.0 if action in ("RUN", "TIGHT") else 2.0
        resp = self._send(self.box_addr, 0xFF, FN["SET_PRE_LOADING"],
                           bytes([mask, action_map[action]]), timeout=timeout)
        gcmd.respond_info("CFS_SET_PRE_LOADING ACTION=%s MASK=%#04x: %s" % (
            action, mask, resp.hex() if resp else "(no reply)"))

    def cmd_CFS_STATUS(self, gcmd):
        if not self.addressed:
            gcmd.respond_info("CFS box not addressed (no response at klippy:connect) - "
                               "try CFS_RECONNECT")
            return
        bitmask = self.last_status.get("material_bitmask")
        if bitmask is not None:
            loaded = [name for name, bit in SLOT_BYTES.items() if bitmask & bit]
            gcmd.respond_info("CFS: material loaded in slots: %s" % (", ".join(loaded) or "none"))
        else:
            gcmd.respond_info("CFS: no status polled yet")

    def cmd_CFS_RECONNECT(self, gcmd):
        # Close and reopen the serial connection first, not just retry on
        # the existing one - a serial object that's been open since a
        # failed first attempt at klippy:connect can apparently get stuck
        # in a way a plain retry on the same connection doesn't recover
        # from (matches this project's live experience: the standalone
        # cfs_cli.py tool, which opens a fresh connection every time,
        # kept working throughout even when this extra's persistent one
        # didn't - see FINDINGS.md).
        self._close()
        try:
            self._discover_and_address()
        except Exception as e:
            raise gcmd.error("CFS_RECONNECT failed: %s" % (e,))
        if self.addressed:
            gcmd.respond_info("CFS_RECONNECT: addressed OK")
        else:
            gcmd.respond_info("CFS_RECONNECT: still not addressed - box may not be "
                               "responding right now, check it physically")

    def cmd_CFS_SYNC_FEED(self, gcmd):
        """DIAGNOSTIC, added 2026-08-17 - not part of the normal
        CFS_EXTRUDE flow. Live testing found box-side pushes (verified via
        the measuring wheel) and toolhead-side pulls each individually
        "work", but filament still doesn't reliably make it out the
        nozzle - user's hypothesis: the box and toolhead extruder aren't
        actually moving AT THE SAME TIME, since our stage-based sequence
        sends a box command, waits for its reply, THEN sends a toolhead
        move - never truly concurrent. This command fires the box's raw
        MOVE_DISTANCE (fn 0x31 - a direct feed-motor move, not the full
        EXTRUDE_PROCESS state machine) with a short response timeout,
        immediately followed by a toolhead G1 E move
        of the same distance, to get them physically overlapping in time
        as closely as this request/reply sequence allows. The response wait
        yields to Klipper's reactor, but the toolhead move still starts only
        after that wait returns. CFS_SYNC_FEED DIST=<mm, 1-255, default 100>."""
        dist = gcmd.get_int("DIST", 100, minval=1, maxval=255)
        self.gcode.run_script_from_command("M83")
        self._send(self.box_addr, 0xFF, FN["MOVE_DISTANCE"],
                    bytes([0x00, dist & 0xFF]), timeout=0.3)
        self.gcode.run_script_from_command("G1 E%d F300" % dist)
        self.gcode.run_script_from_command("M400")
        gcmd.respond_info("CFS_SYNC_FEED: sent box MOVE_DISTANCE FORWARD %dmm "
                           "+ toolhead G1 E%d together - check physically" % (dist, dist))

    def cmd_CFS_BOX_FEED(self, gcmd):
        """DIAGNOSTIC, added 2026-08-17 - box-only isolation test, no
        toolhead move at all. Companion to CFS_SYNC_FEED: fires the box's
        raw MOVE_DISTANCE (fn 0x31, FORWARD) by itself, to check whether
        the box's own feed motor/rollers push cleanly in isolation (watch
        physically - does filament actually advance at the box/buffer, or
        does it slip there too?). CFS_BOX_FEED DIST=<mm, 1-255, default 50>
        [SLOT=<A|B|C|D>] - optional, live-testing whether this command
        needs a slot byte at all (first live attempt with the 2-byte
        [direction, distance] payload from decompiled source got
        PARAMS_ERR - same class of live-vs-decompiled mismatch this repo
        already hit once for EXTRUDE_PROCESS, which needed an extra byte
        beyond what the reference code showed. Not yet confirmed which
        byte order/count is right - this param exists purely to test live)."""
        dist = gcmd.get_int("DIST", 50, minval=1, maxval=255)
        slot_letter = gcmd.get("SLOT", None)
        if slot_letter:
            slot_letter = slot_letter.upper()
            if slot_letter not in SLOT_BYTES:
                raise gcmd.error("SLOT must be one of A, B, C, D")
            data = bytes([SLOT_BYTES[slot_letter], 0x00, dist & 0xFF])
        else:
            data = bytes([0x00, dist & 0xFF])
        resp = self._send(self.box_addr, 0xFF, FN["MOVE_DISTANCE"], data, timeout=2.0)
        status = resp[3] if len(resp) >= 4 else None
        gcmd.respond_info("CFS_BOX_FEED: sent data=%s, status=%s - "
                           "check physically at the box/buffer" % (
                               data.hex(), hex(status) if status is not None else "no reply"))

    def cmd_CFS_SET_PRINT_MODE(self, gcmd):
        """DIAGNOSTIC, added 2026-08-17. User's hypothesis: the box's
        per-slot PRINT mode (SET_BOX_MODE payload [slot_bitmask, 0x00])
        may be the real "listen to the buffer sensor and auto-feed on
        demand" mode - the same continuous, reactive feeding the box does
        during an actual print (buffer slider tension -> box feeds more).
        Our cmd_CFS_EXTRUDE only sets this at the very END, after the
        handoff already succeeded or failed - meaning during the actual
        handoff struggle, the box may still be in a more restrictive
        loading state, not the auto-feed mode. This command sets PRINT
        mode for SLOT early/on demand, standalone, so you can then try a
        plain manual extrude (M83 + G1 E...) afterward and see if the box
        auto-feeds along with it more naturally. CFS_SET_PRINT_MODE
        SLOT=<A|B|C|D>."""
        slot_letter = gcmd.get("SLOT").upper()
        if slot_letter not in SLOT_BYTES:
            raise gcmd.error("SLOT must be one of A, B, C, D")
        slot = SLOT_BYTES[slot_letter]
        resp = self._send(self.box_addr, 0xFF, FN["SET_BOX_MODE"], bytes([slot, 0x00]))
        status = resp[3] if len(resp) >= 4 else None
        gcmd.respond_info("CFS_SET_PRINT_MODE: slot=%s status=%s - now try a plain "
                           "manual extrude (M83, G1 E...) and watch whether the box "
                           "auto-feeds along with it" % (
                               slot_letter, hex(status) if status is not None else "no reply"))

    def _toolhead_filament_detected(self, sensor_name=None):
        sensor_name = sensor_name or self.toolhead_sensor_name
        sensor = self.printer.lookup_object(
            "filament_switch_sensor %s" % sensor_name, None)
        if sensor is None:
            return None
        return sensor.get_status(self.reactor.monotonic())["filament_detected"]

    def _extrude_until_toolhead_sensor(
            self, gcmd, slot, polls, delay, sensor_name=None):
        """Poll stage 5 until the physical toolhead sensor confirms arrival."""
        sensor_name = sensor_name or self.toolhead_sensor_name
        sensor = self.printer.lookup_object(
            "filament_switch_sensor %s" % sensor_name, None)
        if sensor is None:
            raise gcmd.error(
                "CFS_EXTRUDE: toolhead sensor '%s' is not configured" %
                sensor_name)

        for attempt in range(1, polls + 1):
            resp = self._send(
                self.box_addr, 0xFF, FN["EXTRUDE_PROCESS"],
                bytes([slot, 0x05, 0x00]))
            status = resp[3] if len(resp) >= 4 else None
            if status not in (None, 0x00):
                gcmd.respond_info(
                    "CFS_EXTRUDE: stage 5 poll %d status=%s; waiting for "
                    "toolhead sensor" % (attempt, hex(status)))
            self._pause(delay)
            if sensor.get_status(
                    self.reactor.monotonic())["filament_detected"]:
                gcmd.respond_info(
                    "CFS_EXTRUDE: toolhead sensor confirmed after %d polls" %
                    attempt)
                return attempt

        raise gcmd.error(
            "CFS_EXTRUDE: toolhead sensor '%s' did not trigger after %d polls" %
            (sensor_name, polls))

    def _recover_direct_load(
            self, recovery_x, recovery_y, recovery_speed, restore_outer):
        """Best-effort bucket retreat and state restore after a direct abort."""
        if None in (recovery_x, recovery_y, recovery_speed):
            return
        try:
            self.gcode.run_script_from_command("G90")
            self.gcode.run_script_from_command(
                "G1 X%.2f Y%.2f F%s" % (
                    recovery_x, recovery_y,
                    format_gcode_number(recovery_speed)))
            self.gcode.run_script_from_command("M400")
        except Exception:
            logging.exception(
                "creality_cfs: direct-load bucket retreat failed")
        try:
            self.gcode.run_script_from_command(
                "RESTORE_GCODE_STATE NAME=CFS_DIRECT_LOAD MOVE=0")
        except Exception:
            logging.exception(
                "creality_cfs: direct-load G-code state restoration failed")
        if restore_outer:
            try:
                self.gcode.run_script_from_command(
                    "RESTORE_GCODE_STATE NAME=CFS_DIRECT_TOOLCHANGE MOVE=0")
            except Exception:
                logging.exception(
                    "creality_cfs: direct-toolchange G-code state "
                    "restoration failed")

    def _retrude_with_tip_form(self, gcmd):
        # UNTESTED (as of this writing) reimplementation of the real
        # official firmware's unload sequence - see TIP_FORM_STEPS above
        # and FINDINGS.md in the private research log for where this came
        # from and why: a single box-side RETRUDE_PROCESS call (what this
        # repo did before) can leave filament jammed in the toolhead
        # extruder's own drive gear, needing a manual lever release -
        # confirmed live, see docs/TOOLCHANGE_TEST_PLAN.md.
        #
        # First "wiggles" the extruder a small net distance to re-melt and
        # re-shape the filament tip into a smooth taper - a blobby/snagged
        # tip is what catches in the gear on the way out - then does the
        # real retraction in -15mm chunks, checking in with the box (a
        # generic, no-specific-slot RETRUDE_PROCESS call) and the toolhead
        # sensor between chunks so it can stop as soon as filament is
        # confirmed clear.
        self.gcode.run_script_from_command("M83")
        for dist, speed in TIP_FORM_STEPS:
            if dist <= -10:
                resp = self._send(self.box_addr, 0xFF, FN["RETRUDE_PROCESS"], bytes([0x00, 0x00]))
                status = resp[3] if len(resp) >= 4 else None
                if status != 0x00:
                    # Either no reply, or the box thinks its part might
                    # already be done (this generic, no-specific-slot
                    # check-in can be a stale/no-op query if the earlier
                    # slot-specific RETRUDE_PROCESS calls above already
                    # finished the actual unload - expected and fine).
                    # The real signal is the toolhead sensor, not this
                    # call's status alone.
                    self._send(self.box_addr, 0xFF, FN["SET_BOX_MODE"], bytes([0x00, 0x01]))
                    detected = self._toolhead_filament_detected()
                    if detected is False:
                        gcmd.respond_info("CFS_RETRUDE: toolhead sensor clear, "
                                           "unload complete (stopped early)")
                        return True
                    if status is None:
                        gcmd.respond_info("CFS_RETRUDE: no reply from box AND toolhead "
                                           "sensor still sees filament, stopping - "
                                           "check physically")
                        return False
                    # Sensor still sees filament but box did reply - keep
                    # going with the remaining steps.
            self.gcode.run_script_from_command("G0 E%.2f F%.0f" % (dist, speed))
            self.gcode.run_script_from_command("M400")
        detected = self._toolhead_filament_detected()
        return detected is False

    def cmd_CFS_RETRUDE(self, gcmd):
        slot_letter = gcmd.get("SLOT", "A").upper()
        if slot_letter not in SLOT_BYTES:
            raise gcmd.error("SLOT must be one of A, B, C, D")
        slot = SLOT_BYTES[slot_letter]

        self._reset_pre_loading()
        self._send(self.box_addr, 0xFF, FN["SET_BOX_MODE"], bytes([0x00, 0x01]))
        self._pause(0.3)
        self._send(self.box_addr, 0xFF, FN["RETRUDE_PROCESS"], bytes([slot, 0x00]))
        self._pause(0.5)
        self._send(self.box_addr, 0xFF, FN["RETRUDE_PROCESS"], bytes([slot, 0x01]))

        ok = self._retrude_with_tip_form(gcmd)
        gcmd.respond_info("CFS_RETRUDE slot=%s complete (tip-form unload %s)" % (
            slot_letter, "confirmed clear" if ok else "did NOT confirm clear - check physically"))

    def _approach_purge_bucket(self, current_z):
        """Lift safely, then travel to the bucket's outside entry point."""
        travel_z = (self.purge_min_z if current_z < self.purge_min_z
                    else current_z + self.purge_z_hop)

        self.gcode.run_script_from_command("G90")
        self.gcode.run_script_from_command(
            "G1 Z%.2f F%.0f" % (travel_z, self.purge_move_speed))
        self.gcode.run_script_from_command("M400")
        self.gcode.run_script_from_command(
            "G1 X%.2f Y%.2f F%.0f" % (
                self.purge_entry_x, self.purge_entry_y,
                self.purge_move_speed))
        self.gcode.run_script_from_command("M400")

    def _move_into_purge_bucket(self, speed):
        """Queue the configured cardinal move from the entry point inward."""
        purge_axes = []
        if self.purge_x is not None:
            purge_axes.append("X%.2f" % self.purge_x)
        if self.purge_y is not None:
            purge_axes.append("Y%.2f" % self.purge_y)
        self.gcode.run_script_from_command(
            "G1 %s F%.0f" % (" ".join(purge_axes), speed))

    def _exit_purge_bucket(self, original_accel):
        """Break nozzle strands, finish outside the bucket, and restore state."""
        entry_move = "G1 X%.2f Y%.2f F%.0f" % (
            self.purge_entry_x, self.purge_entry_y, self.purge_wipe_speed)
        purge_axes = []
        if self.purge_x is not None:
            purge_axes.append("X%.2f" % self.purge_x)
        if self.purge_y is not None:
            purge_axes.append("Y%.2f" % self.purge_y)
        purge_move = "G1 %s F%.0f" % (
            " ".join(purge_axes), self.purge_wipe_speed)

        first_error = None
        acceleration_changed = False
        try:
            self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%s" %
                format_gcode_number(self.purge_wipe_accel))
            acceleration_changed = True
            for _ in range(self.purge_wipe_repetitions):
                self.gcode.run_script_from_command(entry_move)
                self.gcode.run_script_from_command(purge_move)
        except Exception as exc:
            first_error = exc

        # Always attempt the final evacuation independently of wipe failures.
        # This is especially important when an entry move fails while the
        # nozzle is still physically inside the bucket.
        try:
            self.gcode.run_script_from_command(entry_move)
        except Exception as exc:
            if first_error is None:
                first_error = exc
            else:
                logging.exception(
                    "creality_cfs: final purge-bucket evacuation move failed")
        try:
            self.gcode.run_script_from_command("M400")
        except Exception as exc:
            if first_error is None:
                first_error = exc
            else:
                logging.exception(
                    "creality_cfs: purge-bucket evacuation wait failed")

        if acceleration_changed:
            try:
                self.gcode.run_script_from_command(
                    "SET_VELOCITY_LIMIT ACCEL=%s" %
                    format_gcode_number(original_accel))
            except Exception as exc:
                if first_error is None:
                    first_error = exc
                else:
                    logging.exception(
                        "creality_cfs: acceleration restoration failed")
        try:
            self.gcode.run_script_from_command(
                "RESTORE_GCODE_STATE NAME=CFS_EXTRUDE")
        except Exception as exc:
            if first_error is None:
                first_error = exc
            else:
                logging.exception(
                    "creality_cfs: G-code state restoration failed")

        if first_error is not None:
            raise first_error

    def _cleanup_box_after_load(self):
        """Attempt every box cleanup command and return the first failure."""
        first_error = None
        cleanup_commands = (
            (FN["TIGHTEN_UP_ENABLE"], bytes([0x00])),
            (FN["CTRL_CONNECTION_MOTOR_ACTION"], bytes([0x00])),
            # A completed run can leave a latched error status even after a
            # successful load. Re-entering IDLE clears it on live hardware.
            (FN["SET_BOX_MODE"], bytes([0x00, 0x01])),
        )
        for function_code, data in cleanup_commands:
            try:
                self._send(self.box_addr, 0xFF, function_code, data)
            except Exception as exc:
                if first_error is None:
                    first_error = exc
                logging.exception(
                    "creality_cfs: box cleanup command %#04x failed",
                    function_code)
        return first_error

    def _load_filament_in_bucket(self, gcmd, slot, polls):
        """Run the box loading stages while guaranteeing box-side cleanup."""
        load_error = None
        try:
            self._send(
                self.box_addr, 0xFF, FN["CTRL_CONNECTION_MOTOR_ACTION"],
                bytes([0x01]))
            self._pause(0.5)
            self._send(
                self.box_addr, 0xFF, FN["TIGHTEN_UP_ENABLE"], bytes([0x01]))
            self._pause(0.3)

            # EXTRUDE_PROCESS payload is [slot, stage, amount] - 3 bytes.
            # Live hardware rejected the 2-byte form inferred from decompiled
            # host code, so retain the empirically validated 3-byte payload.
            self._send(
                self.box_addr, 0xFF, FN["EXTRUDE_PROCESS"],
                bytes([slot, 0x00, 0x00]))
            self._pause(0.3)
            self._send(
                self.box_addr, 0xFF, FN["EXTRUDE_PROCESS"],
                bytes([slot, 0x04, 0x00]))
            self._pause(0.3)

            self._extrude_until_toolhead_sensor(
                gcmd, slot, polls, 0.4)

            handoff_ok = self._extrude_material_handoff(gcmd, slot)
            if not handoff_ok:
                raise gcmd.error(
                    "CFS_EXTRUDE: handoff did not confirm success; refusing "
                    "to enter print mode")

            # Mark this slot as the active PRINT-mode slot after handoff.
            self._send(
                self.box_addr, 0xFF, FN["SET_BOX_MODE"], bytes([slot, 0x00]))
            return handoff_ok
        except Exception as exc:
            load_error = exc
            raise
        finally:
            cleanup_error = self._cleanup_box_after_load()
            if load_error is None and cleanup_error is not None:
                raise cleanup_error

    def cmd_CFS_EXTRUDE(self, gcmd):
        # STATUS 2026-08-16/17: slot switching itself is confirmed working
        # live on all 4 slots via this command. What ISN'T yet confirmed
        # live is the stage 6/7 handoff rewrite in
        # _extrude_material_handoff() (2026-08-17, see its own docstring
        # and FINDINGS.md) - a faithful port of the real firmware's
        # retry+measuring-wheel-verified logic, replacing an earlier
        # single-shot version that reliably got filament to the toolhead
        # sensor but not reliably past the extruder gear and out the
        # nozzle. Test that specifically before trusting this for real
        # prints - see docs/TOOLCHANGE_TEST_PLAN.md.
        #
        # Below: history of how slot-switching itself got fixed (not yet
        # updated since it happened - after exhausting live guessing
        # (7 failed attempts at slots other than A, always the same
        # EXTRUDE_ERR8/FR2832 failure), we downloaded Creality's real
        # official K1C firmware for this board variant and decompiled its
        # actual box.py modules - see FINDINGS.md's "PRŮLOM" section. That
        # gave us ground truth instead of guesses:
        #   1. An error-clear-equivalent BEFORE starting, not just cleanup
        #      after (best effort - we log GET_BOX_STATE, then SET_BOX_MODE
        #      IDLE; we don't know exactly what stock BOX_ERROR_CLEAR puts
        #      on the wire since box_wrapper.py didn't decompile cleanly).
        #   2. BOX_GO_TO_EXTRUDE_POS - move the toolhead to a specific
        #      position before EXTRUDE_PROCESS (coords from a real factory
        #      box.cfg, see __init__ / printer.cfg to override).
        #   3. THE key missing piece: the real sequence does NOT stop after
        #      polling stage 5 until the toolhead sensor trips, like we
        #      always did. It continues: M83 + a slow toolhead-side G0 E10
        #      F35 move, THEN EXTRUDE_PROCESS stage 6, THEN another M83 + an
        #      even slower G0 E5 F10, THEN stage 7 - and only then marks the
        #      slot as loaded via the per-slot PRINT form of SET_BOX_MODE
        #      (payload [slot_bitmask, 0x00], confirmed from the real
        #      firmware - our old belief that SET_BOX_MODE always took a
        #      fixed [0x00, mode] pair was wrong, see set_box_mode() below).
        #      We had NEVER done any of this - we always stopped right
        #      after stage 5 and went straight to cleanup. This may be why
        #      the box never properly "finished" a load internally, which
        #      would explain both the post-run latched-error gotcha above
        #      AND why it could never switch to a different slot afterwards.
        #   4. EXTRUDE_PROCESS's real payload is [slot, stage] (2 bytes),
        #      not the 3 bytes ([slot, stage, 0x00]) we always sent - real
        #      firmware only appends an extra byte (fixed 0x02) for stage 7
        #      specifically. Fixed below; probably harmless before, but not
        #      what the real protocol does.
        # Still not implemented: BOX_NOZZLE_CLEAN (a wipe step the stock
        # sequence also runs) and the cutter-homing RS485 exchange the real
        # firmware does via its own cut_action object before every load
        # (we currently home the cutter with plain G-code moves instead,
        # which is physically confirmed to work, just not how stock does it).
        slot_letter = gcmd.get("SLOT", "A").upper()
        if slot_letter not in SLOT_BYTES:
            raise gcmd.error("SLOT must be one of A, B, C, D")
        slot = SLOT_BYTES[slot_letter]
        polls = gcmd.get_int("POLLS", 50, minval=1, maxval=200)

        toolhead = self.printer.lookup_object("toolhead")
        toolhead_status = toolhead.get_status(self.reactor.monotonic())
        homed = toolhead_status["homed_axes"]
        original_accel = toolhead_status["max_accel"]
        if not all(axis in homed for axis in "xyz"):
            raise gcmd.error("CFS_EXTRUDE: home the printer first (G28) - "
                              "refusing to move to the extrude position unhomed")
        has_required_positions = None not in (
            self.purge_min_z, self.purge_entry_x, self.purge_entry_y)
        has_purge_displacement = has_required_positions and (
            (self.purge_x is not None
             and float("%.2f" % self.purge_x)
             != float("%.2f" % self.purge_entry_x))
            or (self.purge_y is not None
                and float("%.2f" % self.purge_y)
                != float("%.2f" % self.purge_entry_y)))
        if not has_purge_displacement:
            raise gcmd.error(
                "CFS_EXTRUDE: purge_min_z and purge_entry_x/y must be set, "
                "and purge_x/purge_y must define non-zero bucket geometry, "
                "in [creality_cfs]. "
                "Refusing to guess bucket geometry; calibrate these positions "
                "with supervised manual jogs first (see docs/MANUAL.md).")

        self._reset_pre_loading()

        # Step 1: error-clear-equivalent (see note above - best effort only)
        status = self._send(self.box_addr, 0xFF, FN["GET_BOX_STATE"])
        if status:
            gcmd.respond_info("CFS_EXTRUDE: pre-run GET_BOX_STATE=%s" % status.hex())
        self._send(self.box_addr, 0xFF, FN["SET_BOX_MODE"], bytes([0x00, 0x01]))
        self._pause(0.3)

        # Step 2: save the caller's coordinate state, lift to a safe travel Z,
        # approach the bucket, then move inward to actuate it. Keep the inward
        # move separate so a failure during the safe approach restores state
        # without performing a potentially unsafe wipe sequence.
        state_saved = False
        bucket_entry_commanded = False
        operation_error = None
        try:
            self.gcode.run_script_from_command(
                "SAVE_GCODE_STATE NAME=CFS_EXTRUDE")
            state_saved = True
            gcode_move = self.printer.lookup_object("gcode_move")
            move_status = gcode_move.get_status(self.reactor.monotonic())
            current_z = move_status["gcode_position"][2]
            self._approach_purge_bucket(current_z)
            self._move_into_purge_bucket(self.purge_move_speed)
            bucket_entry_commanded = True
            self.gcode.run_script_from_command("M400")
            handoff_ok = self._load_filament_in_bucket(gcmd, slot, polls)
        except Exception as exc:
            operation_error = exc
            raise
        finally:
            if state_saved:
                try:
                    if bucket_entry_commanded:
                        # Even a failed entry wait or load must leave the
                        # nozzle outside and restore acceleration/state.
                        self._exit_purge_bucket(original_accel)
                    else:
                        self.gcode.run_script_from_command(
                            "RESTORE_GCODE_STATE NAME=CFS_EXTRUDE")
                except Exception:
                    if operation_error is None:
                        raise
                    logging.exception(
                        "creality_cfs: bucket recovery failed while handling "
                        "an earlier CFS_EXTRUDE error")
        gcmd.respond_info("CFS_EXTRUDE slot=%s complete (%d polls, handoff %s)" % (
            slot_letter, polls, "confirmed" if handoff_ok else "NOT confirmed - check physically"))


def load_config(config):
    return CrealityCFS(config)
