"""
Simple data logger for a BMW MSS54HP DME (S54) running CSL firmware.

Logs manifold absolute pressure plus a set of standard engine parameters to CSV, one row with a
timestamp every --interval-ms milliseconds.

Talks BMW DS2 directly over a K+DCAN / INPA cable (K-line, 9600 8E1, DME address 0x12).
No EDIABAS needed. Read-only: only identification (0x00), status-block (0x0B) and RAM-read (0x06,
same as the SGBD's RAM_LESEN job) requests are sent.

Examples:
    python mss54_logger.py                            # COM1 (Windows) / auto-detected port (macOS)
    python3 mss54_logger.py --port /dev/cu.usbserial-A50285BI   # explicit port on macOS
    python mss54_logger.py --list-ports
    python mss54_logger.py --interval-ms 1000 --csv run1.csv
    python mss54_logger.py --knock-adapt-every 5      # refresh knock adaptation every 5th sample
    python mss54_logger.py --discover 0x23            # dump a raw block to locate fields

Requires: pip install pyserial
Runs on Windows and macOS (Intel or Apple Silicon). FTDI-based K+DCAN cables use the driver built
into macOS; CH340-based clones need the WCH driver.
"""

import argparse
import csv
import sys
import time
from datetime import datetime

import serial
from serial.tools import list_ports

DME_ADDRESS = 0x12
STATUS_OK = 0xA0


def c_to_f(c):
    return c * 9 / 5 + 32


def kmh_to_mph(kmh):
    return kmh * 0.621371


# DS2 RAM window read with request 0x06 (segment 0x01 = 0xFF8xxx RAM). Symbol addresses come from
# karter16's MSS54 Ghidra project and were checked live on CSL 0401 (TZ[] equals block 0x13).
#   0xFF81FC TZ[1..8]      ignition angle per cylinder (0.1 deg)
#   0xFF820C TZ_VE[1..8]   ignition angle per cylinder before knock retard (0.1 deg)
#   0xFF8252 KR_DTZ_SUM, KR_DTZ_SUM1, KR_DTZ_SUM2   knock retard sum: all / bank 1 / bank 2
RAM_IGN = "ram_ign"
RAM_IGN_START = 0xFF81FC
RAM_READS = {RAM_IGN: [0x06, 0x01, 0xFF, 0x81, 0xFC, 0x62]}


def ram_pos(address):
    return 3 + address - RAM_IGN_START


# Knock adaptation (block 0x16) changes slowly and costs ~200 ms to read, so it is refreshed less often.
KNOCK_ADAPT_BLOCK = 0x16

# (column, source, byte position, type, factor, offset, convert, decimals)
# source = DS2 status block number, or a RAM_READS key.
# Positions use the EDIABAS convention: 0 = address byte, 3 = first data byte.
# Layout from the BMW SGBD mss54ds0.prg (MSS52/MSS54/MSS54HP), checked live on CSL program 0401.
SIGNALS = [
    ("Engine Load [ms]",                 0x03,  9, "u16", 0.001,       0,   None,       3),
    ("Engine Speed [RPM]",               0x03,  3, "u16", 1,           0,   None,       0),
    ("Engine Temp [deg F]",              0x03, 14, "u8",  1,         -48,   c_to_f,     0),
    # TABG is EGT/16 as a signed byte (the SGBD's factor of 1 gives ~11 degC at idle, which is wrong).
    ("Exhaust Gas Temp [deg F]",         0x03, 17, "s8",  16,          0,   c_to_f,     0),
    ("Idle Speed Target [RPM]",          0x03,  5, "u16", 1,           0,   None,       0),
    ("Intake Air Temp [deg F]",          0x03, 13, "u8",  1,         -48,   c_to_f,     0),
    ("Mass Air Flow [kg/h]",             0x03,  7, "u16", 0.25,        0,   None,       2),
    ("Oil Temp [deg F]",                 0x03, 15, "u8",  1,         -48,   c_to_f,     0),
    ("Pedal Angle 1 [%]",                0x03, 26, "s16", 0.1,         0,   None,       1),
    ("Pedal Angle 2 [%]",                0x03, 28, "s16", 0.1,         0,   None,       1),
    ("Relative Filling [%]",             0x03, 11, "u16", 0.1,         0,   None,       1),
    ("Throttle Angle: Actual [%]",       0x03, 30, "s16", 0.1,         0,   None,       1),
    ("Throttle Angle: Target [%]",       0x03, 34, "s16", 0.1,         0,   None,       1),
    *[(f"Ignition Angle (Cylinder {n}) [deg]", 0x13, 23 + 2 * n, "s16", 0.1, 0, None, 1) for n in range(1, 7)],
    *[(f"Injection Time (Cylinder {n}) [ms]",  0x13,  7 + 2 * n, "u16", 0.001, 0, None, 3) for n in range(1, 7)],
    ("Lambda Integrator (Bank 1) [-]",   0x13, 43, "u16", 1 / 32768,   0,   None,       4),
    ("Lambda Integrator (Bank 2) [-]",   0x13, 45, "u16", 1 / 32768,   0,   None,       4),
    ("Vehicle Speed [mph]",              0x13,  3, "u16", 1,           0,   kmh_to_mph, 1),
    ("Exhaust VANOS: Actual (Bank 1) [deg]", 0x23, 9, "s16", 0.1,      0,   None,       1),
    ("Exhaust VANOS: Target (Bank 1) [deg]", 0x23, 11, "s16", 0.1,     0,   None,       1),
    ("Intake VANOS: Actual (Bank 1) [deg]",  0x23, 5, "s16", 0.1,      0,   None,       1),
    ("Intake VANOS: Target (Bank 1) [deg]",  0x23, 7, "s16", 0.1,      0,   None,       1),
    # MAP: found by experiment on CSL 0401 (engine off ~= ambient, warm idle ~300 mbar).
    ("Manifold Abs Pressure [mbar]",     0x23, 25, "u16", 1 / 32,      0,   None,       1),
    ("Ambient Pressure [mbar]",          0x03, 22, "u8",  3,         500,   None,       0),
    # Live knock retard per cylinder = TZ_VE[n] - TZ[n] (see "tzdiff" in get_raw).
    *[(f"Knock Retard (Cylinder {n}) [deg]", RAM_IGN, ram_pos(0xFF81FC + 2 * (n - 1)), "tzdiff", 0.1, 0, None, 1)
      for n in range(1, 7)],
    ("Knock Retard Sum [deg]",           RAM_IGN, ram_pos(0xFF8252), "s16", 0.1,   0,   None,       1),
    ("Knock Retard Sum (Bank 1) [deg]",  RAM_IGN, ram_pos(0xFF8254), "s16", 0.1,   0,   None,       1),
    ("Knock Retard Sum (Bank 2) [deg]",  RAM_IGN, ram_pos(0xFF8256), "s16", 0.1,   0,   None,       1),
    # ka_adap_tz[n]: learned knock adaptation, int32, 0.1 deg / 16384 per bit.
    *[(f"Knock Adaptation (Cylinder {n}) [deg]", KNOCK_ADAPT_BLOCK, 51 + 4 * (n - 1), "s32", 6.103515625e-06, 0,
       None, 3) for n in range(1, 7)],
]


class Ds2Error(Exception):
    pass


def open_kline(port):
    ser = serial.Serial(
        port=port,
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        timeout=2.0,
        write_timeout=1.0,
    )
    # K+DCAN cables: DTR low = K-line mode
    ser.dtr = False
    ser.rts = False
    ser.reset_input_buffer()
    return ser


def read_exact(ser, count):
    data = ser.read(count)
    if len(data) != count:
        raise TimeoutError("No response from DME (ignition on?)")
    return data


def xor_checksum(data):
    cs = 0
    for b in data:
        cs ^= b
    return cs


def ds2_request(ser, payload):
    # Frame: [addr][len][payload...][xor]; len counts the whole frame.
    frame = bytearray([DME_ADDRESS, len(payload) + 3]) + bytes(payload)
    frame.append(xor_checksum(frame))

    ser.reset_input_buffer()
    ser.write(frame)

    # Half-duplex K-line: our own bytes come back first.
    if read_exact(ser, len(frame)) != frame:
        raise Ds2Error("Echo mismatch (check cable/ignition)")

    header = read_exact(ser, 2)
    length = header[1]
    if length < 4:
        raise Ds2Error(f"Bad response length {length}")
    resp = header + read_exact(ser, length - 2)

    if xor_checksum(resp[:-1]) != resp[-1]:
        raise Ds2Error("Response checksum error")
    if resp[2] != STATUS_OK:
        raise Ds2Error(f"DME rejected request, status 0x{resp[2]:02X}")
    return resp


def read_block(ser, selection):
    return ds2_request(ser, [0x0B, selection])


def read_source(ser, source):
    if source in RAM_READS:
        return ds2_request(ser, RAM_READS[source])
    return read_block(ser, source)


def get_raw(resp, pos, kind):
    if kind in ("u8", "s8"):
        value = resp[pos]
        return value - 0x100 if kind == "s8" and value >= 0x80 else value
    if kind == "s32":
        return int.from_bytes(resp[pos:pos + 4], "big", signed=True)
    if kind == "tzdiff":
        # TZ_VE[n] sits 16 bytes after TZ[n].
        return get_raw(resp, pos + 0x10, "s16") - get_raw(resp, pos, "s16")
    value = (resp[pos] << 8) | resp[pos + 1]
    if kind == "s16" and value >= 0x8000:
        value -= 0x10000
    return value


def decode(blocks, signal):
    _, block, pos, kind, factor, offset, convert, decimals = signal
    value = get_raw(blocks[block], pos, kind) * factor + offset
    if convert:
        value = convert(value)
    return round(value, decimals) if decimals else int(round(value))


def print_ident(ser):
    ident = ds2_request(ser, [0x00])
    text = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in ident[3:-1])
    print(f"DME ident: {text}")


def discover(ser, block):
    print(f"Discover mode - block 0x{block:02X}, Ctrl+C to stop.")
    first = True
    while True:
        b3 = read_block(ser, 0x03)
        bm = b3 if block == 0x03 else read_block(ser, block)
        data = bm[3:-1]
        if first:
            print(f"{'pos ->':<11}" + " ".join(f"{(i + 3) % 100:2d}" for i in range(len(data))))
            first = False
        print(f"rpm {get_raw(b3, 3, 'u16'):5d}  " + data.hex(" ").upper())
        time.sleep(0.25)


def log(ser, args):
    interval = args.interval_ms / 1000.0
    sources = list(dict.fromkeys(s[1] for s in SIGNALS))
    fast_sources = [s for s in sources if s != KNOCK_ADAPT_BLOCK]
    col = {s[0]: i for i, s in enumerate(SIGNALS)}
    i_rpm, i_map = col["Engine Speed [RPM]"], col["Manifold Abs Pressure [mbar]"]
    i_clt, i_rf = col["Engine Temp [deg F]"], col["Relative Filling [%]"]
    i_kr = [col[f"Knock Retard (Cylinder {n}) [deg]"] for n in range(1, 7)]

    print(f"Logging {len(SIGNALS)} channels every {args.interval_ms} ms to {args.csv} "
          f"(knock adaptation refreshed every {args.knock_adapt_every} samples) - Ctrl+C to stop")

    with open(args.csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Elapsed [s]"] + [s[0] for s in SIGNALS])

        start = time.perf_counter()
        next_due = start
        overrun_warned = False
        sample_no = 0
        knock_adapt = None
        while True:
            # Fixed-rate schedule: sleep until the next slot, not a fixed delay after each sample.
            now = time.perf_counter()
            if now < next_due:
                time.sleep(next_due - now)
            sample_time = datetime.now().astimezone()
            elapsed = time.perf_counter() - start

            slow_sample = knock_adapt is None or sample_no % args.knock_adapt_every == 0
            try:
                if slow_sample:
                    knock_adapt = read_block(ser, KNOCK_ADAPT_BLOCK)
                blocks = {s: read_source(ser, s) for s in fast_sources}
                blocks[KNOCK_ADAPT_BLOCK] = knock_adapt
                values = [decode(blocks, s) for s in SIGNALS]
                writer.writerow([sample_time.isoformat(timespec="milliseconds"), f"{elapsed:.3f}"] + values)
                f.flush()
                sample_no += 1
                print(f"{elapsed:8.2f}s  rpm {values[i_rpm]:5d}  MAP {values[i_map]:6.1f} mbar  "
                      f"RF {values[i_rf]:5.1f} %  coolant {values[i_clt]} F  "
                      f"KR " + " ".join(f"{values[i]:.1f}" for i in i_kr))
            except (TimeoutError, Ds2Error, serial.SerialException) as e:
                print(f"WARNING: {e}", file=sys.stderr)

            next_due += interval
            if time.perf_counter() > next_due:
                if not overrun_warned and not slow_sample:
                    print(f"WARNING: a sample takes longer than {args.interval_ms} ms at 9600 baud; "
                          "logging as fast as the DME allows.", file=sys.stderr)
                    overrun_warned = True
                next_due = time.perf_counter()


def parse_args():
    p = argparse.ArgumentParser(description="MSS54HP (CSL) engine data logger over DS2/K-line")
    p.add_argument("--port", default=None,
                   help="serial port (default: COM1 on Windows; auto-detected USB serial adapter on "
                        "macOS/Linux, e.g. /dev/cu.usbserial-XXXX)")
    p.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    p.add_argument("--interval-ms", type=int, default=700,
                   help="time between CSV samples in ms (default 700; ~600 ms is the minimum at 9600 baud)")
    p.add_argument("--knock-adapt-every", type=int, default=10, metavar="N",
                   help="re-read knock adaptation (block 0x16, ~200 ms) every N samples (default 10)")
    p.add_argument("--csv", default=f"dme_log_{datetime.now():%Y%m%d_%H%M%S}.csv")
    p.add_argument("--discover", type=lambda s: int(s, 0), metavar="BLOCK",
                   help="dump a raw DS2 status block (e.g. 0x23) instead of logging")
    args = p.parse_args()
    if args.interval_ms <= 0:
        p.error("--interval-ms must be positive")
    if args.knock_adapt_every <= 0:
        p.error("--knock-adapt-every must be positive")
    return args


def default_port():
    if sys.platform == "win32":
        return "COM1"
    # macOS: use /dev/cu.* (the /dev/tty.* twin blocks on open waiting for carrier detect).
    candidates = sorted(
        p.device for p in list_ports.comports()
        if p.device.startswith(("/dev/cu.usbserial", "/dev/ttyUSB"))
        or (p.vid == 0x0403 and not p.device.startswith("/dev/tty."))  # FTDI
    )
    if not candidates:
        sys.exit("No USB serial adapter found - plug in the K+DCAN cable or pass --port "
                 "(see --list-ports).")
    if len(candidates) > 1:
        print(f"Several serial adapters found, using {candidates[0]} "
              f"(others: {', '.join(candidates[1:])})", file=sys.stderr)
    return candidates[0]


def main():
    args = parse_args()
    if args.list_ports:
        for p in list_ports.comports():
            print(f"{p.device:30s} {p.description}")
        return
    port = args.port or default_port()
    print(f"Port: {port}")
    ser = open_kline(port)
    try:
        print_ident(ser)
        if args.discover is not None:
            discover(ser, args.discover)
        else:
            log(ser, args)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
