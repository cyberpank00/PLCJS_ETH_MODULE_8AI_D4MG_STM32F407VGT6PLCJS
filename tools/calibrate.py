#!/usr/bin/env python3
"""
calibrate.py - dialog-driven calibration / configuration tool for the
PLCJS 8AIC analog current-input module (8x 4-20 mA, ADS1220) over Modbus TCP.

Pure standard-library Python 3.8+ (socket, struct, argparse) - no pymodbus
required. Functionally identical to tools/calibrate.mjs.

The module measures  I_raw = code / 2^23 * V_REF / R_shunt  (single-ended,
gain 1, V_REF = 2.048 V, R_shunt ~ 89.9 Ohm). Per-channel linear model fitted
over >= 2 points from a precision current source:

    I_true = gain * I_raw + offset        (mA)

Coefficients written to 540+ are a LIVE PREVIEW. Persisting them is a
write-once COMMIT (HR131 = 0xCA00 | ch) that locks the channel slot forever;
the tool asks for explicit confirmation before committing.

Examples:
    python calibrate.py status --ip 192.168.1.13
    python calibrate.py calibrate --ch 0
    python calibrate.py set --ch 0 --enable 1 --lo 4000 --hi 20000
    python calibrate.py rate 0            # 0 = 20 SPS+FIR, 1 = 90 SPS, 2 = 330 SPS
"""

import argparse
import socket
import struct
import sys
import time

# ------------------------------ register map ------------------------------
IR_CURRENT, IR_RAW, IR_FLAGS, IR_CODE, IR_END = 300, 316, 332, 340, 356
IR_MODULE_ID, IR_CAL_LOCK = 125, 127

HR_READING, HR_SCALE_LO, HR_SCALE_HI, HR_ENABLED, HR_SMOOTH = 0, 8, 16, 24, 32
HR_CAL_BASE, HR_CAL_STRIDE = 540, 4
HR_TRIG_SAVE, TRIG_SAVE = 117, 0xA5A5
HR_CAL_COMMIT, CAL_COMMIT_BASE = 131, 0xCA00
HR_ADC_RATE = 133

CHANNELS = 8
FAULT_NAME = {1: "OPEN", 2: "OVER", 3: "ADC"}
RATE_NAME = ["20 SPS+FIR", "90 SPS", "330 SPS"]


# --------------------------- Modbus TCP client ----------------------------
class ModbusTCP:
    def __init__(self, ip, port=502, unit=1, timeout=3.0):
        self.ip, self.port, self.unit, self.timeout = ip, port, unit, timeout
        self.txid = 0
        self.sock = None

    def connect(self):
        self.sock = socket.create_connection((self.ip, self.port), self.timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def close(self):
        if self.sock:
            self.sock.close()

    def _txn(self, pdu):
        self.txid = (self.txid + 1) & 0xFFFF
        header = struct.pack(">HHHB", self.txid, 0, len(pdu) + 1, self.unit)
        self.sock.sendall(header + pdu)
        # read MBAP header
        buf = b""
        while len(buf) < 7:
            chunk = self.sock.recv(260)
            if not chunk:
                raise IOError("connection closed")
            buf += chunk
        length = struct.unpack(">H", buf[4:6])[0]
        while len(buf) < 6 + length:
            chunk = self.sock.recv(260)
            if not chunk:
                raise IOError("connection closed")
            buf += chunk
        fn = buf[7]
        if fn & 0x80:
            raise IOError("Modbus exception %d" % buf[8])
        return buf[8:6 + length]

    def read_input(self, addr, qty):
        data = self._txn(struct.pack(">BHH", 0x04, addr, qty))
        n = data[0]
        return list(struct.unpack(">%dH" % (n // 2), data[1:1 + n]))

    def read_holding(self, addr, qty):
        data = self._txn(struct.pack(">BHH", 0x03, addr, qty))
        n = data[0]
        return list(struct.unpack(">%dH" % (n // 2), data[1:1 + n]))

    def write_multiple(self, addr, regs):
        pdu = struct.pack(">BHHB", 0x10, addr, len(regs), len(regs) * 2)
        pdu += b"".join(struct.pack(">H", r & 0xFFFF) for r in regs)
        self._txn(pdu)

    def write_single(self, addr, val):
        self._txn(struct.pack(">BHH", 0x06, addr, val & 0xFFFF))


# ------------------------------ float codec -------------------------------
def regs_to_float(hi, lo):
    return struct.unpack(">f", struct.pack(">HH", hi, lo))[0]


def float_to_regs(f):
    return list(struct.unpack(">HH", struct.pack(">f", f)))



# ------------------------------- helpers ----------------------------------
def i16(v):
    return v - 0x10000 if v > 0x7FFF else v


def read_all(mb):
    r = mb.read_input(IR_CURRENT, IR_END - IR_CURRENT)  # 300..355
    out = []
    for ch in range(CHANNELS):
        def f32(base):
            i = base - 300 + ch * 2
            return regs_to_float(r[i], r[i + 1])
        code = (r[IR_CODE - 300 + ch * 2] << 16) | r[IR_CODE - 300 + ch * 2 + 1]
        if code & 0x80000000:
            code -= 0x100000000
        out.append({"cur": f32(IR_CURRENT), "raw": f32(IR_RAW),
                    "flags": r[IR_FLAGS - 300 + ch], "code": code})
    return out


def read_raw_averaged(mb, ch, samples=8, delay=0.3):
    total, n, fault = 0.0, 0, False
    for _ in range(samples):
        s = read_all(mb)[ch]
        if s["flags"] & 0x0004:
            fault = True
        if s["raw"] == s["raw"]:  # not NaN
            total += s["raw"]
            n += 1
        time.sleep(delay)
    return (total / n if n else float("nan"), fault)


def linfit(points):
    n = len(points)
    sx = sum(x for x, _ in points)
    sy = sum(y for _, y in points)
    sxx = sum(x * x for x, _ in points)
    sxy = sum(x * y for x, y in points)
    denom = n * sxx - sx * sx
    gain = (n * sxy - sx * sy) / denom
    offset = (sy - gain * sx) / n
    max_err = max(abs(gain * x + offset - y) for x, y in points)
    return gain, offset, max_err


# ------------------------------- commands ---------------------------------
def cmd_status(mb, _args):
    mid = mb.read_input(IR_MODULE_ID, 1)[0]
    lock = mb.read_input(IR_CAL_LOCK, 1)[0]
    cfg = mb.read_holding(0, 40)
    rate = mb.read_holding(HR_ADC_RATE, 1)[0]
    print("Module ID: 0x%04X  ADC rate: %s" % (mid, RATE_NAME[rate] if rate < 3 else rate))
    for ch, s in enumerate(read_all(mb)):
        flags = [n for b, n in ((1, "EN"), (2, "VALID"), (4, "FAULT")) if s["flags"] & b]
        fcode = (s["flags"] >> 8) & 0xFF
        print("CH%d: scale %d..%d uA  I=%.4f mA  raw=%.4f mA  i16=%d  code=%d  [%s]%s%s" % (
            ch, cfg[HR_SCALE_LO + ch], cfg[HR_SCALE_HI + ch], s["cur"], s["raw"],
            i16(cfg[HR_READING + ch]), s["code"], ",".join(flags),
            (" fault=%s" % FAULT_NAME.get(fcode, fcode)) if fcode else "",
            "  LOCKED" if (lock >> ch) & 1 else ""))


def calibrate_channel(mb, ch):
    print("\n=== Calibrating CH%d ===" % ch)
    points = []
    while True:
        print("\nSource a known current into CH%d (+ / GND_ISO) from the calibrator." % ch)
        ans = input('Enter applied current in mA (or "done", need >= 2 points): ').strip()
        if ans.lower() == "done":
            if len(points) >= 2:
                break
            print("Need at least 2 points.")
            continue
        try:
            i_true = float(ans.replace(",", "."))
        except ValueError:
            print("Invalid number.")
            continue
        sys.stdout.write("Measuring raw current ...\n")
        raw, fault = read_raw_averaged(mb, ch)
        print("-> I_raw = %.5f mA%s" % (raw, "  (WARNING: channel fault!)" if fault else ""))
        if raw != raw:
            print("No reading; skipped.")
            continue
        points.append((raw, i_true))
        print("Recorded point %d: I_raw=%.5f -> I_true=%s" % (len(points), raw, i_true))

    gain, offset, max_err = linfit(points)
    print("\nFit: gain=%.7f  offset=%.5f mA  max residual=%.5f mA" % (gain, offset, max_err))

    cal_base = HR_CAL_BASE + ch * HR_CAL_STRIDE
    mb.write_multiple(cal_base, float_to_regs(gain) + float_to_regs(offset))
    print("Wrote coefficients (live preview) to holding regs %d..%d." % (cal_base, cal_base + 3))

    ans = input("\nCOMMIT CH%d to write-once Flash? This is IRREVERSIBLE. "
                'Type "COMMIT" to proceed: ' % ch).strip()
    if ans == "COMMIT":
        mb.write_single(HR_CAL_COMMIT, CAL_COMMIT_BASE | ch)
        print("Committed and locked.")
    else:
        print("Not committed (preview stays active until reboot).")


def cmd_calibrate(mb, args):
    channels = list(range(CHANNELS)) if args.all else [args.ch]
    if any(c is None or not (0 <= c < CHANNELS) for c in channels):
        raise SystemExit("Specify --ch 0..7 or --all")
    for ch in channels:
        calibrate_channel(mb, ch)


def cmd_set(mb, args):
    ch = args.ch
    if ch is None or not (0 <= ch < CHANNELS):
        raise SystemExit("Specify --ch 0..7")
    if args.lo is not None or args.hi is not None:
        # Thresholds are cross-validated by the firmware (lo < hi): raise hi
        # first when the span moves up, lower lo first when it moves down.
        cur_lo = mb.read_holding(HR_SCALE_LO + ch, 1)[0]
        cur_hi = mb.read_holding(HR_SCALE_HI + ch, 1)[0]
        lo = args.lo if args.lo is not None else cur_lo
        hi = args.hi if args.hi is not None else cur_hi
        if not hi > lo:
            raise SystemExit("--hi must be greater than --lo")
        order = ((HR_SCALE_HI, hi), (HR_SCALE_LO, lo)) if hi > cur_hi else ((HR_SCALE_LO, lo), (HR_SCALE_HI, hi))
        for reg, val in order:
            mb.write_single(reg + ch, val)
        print("CH%d scale = %d..%d uA" % (ch, lo, hi))
    if args.smooth is not None:
        mb.write_single(HR_SMOOTH + ch, args.smooth)
    if args.enable is not None:
        mb.write_single(HR_ENABLED + ch, 1 if args.enable else 0)
    mb.write_single(HR_TRIG_SAVE, TRIG_SAVE)
    print("Configuration saved.")


def cmd_rate(mb, args):
    if args.value is None or not (0 <= args.value <= 2):
        raise SystemExit("rate value must be 0 (20 SPS+FIR), 1 (90 SPS) or 2 (330 SPS)")
    mb.write_single(HR_ADC_RATE, args.value)
    mb.write_single(HR_TRIG_SAVE, TRIG_SAVE)
    print("ADC rate = %d (%s), saved." % (args.value, RATE_NAME[args.value]))


def main():
    p = argparse.ArgumentParser(description="PLCJS 8AIC calibration tool (Modbus TCP)")
    p.add_argument("command", choices=["status", "calibrate", "set", "rate"])
    p.add_argument("value", type=int, nargs="?", help="rate value for the 'rate' command")
    p.add_argument("--ip", default="192.168.1.13")
    p.add_argument("--port", type=int, default=502)
    p.add_argument("--unit", type=int, default=1)
    p.add_argument("--ch", type=int)
    p.add_argument("--all", action="store_true")
    p.add_argument("--lo", type=int, help="scale low threshold, uA (default 4000)")
    p.add_argument("--hi", type=int, help="scale high threshold, uA (default 20000)")
    p.add_argument("--smooth", type=int, choices=range(4))
    p.add_argument("--enable", type=int)
    args = p.parse_args()

    mb = ModbusTCP(args.ip, args.port, args.unit)
    print("Connecting to %s:%d (unit %d) ..." % (args.ip, args.port, args.unit))
    mb.connect()
    try:
        {"status": cmd_status, "calibrate": cmd_calibrate, "set": cmd_set, "rate": cmd_rate}[args.command](mb, args)
    finally:
        mb.close()


if __name__ == "__main__":
    main()