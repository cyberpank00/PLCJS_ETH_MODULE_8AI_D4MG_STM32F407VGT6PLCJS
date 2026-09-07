#!/usr/bin/env node
/**
 * calibrate.mjs — dialog-driven calibration / configuration tool for the
 * PLCJS 8AIC analog current-input module (8× 4–20 mA, ADS1220) over Modbus TCP.
 *
 * Node.js 18+ only, no external npm packages (uses node:net / node:readline).
 *
 * The module measures  I_raw = code / 2^23 * V_REF / R_shunt  (single-ended,
 * gain 1, V_REF = 2.048 V, R_shunt ≈ 89.9 Ω). Shunt tolerance, reference
 * error and the 1 kΩ / input-impedance divider are removed per channel with a
 * linear model fitted over >= 2 points from a precision current source:
 *
 *      I_true = gain * I_raw + offset            (mA)
 *
 * Coefficients written to 540+ are a LIVE PREVIEW. Persisting them is a
 * write-once COMMIT (HR131 = 0xCA00 | ch) that locks the channel slot forever;
 * the tool asks for explicit confirmation before committing.
 *
 * Usage:
 *   node calibrate.mjs status                 [--ip A.B.C.D] [--port 502]
 *   node calibrate.mjs calibrate --ch N | --all
 *   node calibrate.mjs set --ch N [--enable 0|1] [--lo uA] [--hi uA] [--smooth 0..3]
 *   node calibrate.mjs rate 0|1|2             (20 SPS+FIR / 90 SPS / 330 SPS)
 */

import net from 'node:net';
import readline from 'node:readline';

/* ----------------------------- register map ----------------------------- */
const MB = {
  // input registers (readings), grouped by quantity, 8 channels each
  IR_CURRENT: 300, IR_RAW: 316, IR_FLAGS: 332, IR_CODE: 340, IR_END: 356,
  IR_MODULE_ID: 125, IR_CAL_LOCK: 127,
  // compact holding block: group*8 + ch
  HR_READING: 0, HR_SCALE_LO: 8, HR_SCALE_HI: 16, HR_ENABLED: 24, HR_SMOOTH: 32,
  // calibration coefficients: 540 + ch*4 -> gain(2), offset(2)
  HR_CAL_BASE: 540, HR_CAL_STRIDE: 4,
  HR_TRIG_SAVE: 117, TRIG_SAVE: 0xA5A5,
  HR_CAL_COMMIT: 131, CAL_COMMIT_BASE: 0xCA00,
  HR_ADC_RATE: 133,
};

const CHANNELS = 8;
const FAULT_NAME = { 1: 'OPEN', 2: 'OVER', 3: 'ADC' };

/* --------------------------- Modbus TCP client -------------------------- */
class ModbusTCP {
  constructor(ip, port, unit = 1) { this.ip = ip; this.port = port; this.unit = unit; this.txid = 0; }

  connect() {
    return new Promise((resolve, reject) => {
      this.sock = net.connect({ host: this.ip, port: this.port }, () => resolve());
      this.sock.on('error', reject);
      this.sock.setNoDelay(true);
    });
  }
  close() { if (this.sock) this.sock.end(); }

  _txn(pdu) {
    return new Promise((resolve, reject) => {
      this.txid = (this.txid + 1) & 0xffff;
      const header = Buffer.alloc(7);
      header.writeUInt16BE(this.txid, 0);
      header.writeUInt16BE(0, 2);
      header.writeUInt16BE(pdu.length + 1, 4);
      header.writeUInt8(this.unit, 6);
      const frame = Buffer.concat([header, pdu]);

      let buf = Buffer.alloc(0);
      const onData = (chunk) => {
        buf = Buffer.concat([buf, chunk]);
        if (buf.length < 7) return;
        const len = buf.readUInt16BE(4);
        if (buf.length < 6 + len) return;
        cleanup();
        const fn = buf.readUInt8(7);
        if (fn & 0x80) { reject(new Error('Modbus exception ' + buf.readUInt8(8))); return; }
        resolve(buf.slice(8));
      };
      const onErr = (e) => { cleanup(); reject(e); };
      const to = setTimeout(() => { cleanup(); reject(new Error('timeout')); }, 3000);
      const cleanup = () => { clearTimeout(to); this.sock.removeListener('data', onData); this.sock.removeListener('error', onErr); };
      this.sock.on('data', onData);
      this.sock.on('error', onErr);
      this.sock.write(frame);
    });
  }

  async readInput(addr, qty) {
    const pdu = Buffer.alloc(5);
    pdu.writeUInt8(0x04, 0); pdu.writeUInt16BE(addr, 1); pdu.writeUInt16BE(qty, 3);
    const data = await this._txn(pdu);
    const n = data.readUInt8(0);
    const regs = [];
    for (let i = 0; i < n / 2; i++) regs.push(data.readUInt16BE(1 + i * 2));
    return regs;
  }
  async readHolding(addr, qty) {
    const pdu = Buffer.alloc(5);
    pdu.writeUInt8(0x03, 0); pdu.writeUInt16BE(addr, 1); pdu.writeUInt16BE(qty, 3);
    const data = await this._txn(pdu);
    const n = data.readUInt8(0);
    const regs = [];
    for (let i = 0; i < n / 2; i++) regs.push(data.readUInt16BE(1 + i * 2));
    return regs;
  }
  async writeMultiple(addr, regs) {
    const pdu = Buffer.alloc(6 + regs.length * 2);
    pdu.writeUInt8(0x10, 0); pdu.writeUInt16BE(addr, 1); pdu.writeUInt16BE(regs.length, 3);
    pdu.writeUInt8(regs.length * 2, 5);
    regs.forEach((r, i) => pdu.writeUInt16BE(r & 0xffff, 6 + i * 2));
    await this._txn(pdu);
  }
  async writeSingle(addr, val) {
    const pdu = Buffer.alloc(5);
    pdu.writeUInt8(0x06, 0); pdu.writeUInt16BE(addr, 1); pdu.writeUInt16BE(val & 0xffff, 3);
    await this._txn(pdu);
  }
}

/* ------------------------------ float codec ----------------------------- */
function regsToFloat(hi, lo) {
  const b = Buffer.alloc(4);
  b.writeUInt16BE(hi, 0); b.writeUInt16BE(lo, 2);
  return b.readFloatBE(0);
}
function floatToRegs(f) {
  const b = Buffer.alloc(4);
  b.writeFloatBE(f, 0);
  return [b.readUInt16BE(0), b.readUInt16BE(2)];
}

/* ----------------------------- helpers ---------------------------------- */
const rl = readline.createInterface({ input: process.stdin, output: process.stdout });
const ask = (q) => new Promise((res) => rl.question(q, res));
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function parseArgs(argv) {
  const a = { _: [] };
  for (let i = 0; i < argv.length; i++) {
    if (argv[i].startsWith('--')) { const k = argv[i].slice(2); const v = (argv[i + 1] && !argv[i + 1].startsWith('--')) ? argv[++i] : true; a[k] = v; }
    else a._.push(argv[i]);
  }
  return a;
}
async function readAll(mb) {
  const r = await mb.readInput(MB.IR_CURRENT, MB.IR_END - MB.IR_CURRENT);   // 300..355
  const out = [];
  for (let ch = 0; ch < CHANNELS; ch++) {
    const f32 = (base) => regsToFloat(r[base - 300 + ch * 2], r[base - 300 + ch * 2 + 1]);
    const hi = r[MB.IR_CODE - 300 + ch * 2], lo = r[MB.IR_CODE - 300 + ch * 2 + 1];
    out.push({
      cur: f32(MB.IR_CURRENT), raw: f32(MB.IR_RAW),
      flags: r[MB.IR_FLAGS - 300 + ch], code: ((hi << 16) | lo) | 0,
    });
  }
  return out;
}

async function readRawAveraged(mb, ch, samples = 8, delayMs = 300) {
  let sum = 0, n = 0, fault = false;
  for (let i = 0; i < samples; i++) {
    const s = (await readAll(mb))[ch];
    if (s.flags & 0x0004) fault = true;
    if (Number.isFinite(s.raw)) { sum += s.raw; n++; }
    await sleep(delayMs);
  }
  return { raw: n ? sum / n : NaN, fault };
}

function linfit(points) {
  const n = points.length;
  let sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (const [x, y] of points) { sx += x; sy += y; sxx += x * x; sxy += x * y; }
  const denom = n * sxx - sx * sx;
  const gain = (n * sxy - sx * sy) / denom;
  const offset = (sy - gain * sx) / n;
  let maxErr = 0;
  for (const [x, y] of points) maxErr = Math.max(maxErr, Math.abs(gain * x + offset - y));
  return { gain, offset, maxErr };
}

const i16 = (v) => (v > 0x7fff ? v - 0x10000 : v);

/* ------------------------------ commands -------------------------------- */
async function cmdStatus(mb) {
  const id = (await mb.readInput(MB.IR_MODULE_ID, 1))[0];
  const lock = (await mb.readInput(MB.IR_CAL_LOCK, 1))[0];
  const cfg = await mb.readHolding(0, 40);
  const rate = (await mb.readHolding(MB.HR_ADC_RATE, 1))[0];
  const all = await readAll(mb);
  console.log(`Module ID: 0x${id.toString(16)}  ADC rate: ${['20 SPS+FIR', '90 SPS', '330 SPS'][rate] ?? rate}`);
  for (let ch = 0; ch < CHANNELS; ch++) {
    const s = all[ch];
    const f = [];
    if (s.flags & 1) f.push('EN'); if (s.flags & 2) f.push('VALID'); if (s.flags & 4) f.push('FAULT');
    const fcode = (s.flags >> 8) & 0xff;
    console.log(`CH${ch}: scale ${cfg[MB.HR_SCALE_LO + ch]}..${cfg[MB.HR_SCALE_HI + ch]} uA  I=${s.cur.toFixed(4)} mA  ` +
      `raw=${s.raw.toFixed(4)} mA  i16=${i16(cfg[MB.HR_READING + ch])}  ` +
      `code=${s.code}  [${f.join(',')}]` +
      (fcode ? ` fault=${FAULT_NAME[fcode] ?? fcode}` : '') +
      ((lock >> ch) & 1 ? '  LOCKED' : ''));
  }
}

async function calibrateChannel(mb, ch) {
  console.log(`\n=== Calibrating CH${ch} ===`);
  const points = [];
  for (;;) {
    console.log(`\nSource a known current into CH${ch} (+ / GND_ISO) from the calibrator.`);
    const ans = await ask(`Enter applied current in mA (or "done" to finish, need >= 2 points): `);
    if (ans.trim().toLowerCase() === 'done') {
      if (points.length >= 2) break;
      console.log('Need at least 2 points.'); continue;
    }
    const iTrue = parseFloat(ans.replace(',', '.'));
    if (!Number.isFinite(iTrue)) { console.log('Invalid number.'); continue; }
    process.stdout.write('Measuring raw current ');
    const { raw, fault } = await readRawAveraged(mb, ch);
    console.log(`-> I_raw = ${raw.toFixed(5)} mA${fault ? '  (WARNING: channel fault!)' : ''}`);
    if (!Number.isFinite(raw)) { console.log('No reading; skipped.'); continue; }
    points.push([raw, iTrue]);
    console.log(`Recorded point ${points.length}: I_raw=${raw.toFixed(5)} -> I_true=${iTrue}`);
  }

  const { gain, offset, maxErr } = linfit(points);
  console.log(`\nFit: gain=${gain.toFixed(7)}  offset=${offset.toFixed(5)} mA  max residual=${maxErr.toFixed(5)} mA`);

  const calBase = MB.HR_CAL_BASE + ch * MB.HR_CAL_STRIDE;
  await mb.writeMultiple(calBase, [...floatToRegs(gain), ...floatToRegs(offset)]);
  console.log(`Wrote coefficients (live preview) to holding regs ${calBase}..${calBase + 3}.`);

  const ans = await ask(`\nCOMMIT CH${ch} to write-once Flash? This is IRREVERSIBLE. Type "COMMIT" to proceed: `);
  if (ans.trim() === 'COMMIT') {
    await mb.writeSingle(MB.HR_CAL_COMMIT, MB.CAL_COMMIT_BASE | ch);
    console.log('Committed and locked.');
  } else {
    console.log('Not committed (preview stays active until reboot).');
  }
}

async function cmdCalibrate(mb, args) {
  const channels = args.all ? [...Array(CHANNELS).keys()] : [parseInt(args.ch, 10)];
  if (channels.some((c) => !(c >= 0 && c < CHANNELS))) throw new Error('Specify --ch 0..7 or --all');
  for (const ch of channels) await calibrateChannel(mb, ch);
}

async function cmdSet(mb, args) {
  const ch = parseInt(args.ch, 10);
  if (!(ch >= 0 && ch < CHANNELS)) throw new Error('Specify --ch 0..7');
  // Thresholds are cross-validated by the firmware (lo < hi): write hi first
  // when the span moves up, lo first when it moves down.
  if (args.lo !== undefined || args.hi !== undefined) {
    const cur = await mb.readHolding(MB.HR_SCALE_LO + ch, 1);
    const curHi = (await mb.readHolding(MB.HR_SCALE_HI + ch, 1))[0];
    const lo = args.lo !== undefined ? parseInt(args.lo, 10) : cur[0];
    const hi = args.hi !== undefined ? parseInt(args.hi, 10) : curHi;
    if (!(hi > lo)) throw new Error('--hi must be greater than --lo');
    if (hi > curHi) {          // span moves up: raise hi first so lo < hi holds
      await mb.writeSingle(MB.HR_SCALE_HI + ch, hi);
      await mb.writeSingle(MB.HR_SCALE_LO + ch, lo);
    } else {
      await mb.writeSingle(MB.HR_SCALE_LO + ch, lo);
      await mb.writeSingle(MB.HR_SCALE_HI + ch, hi);
    }
    console.log(`CH${ch} scale = ${lo}..${hi} uA`);
  }
  if (args.smooth !== undefined) await mb.writeSingle(MB.HR_SMOOTH + ch, parseInt(args.smooth, 10));
  if (args.enable !== undefined) await mb.writeSingle(MB.HR_ENABLED + ch, parseInt(args.enable, 10) ? 1 : 0);
  await mb.writeSingle(MB.HR_TRIG_SAVE, MB.TRIG_SAVE);
  console.log('Configuration saved.');
}

async function cmdRate(mb, args) {
  const rate = parseInt(args._[1], 10);
  if (!(rate >= 0 && rate <= 2)) throw new Error('rate must be 0 (20 SPS+FIR), 1 (90 SPS) or 2 (330 SPS)');
  await mb.writeSingle(MB.HR_ADC_RATE, rate);
  await mb.writeSingle(MB.HR_TRIG_SAVE, MB.TRIG_SAVE);
  console.log(`ADC rate = ${rate}, saved.`);
}

/* ------------------------------- main ----------------------------------- */
async function main() {
  const args = parseArgs(process.argv.slice(2));
  const cmd = args._[0] || 'status';
  const ip = args.ip || '192.168.1.13';
  const port = parseInt(args.port || '502', 10);
  const unit = parseInt(args.unit || '1', 10);

  const mb = new ModbusTCP(ip, port, unit);
  console.log(`Connecting to ${ip}:${port} (unit ${unit}) ...`);
  await mb.connect();

  try {
    if (cmd === 'status') await cmdStatus(mb);
    else if (cmd === 'calibrate') await cmdCalibrate(mb, args);
    else if (cmd === 'set') await cmdSet(mb, args);
    else if (cmd === 'rate') await cmdRate(mb, args);
    else console.log('Unknown command. Use: status | calibrate | set | rate');
  } finally {
    mb.close();
    rl.close();
  }
}

main().catch((e) => { console.error('Error:', e.message); process.exit(1); });
