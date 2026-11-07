/**
 * Printed-resistor artwork regression (simulator only).
 *
 * The printed resistor symbol is a HOLLOW rectangle stroked on top of the wire.
 * Its two long edges run along the wire at v = +/- W/2 (stroke centred on that
 * line, so the white band is |v| in [W/2 - s/2, W/2 + s/2]).  When the sensor
 * bar is centred on the wire at the middle of the symbol, the eight channels
 * sit at v = [-3.85, -2.75, -1.65, -0.55, 0.55, 1.65, 2.75, 3.85] cm, so the
 * requirement is:
 *
 *   S0/S1 and S6/S7  -> white (outermost pairs on both long edges)
 *   S2/S3/S4/S5      -> dark  (board shows through the middle)
 *   raw mask         -> 0xC3  (11000011)
 *
 * That is also the "parallel pair" signature grayClassify() already knows, so
 * the car tracks the symbol centre instead of treating it as a junction.
 *
 * The artwork is drawn at the official-proportion scale (length 14.5 cm =
 * 8.0 cm * 1.8125); the resistor constants must stay in the same proportion
 * (length : width : stroke = 8.0 : 3.8 : 1.3).  This is a simulator picture
 * calibration, never a firmware parameter.
 */
import assert from 'node:assert/strict';
import {
  CIRCUIT_COMPONENTS,
  RESISTOR_LENGTH_CM,
  RESISTOR_WIDTH_CM,
  RESISTOR_STROKE_CM,
} from '../src/track/map_data';
import { VehicleSimulator } from '../src/sim/physics';
import { LineFollowModel } from '../src/sim/line_follow';
import { grayClassify, grayIsParallelPair } from '../src/sim/gray_sensor';
import { GRAY_FORWARD_OFFSET_CM } from '../src/sim/config';
import { GrayKind } from '../src/sim/types';

const hex = (m: number) => '0x' + m.toString(16).toUpperCase().padStart(2, '0');
const PITCH_CM = 1.1;              // physics.ts VehicleSimulator.sensor_pitch_cm
const CHANNEL_OFFSET = (i: number) => (i - 3.5) * PITCH_CM;
const OUTER_PAIR_MASK = 0x03 | 0xc0; // S0,S1 + S6,S7

/* ---------------------------------------------------------------- artwork -- */

// The three constants must be one single isotropic scale of the artwork
// proportions; a different k on one axis would tilt the long edges away from
// the outermost channels.
{
  const k = RESISTOR_LENGTH_CM / 8.0;
  assert.ok(Math.abs(RESISTOR_LENGTH_CM - 14.5) < 1e-9, `resistor length ${RESISTOR_LENGTH_CM} cm must be the 14.5 cm artwork length`);
  assert.ok(Math.abs(RESISTOR_WIDTH_CM - 3.8 * k) < 1e-9, 'resistor width must use the same scale as the length');
  assert.ok(Math.abs(RESISTOR_STROKE_CM - 1.3 * k) < 1e-9, 'resistor stroke must use the same scale as the length');
  assert.ok(k > 1.7 && k < 1.9, `artwork scale ${k} must stay inside the calibrated 1.75..1.85 band`);
  console.log(`PASS: resistor artwork keeps one isotropic k=${k} (L=${RESISTOR_LENGTH_CM}, W=${RESISTOR_WIDTH_CM}, s=${RESISTOR_STROKE_CM})`);
}

// The long-edge white band must sit exactly on the outer two channels a side.
{
  const inner = RESISTOR_WIDTH_CM / 2 - RESISTOR_STROKE_CM / 2;
  const outer = RESISTOR_WIDTH_CM / 2 + RESISTOR_STROKE_CM / 2;
  assert.ok(inner <= 2.75 && 2.75 <= outer, `S1/S6 channel 2.75 cm must hit the long edge, band is [${inner}, ${outer}]`);
  assert.ok(inner <= 3.85 && 3.85 <= outer, `S0/S7 channel 3.85 cm must hit the long edge, band is [${inner}, ${outer}]`);
  assert.ok(Math.abs(1.65) < inner, `S2/S5 channel 1.65 cm must stay inside the hollow body, band starts at ${inner}`);
  assert.ok(Math.abs(0.55) < inner, `S3/S4 channel 0.55 cm must stay inside the hollow body, band starts at ${inner}`);
  // The footprint (board) must reach the outer edge of the stroke, otherwise the
  // rendering paints white pixels the sensor cannot see.
  const footprintHalfAcross = RESISTOR_WIDTH_CM / 2 + RESISTOR_STROKE_CM / 2;
  assert.ok(footprintHalfAcross >= 3.85, 'resistor footprint must contain the outermost channel');
  console.log(`PASS: long edges span |v| in [${inner.toFixed(4)}, ${outer.toFixed(4)}] cm, straddling S0/S1 and S6/S7 while S2..S5 stay dark`);
}

// Proportional tolerance: the design must keep the same raw mask for any
// uniform artwork scale in 1.75..1.85.  Scaling the artwork by k0/k is
// geometrically identical to scaling the sensor offsets by k/k0, so the sweep
// re-runs the real sensor sampler with a scaled channel pitch.  Every channel
// stays inside the symbol footprint for the whole band (max reach 3.99 cm <
// 4.62 cm), so the wire underneath never enters the comparison.
{
  const k0 = RESISTOR_LENGTH_CM / 8.0;
  const sim = new VehicleSimulator(new LineFollowModel());
  const r = CIRCUIT_COMPONENTS.find((c) => c.id === 'r2')!;
  const h = r.rotation;
  const ux = Math.cos(h);
  const uy = Math.sin(h);
  for (let k = 1.75; k <= 1.8500001; k += 0.005) {
    sim.sensor_pitch_cm = (PITCH_CM * k0) / k;
    sim.reset({ x: r.x - GRAY_FORWARD_OFFSET_CM * ux, y: r.y - GRAY_FORWARD_OFFSET_CM * uy, heading: h });
    assert.equal(hex(sim.updateSensors()), '0xC3', `raw mask must stay 0xC3 for artwork scale k=${k.toFixed(3)}`);
  }
  sim.sensor_pitch_cm = PITCH_CM;
  console.log('PASS: raw mask stays 0xC3 across artwork scale k in [1.75, 1.85]');
}

/* ------------------------------------------------------- sensor sampling -- */

/** Raw sensor mask with the bar centre placed at comp + du (along wire) + dv (across wire). */
function rawMaskAt(comp: (typeof CIRCUIT_COMPONENTS)[number], du = 0, dv = 0): number {
  const h = comp.rotation;
  const ux = Math.cos(h);
  const uy = Math.sin(h);
  const lx = -uy;
  const ly = ux;
  const cx = comp.x + du * ux + dv * lx;
  const cy = comp.y + du * uy + dv * ly;
  const sim = new VehicleSimulator(new LineFollowModel());
  sim.reset({ x: cx - GRAY_FORWARD_OFFSET_CM * ux, y: cy - GRAY_FORWARD_OFFSET_CM * uy, heading: h });
  return sim.updateSensors();
}

// rotation 0 (wire along +x) and rotation PI/2 (wire along +y): both must show
// the same two long edges.
for (const id of ['r2', 'r1']) {
  const comp = CIRCUIT_COMPONENTS.find((c) => c.id === id)!;
  assert.ok(comp.rotation === 0 || Math.abs(comp.rotation - Math.PI / 2) < 1e-9, `${id} fixture must be axis aligned`);
  assert.equal(hex(rawMaskAt(comp)), '0xC3', `${id} (rotation ${comp.rotation.toFixed(2)}) must show the two long edges, got ${hex(rawMaskAt(comp))}`);
}
console.log('PASS: rotation 0 and rotation PI/2 resistors show raw mask 0xC3 at the symbol centre');

// Every printed resistor on this track, not just the two fixtures.
{
  const resistors = CIRCUIT_COMPONENTS.filter((c) => c.type === 'resistor');
  assert.ok(resistors.length >= 19, `expected the 19 printed resistors, found ${resistors.length}`);
  for (const r of resistors) {
    assert.equal(hex(rawMaskAt(r)), '0xC3', `${r.id} at (${r.x}, ${r.y}) must show 0xC3 at its centre`);
  }
  console.log(`PASS: all ${resistors.length} printed resistors show raw mask 0xC3 at their centre`);
}

// A small lateral misalignment must not drop an outermost channel: the car is
// allowed to be 0.4 cm off the wire centre line when it reaches the symbol.
for (const id of ['r2', 'r1']) {
  const comp = CIRCUIT_COMPONENTS.find((c) => c.id === id)!;
  for (const sign of [-1, 1]) {
    for (const along of [0, 0.4]) {
      const mask = rawMaskAt(comp, (along * sign), (0.4 * sign));
      assert.equal(mask & OUTER_PAIR_MASK, OUTER_PAIR_MASK,
        `${id} offset ${sign * 0.4} cm across / ${along} cm along lost an outermost channel: ${hex(mask)}`);
    }
  }
  // world-x lateral offset of the whole bar (the requirement is stated in x)
  for (const dx of [-0.4, 0.4]) {
    const sim = new VehicleSimulator(new LineFollowModel());
    const h = comp.rotation;
    sim.reset({
      x: comp.x + dx - GRAY_FORWARD_OFFSET_CM * Math.cos(h),
      y: comp.y - GRAY_FORWARD_OFFSET_CM * Math.sin(h),
      heading: h,
    });
    assert.equal(sim.updateSensors() & OUTER_PAIR_MASK, OUTER_PAIR_MASK,
      `${id} with the bar centre shifted ${dx} cm in x lost an outermost channel`);
  }
}
console.log('PASS: +/-0.4 cm lateral offset still triggers the outermost pairs S0/S1 and S6/S7');

/* ------------------------------------------------------------- end caps -- */

// The two end caps (u = +/- L/2) cross the wire: they must read as ONE wide
// white band, never as a parallel pair (that would fake a second hollow body).
for (const id of ['r2', 'r1']) {
  const comp = CIRCUIT_COMPONENTS.find((c) => c.id === id)!;
  for (const sign of [-1, 1]) {
    const mask = rawMaskAt(comp, (sign * RESISTOR_LENGTH_CM) / 2);
    const obs = grayClassify(mask, true, 0);
    assert.equal(obs.region_count, 1, `${id} cap at u=${sign} * L/2 must be one region, got ${hex(mask)}`);
    assert.equal(grayIsParallelPair(obs), false, `${id} cap at u=${sign} * L/2 must not look like a parallel pair`);
    assert.equal(obs.kind, GrayKind.GRAY_AMBIGUOUS, `${id} cap at u=${sign} * L/2 is a wide band, not a trackable line`);
    assert.ok(obs.white_count >= 5, `${id} cap must be a wide band, got white_count=${obs.white_count}`);
  }
}
console.log('PASS: both end caps read as one wide (non-parallel) band');

/* ---------------------------------------------------- gray classification -- */

// 0xC3 is the documented parallel-pair signature: two regions, centred, with the
// middle channels empty, so the controller follows the wire straight through.
{
  const obs = grayClassify(0xC3, true, 0);
  assert.equal(obs.kind, GrayKind.GRAY_VALID);
  assert.equal(obs.region_count, 2);
  assert.ok(Math.abs(obs.position) < 1e-9, `parallel pair must be centred, position=${obs.position}`);
  assert.equal(obs.white_count, 4);
  assert.equal(grayIsParallelPair(obs), true);
  assert.equal(obs.selected_mask, 0xC3);
  console.log('PASS: grayClassify(0xC3) is a centred 2-region parallel pair (GRAY_VALID)');
}
