/**
 * Printed-transistor artwork regression (simulator only).
 *
 * The printed transistor symbol is a V (two arms leaving the through wire and
 * meeting on the base bar) plus a base bar perpendicular to the branch lead.
 * The user requirement is:
 *
 *   1. the two V ends must JOIN the through wire - no black sliver between the
 *      round stroke cap of an arm and the wire it sits on;
 *   2. the symbol stroke must be as thick as the track wire
 *      (TRANSISTOR_STROKE_CM === TRACK_LINE_WIDTH_CM);
 *   3. the base bar ("one horizontal stroke on a straight run") must cover ALL
 *      eight phototransistor channels (pitch 1.1 cm, offsets (i - 3.5) * 1.1 =
 *      +/-0.55 .. +/-3.85 cm), i.e. raw sensor mask 0xFF when the sensor bar
 *      centre crosses the bar along the branch direction.
 *
 * The plane is the same one sampled by the grayscale sensor
 * (map_data.isWhiteAt), so these assertions are exactly what the car "sees".
 */
import assert from 'node:assert/strict';
import {
  CIRCUIT_COMPONENTS,
  TRACK_SEGMENTS,
  TRACK_LINE_WIDTH_CM,
  TRANSISTOR_MOUTH_CM,
  TRANSISTOR_DEPTH_CM,
  TRANSISTOR_BAR_HALF_CM,
  TRANSISTOR_STROKE_CM,
  HEXAGON_VERTICES,
  RESISTOR_LENGTH_CM,
  RESISTOR_STROKE_CM,
  isWhiteAt,
  type TrackComponent,
} from '../src/track/map_data';
import { VehicleSimulator } from '../src/sim/physics';
import { LineFollowModel } from '../src/sim/line_follow';
import { GRAY_FORWARD_OFFSET_CM } from '../src/sim/config';

const STEP_CM = 0.1;
const CHANNEL_LIMIT_CM = 3.85; // outermost channel, (7 - 3.5) * 1.1 (pitch 1.1 cm in physics.ts)

const hex = (m: number) => '0x' + m.toString(16).toUpperCase().padStart(2, '0');

/** local (u along wire, v across / branch positive) -> world cm */
function fromLocal(comp: TrackComponent, u: number, v: number): { x: number; y: number } {
  const c = Math.cos(comp.rotation);
  const s = Math.sin(comp.rotation);
  return { x: comp.x + u * c - v * s, y: comp.y + u * s + v * c };
}

/** isWhiteAt() for a local point of the component frame */
function whiteLocal(comp: TrackComponent, u: number, v: number): boolean {
  const p = fromLocal(comp, u, v);
  return isWhiteAt(p.x, p.y);
}

function worldToLocal(comp: TrackComponent, x: number, y: number): { u: number; v: number } {
  const dx = x - comp.x;
  const dy = y - comp.y;
  const c = Math.cos(comp.rotation);
  const s = Math.sin(comp.rotation);
  return { u: dx * c + dy * s, v: -dx * s + dy * c };
}

const TRANSISTORS = CIRCUIT_COMPONENTS.filter((c) => c.type === 'transistor');

/* ------------------------------------------------------------ constants -- */

// Same stroke as the wire, and a mouth wide enough that the V ends land on the
// wire while the base bar spans the whole sensor bar.
{
  assert.equal(TRANSISTOR_STROKE_CM, TRACK_LINE_WIDTH_CM,
    `transistor stroke ${TRANSISTOR_STROKE_CM} cm must equal the track line width ${TRACK_LINE_WIDTH_CM} cm`);
  assert.equal(TRANSISTORS.length, 7, `expected the seven printed transistors, found ${TRANSISTORS.length}`);
  // The round cap of the V arms / base bar must end exactly on the footprint
  // edge (bar half = mouth half); otherwise the board patch either leaves a
  // black sliver (footprint wider) or paints white outside it (bar longer).
  assert.ok(TRANSISTOR_BAR_HALF_CM >= TRANSISTOR_MOUTH_CM / 2 - 1e-9,
    `base bar half ${TRANSISTOR_BAR_HALF_CM} cm must reach the V mouth half ${TRANSISTOR_MOUTH_CM / 2} cm`);
  assert.ok(TRANSISTOR_BAR_HALF_CM + TRANSISTOR_STROKE_CM / 2 >= CHANNEL_LIMIT_CM,
    `painted base bar ${TRANSISTOR_BAR_HALF_CM + TRANSISTOR_STROKE_CM / 2} cm must cover the outermost channel ${CHANNEL_LIMIT_CM} cm`);
  assert.ok(TRANSISTOR_MOUTH_CM / 2 + TRANSISTOR_STROKE_CM / 2 >= TRANSISTOR_BAR_HALF_CM + TRANSISTOR_STROKE_CM / 2 - 1e-9,
    'footprint u half must be max(mouth half, bar half) + stroke/2');
  console.log(
    `PASS: transistor stroke=${TRANSISTOR_STROKE_CM} cm = wire width, mouth=${TRANSISTOR_MOUTH_CM} cm, ` +
    `depth=${TRANSISTOR_DEPTH_CM} cm, bar half=${TRANSISTOR_BAR_HALF_CM} cm`
  );
}

/* ------------------------------------ V ends / base bar join the wires -- */

// Requirement (a): walk the through-wire axis outward from each V arm tip and
// the branch axis outward from the base bar.  Every sample must be white: the
// old footprint (mouth/2 + full stroke) left a black gap of half a line width
// between the arm cap and the wire, which is exactly the "V does not touch the
// line" artefact the user reported.
for (const comp of TRANSISTORS) {
  const m = TRANSISTOR_MOUTH_CM / 2;
  const d = TRANSISTOR_DEPTH_CM;
  const s = comp.stroke ?? TRANSISTOR_STROKE_CM;
  let samples = 0;

  // both V arm tips -> through wire on both sides
  for (const side of [-1, 1]) {
    for (let k = 0; k * STEP_CM <= 3.5 + 1e-9; ++k) {
      const u = side * (m + k * STEP_CM);
      assert.ok(whiteLocal(comp, u, 0),
        `${comp.id}: black gap on the through wire at u=${u.toFixed(2)} cm (V arm at ${side < 0 ? '-' : '+'}${m} cm must join the wire)`);
      samples++;
    }
  }

  // the arms themselves and their meeting point on the base bar
  for (const arm of [0, 1]) {
    for (let k = 0; k <= Math.round(1 / STEP_CM); ++k) {
      const t = k * STEP_CM;
      const u = arm === 0 ? -m + t * m : t * m;
      const v = arm === 0 ? t * d : (1 - t) * d;
      assert.ok(whiteLocal(comp, u, v), `${comp.id}: V arm ${arm} is not white at t=${t.toFixed(2)}`);
      samples++;
    }
  }

  // base bar -> branch wire: bar centred at v = depth, stroke reaches v = d + s/2
  const vJoin = d + s / 2;
  assert.ok(Math.abs(vJoin - (d + TRANSISTOR_STROKE_CM / 2)) < 1e-9, `${comp.id}: expected the base-bar cap at ${vJoin} cm`);
  for (let k = 0; k * STEP_CM <= 3.5 + 1e-9; ++k) {
    const v = d + k * STEP_CM;
    assert.ok(whiteLocal(comp, 0, v),
      `${comp.id}: black gap between the base bar and the branch wire at v=${v.toFixed(2)} cm`);
    samples++;
  }

  // the base bar must be drawn across the full mouth (no clipped ends)
  for (const side of [-1, 1]) {
    for (let k = 0; k * STEP_CM <= TRANSISTOR_BAR_HALF_CM; ++k) {
      const u = side * Math.min(k * STEP_CM, TRANSISTOR_BAR_HALF_CM);
      assert.ok(whiteLocal(comp, u, d), `${comp.id}: base bar is not white at u=${u.toFixed(2)} cm`);
      samples++;
    }
  }

  console.log(`PASS: ${comp.id} (${comp.x},${comp.y}) V ends and base bar join the wires (${samples} samples)`);
}

/* ------------------------------------- base bar covers all 8 channels -- */

// Requirement (b): driving along the branch, with the sensor row centre on the
// base bar, every channel must read white (raw mask 0xFF).  The simulator is
// the real sampler (isWhiteAt + the 24 cm look-ahead bar), not a hand model.
for (const comp of TRANSISTORS) {
  // Branch direction in world coordinates: rotate canonical +v by `rotation`.
  const bx = comp.x - Math.sin(comp.rotation) * TRANSISTOR_DEPTH_CM;
  const by = comp.y + Math.cos(comp.rotation) * TRANSISTOR_DEPTH_CM;
  const heading = comp.rotation + Math.PI / 2; // forward = branch direction

  const sim = new VehicleSimulator(new LineFollowModel());
  sim.reset({
    x: bx - GRAY_FORWARD_OFFSET_CM * Math.cos(heading),
    y: by - GRAY_FORWARD_OFFSET_CM * Math.sin(heading),
    heading,
  });
  const mask = sim.updateSensors();
  assert.equal(mask, 0xff, `${comp.id}: base-bar raw mask must be 0xFF, got ${hex(mask)}`);
  assert.equal(sim.sensor_points.length, 8, `${comp.id}: expected the 8 sensor channels`);

  for (const pt of sim.sensor_points) {
    const { u, v } = worldToLocal(comp, pt.x, pt.y);
    assert.ok(Math.abs(v - TRANSISTOR_DEPTH_CM) < 1e-6,
      `${comp.id}: channel ${pt.index} is ${Math.abs(v - TRANSISTOR_DEPTH_CM).toFixed(3)} cm off the base bar (v=${v.toFixed(3)})`);
    assert.ok(Math.abs(u) <= CHANNEL_LIMIT_CM + 1e-9,
      `${comp.id}: channel ${pt.index} u=${u.toFixed(3)} exceeds the outermost channel ${CHANNEL_LIMIT_CM} cm`);
  }
  console.log(`PASS: ${comp.id} base bar covers all 8 channels (raw mask ${hex(mask)})`);
}

/* ------------------------------------------------- V arm / bar thickness -- */

// Requirement (d): the symbol stroke is the wire width, so a wire joint just
// outside the symbol footprint is 2.8 cm wide: +/-1.35 cm white, +/-1.45 cm
// dark.  Both the through wire and the branch wire are checked.
for (const comp of TRANSISTORS) {
  const m = TRANSISTOR_MOUTH_CM / 2;
  const d = TRANSISTOR_DEPTH_CM;
  const uOut = Math.max(m, TRANSISTOR_BAR_HALF_CM) + TRANSISTOR_STROKE_CM / 2; // footprint edge
  const uProbe = uOut + 2.0; // clear of the footprint

  for (const side of [-1, 1]) {
    for (const across of [-1.45, 1.45]) {
      assert.equal(whiteLocal(comp, side * uProbe, across), false,
        `${comp.id}: through wire is wider than ${TRACK_LINE_WIDTH_CM} cm at u=${side * uProbe}, v=${across}`);
      assert.equal(whiteLocal(comp, side * uProbe, across - Math.sign(across) * 0.1), true,
        `${comp.id}: through wire lost its ${TRACK_LINE_WIDTH_CM} cm width at u=${side * uProbe}, v=${(across - Math.sign(across) * 0.1).toFixed(2)}`);
    }
  }

  const vProbe = d + TRANSISTOR_STROKE_CM / 2 + 2.0; // branch wire clear of the footprint
  for (const across of [-1.45, 1.45]) {
    assert.equal(whiteLocal(comp, across, vProbe), false,
      `${comp.id}: branch wire is wider than ${TRACK_LINE_WIDTH_CM} cm at u=${across}, v=${vProbe}`);
    assert.equal(whiteLocal(comp, across - Math.sign(across) * 0.1, vProbe), true,
      `${comp.id}: branch wire lost its ${TRACK_LINE_WIDTH_CM} cm width at v=${vProbe}`);
  }
  console.log(`PASS: ${comp.id} both connecting wires are ${TRACK_LINE_WIDTH_CM} cm wide (+/-1.35 white, +/-1.45 dark)`);
}

/* --------------------------------------- junction left of the hexagon -- */

// Requirement (c): the junction at (245,170) must continue down to the bottom
// border (245,220).  That missing lower piece is the line left of the ring;
// x=275 must NOT be extended into the hexagon left vertex (the user marked
// that spur as wrong).
{
  const seg = TRACK_SEGMENTS.find((s) => s.id === 'v_245');
  assert.ok(seg, 'TRACK_SEGMENTS must contain the x = 245 vertical');
  assert.equal(seg!.p1.y, 0, 'x = 245 must start at the top border');
  assert.equal(seg!.p2.y, 220, 'x = 245 must reach the bottom border');
  for (let y = 170; y <= 220.0001; y += 0.1) {
    assert.ok(isWhiteAt(245, y), `x = 245 is dark at y = ${y.toFixed(2)} (missing lower link)`);
  }
  assert.ok(!TRACK_SEGMENTS.some((s) => s.id === 'v_275_to_hex'),
    'x = 275 must not be joined to the hexagon left vertex');
  for (let y = 115; y <= 160; y += 0.5) {
    assert.equal(isWhiteAt(275, y), false, `x = 275 must stay dark at y = ${y.toFixed(1)} (no spur into the ring)`);
  }
  console.log('PASS: x = 245 continues to the bottom border; x = 275 stops at y = 110 (no ring spur)');
}

/* --------------------------------------------- hexagon ring integrity -- */

// The ring edges and vertices stay intact after the segment fixes above.
{
  const edges = TRACK_SEGMENTS.filter((s) => s.id.startsWith('hex_'));
  assert.equal(edges.length, 6, 'hexagon ring must keep its six edges');
  for (const edge of edges) {
    for (let t = 0; t <= 1.0001; t += 0.1) {
      const x = edge.p1.x + (edge.p2.x - edge.p1.x) * t;
      const y = edge.p1.y + (edge.p2.y - edge.p1.y) * t;
      const isR14Body = Math.abs(x - 310) < RESISTOR_LENGTH_CM / 2 + RESISTOR_STROKE_CM / 2 &&
        y > 165 + 20;
      if (!isR14Body) assert.ok(isWhiteAt(x, y), `hexagon edge ${edge.id} is dark at (${x.toFixed(1)}, ${y.toFixed(1)})`);
    }
  }
  for (const v of HEXAGON_VERTICES) {
    assert.ok(isWhiteAt(v.x, v.y), `hexagon vertex (${v.x}, ${v.y}) must be white`);
  }
  console.log('PASS: all hexagon edges and vertices (except the printed r14 body) stay white');
}
