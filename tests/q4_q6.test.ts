import assert from 'node:assert/strict';
import { LineFollowModel } from '../src/sim/line_follow';
import { VehicleSimulator } from '../src/sim/physics';
import { RoutePlannerModel } from '../src/sim/route';
import { grayClassify } from '../src/sim/gray_sensor';
import { GRAY_FORWARD_OFFSET_CM } from '../src/sim/config';
import { parseRouteTableC, DEFAULT_ROUTE_TABLE_C } from '../src/sim/route_parser';
import { LineState, RouteState, Action, RouteStep } from '../src/sim/types';

// Mirror the Q4 entry sequence: three-channel drift remains suppressed, but a
// fourth/fifth channel on the actual new branch must break the narrow latch.
for (const mirror of [false, true]) {
  const flip = (m: number) => {
    let result = 0;
    for (let i = 0; i < 8; ++i) if (m & (1 << i)) result |= 1 << (7 - i);
    return result;
  };
  for (const wide of [0x0f, 0x1f]) {
    const steps: RouteStep[] = [
      { junction_count: 2, turn_speed: 16, action: Action.RIGHT, cruise_speed: 60,
        angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 0 },
      { junction_count: 1, turn_speed: 16, action: Action.STOP, cruise_speed: 60,
        angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 0 },
    ];
    const r = new RoutePlannerModel(); r.init(steps); r.start(0, 0);
    let now = 0, distance = 0;
    const sample = (m: number, n = 1) => {
      for (let i = 0; i < n; i++) {
        now += 5; distance += 0.06;
        r.update(grayClassify(mirror ? flip(m) : m, true, 0), distance, 0, now);
      }
    };
    sample(0x18, 3); sample(0x1c); sample(0x0c); sample(0x0e, 3);
    assert.equal(r.junctions_seen, 0);
    sample(wide, 3);
    assert.equal(r.junctions_seen, 1, 'wide real branch must break three-channel smooth latch');
    assert.equal(r.state, RouteState.ROUTE_CLEARING);
    assert.equal(r.passing_junction, true);
    assert.equal(r.index, 0);
  }
}
console.log('PASS: mirrored four/five-channel Q4 branches override narrow drift suppression');

// A real clearance timeout enters RECOVERING on this frame. The borrowed axis
// must be cleared after that transition, not only on the following sample.
{
  const f = new LineFollowModel([
    { junction_count: 1, turn_speed: 16, action: Action.RIGHT, cruise_speed: 60,
      angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 0 },
    { junction_count: 1, turn_speed: 16, action: Action.STOP, cruise_speed: 60,
      angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 0 },
  ]);
  f.state = LineState.LINE_JUNCTION;
  f.route.state = RouteState.ROUTE_CLEARING;
  f.route.phase_ms = 0;
  f.route.clear_cm = 0;
  f.route.entry_heading = 0;
  f.exit_heading_pending = true;
  f.exit_heading_rad = 0.1;
  f.update(grayClassify(0, true, 0), { left: 0, right: 0 }, 2000, 0.005, false, true, 0);
  assert.equal(f.route.state, RouteState.ROUTE_RECOVERING);
  assert.equal(f.exit_heading_pending, false);
}
console.log('PASS: clearance timeout clears the borrowed exit axis on its entry frame');

// This is the complete physical approach through the screenshot, not only the
// first two turns. Decisions are checked against sensor/axle geometry, and the
// target count may never be achieved by skipping Q4 and counting x=275.
const parsed = parseRouteTableC(DEFAULT_ROUTE_TABLE_C);
for (const schemeId of [1, 2, 3]) {
  const scheme = parsed.schemes.find(s => s.id === schemeId)!;
  const f = new LineFollowModel(scheme.steps);
  const sim = new VehicleSimulator(f);
  sim.requestStart();
  let q4 = false, q6 = false, beforeCount = 0;
  let corridorSamples = 0;
  for (let i = 0; i < 6000 && f.route.index < 5 && f.state !== LineState.LINE_STOPPED; ++i) {
    const oldIndex = f.route.index;
    beforeCount = f.route.junctions_seen;
    sim.step();
    if (f.route.index === 3) {
      if (beforeCount === 0 && f.route.junctions_seen === 1) {
        assert.ok(sim.pose.x > 65 && sim.pose.x < 85,
          `scheme ${schemeId} first intermediate node must be Q4: axle x=${sim.pose.x}`);
        assert.equal(f.route.passing_junction, true);
        q4 = true;
      }
      if (beforeCount === 1 && f.route.junctions_seen === 2) {
        assert.ok(q4);
        assert.ok(sim.pose.x > 215 && sim.pose.x < 230,
          `scheme ${schemeId} target must be Q6: axle x=${sim.pose.x}`);
        assert.equal(f.route.passing_junction, false);
        q6 = true;
      }
      if (q4 && !q6 && sim.pose.x > 100 && sim.pose.x < 210) {
        const sensorY = sim.pose.y + GRAY_FORWARD_OFFSET_CM * Math.sin(sim.pose.heading);
        assert.ok(Math.abs(sensorY - 120) < 1.6,
          `scheme ${schemeId} front sensor must remain on y=120 after Q4: y=${sensorY}`);
        corridorSamples++;
      }
    }
    if (oldIndex === 3 && f.route.index === 4) {
      assert.ok(q4 && q6);
      assert.ok(sim.pose.x > 228 && sim.pose.x < 255);
      assert.ok(schemeId === 2 ? sim.pose.y < 110 : sim.pose.y > 128, `scheme ${schemeId} Q6 exit y=${sim.pose.y}`);
    }
    assert.equal(f.route.navigation_recoveries, 0, `scheme ${schemeId}: no hidden recovery allowed`);
  }
  assert.ok(q4 && q6 && corridorSamples > 20);
  assert.ok(f.route.index >= 5, `scheme ${schemeId} did not finish Q6 and following turn`);
  assert.notEqual(f.state, LineState.LINE_STOPPED);
  console.log(`PASS: scheme ${schemeId} counts Q4 then turns at Q6; reaches index ${f.route.index}`);
}
// All four bottom nodes must be observed under inertia. No distance fallback
// may replace a missed count; allow the axle to follow the corner line width.
{
  const scheme = parsed.schemes.find(s => s.id === 1)!;
  const f = new LineFollowModel(scheme.steps);
  const sim = new VehicleSimulator(f);
  sim.requestStart();
  const targetIndex = 8;
  let fourth = false;
  for (let i = 0; i < 9000 && f.route.index < targetIndex && f.state !== LineState.LINE_STOPPED; ++i) {
    const oldIndex = f.route.index;
    sim.step();
    if (f.route.index === 7 && f.route.state === RouteState.ROUTE_EXECUTING &&
        sim.pose.x > 345 && sim.pose.x < 356) {
      assert.equal(f.route.junctions_seen, 4, 'turn must be authorized by four real counts');
      fourth = true;
    }
    assert.equal(f.route.navigation_recoveries, 0, 'step7 must not lose the line before the turn');
    if (oldIndex === 7 && f.route.index === targetIndex) break;
  }
  assert.ok(fourth, 'step7 did not start the right-border turn');
  assert.equal(f.route.index, targetIndex, `step7 did not clear the bottom-right turn: ${f.route.index}`);
  assert.ok(sim.pose.x < 376, `step7 ran past the boundary: ${sim.pose.x}`);
}
console.log('PASS: scheme 1 observes all four bottom nodes and turns with inertia without distance authorization');
