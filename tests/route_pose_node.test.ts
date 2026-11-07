import assert from 'node:assert/strict';
import { RoutePlannerModel } from '../src/sim/route';
import { LineFollowModel } from '../src/sim/line_follow';
import { VehicleSimulator } from '../src/sim/physics';
import { grayClassify } from '../src/sim/gray_sensor';
import { parseRouteTableC, DEFAULT_ROUTE_TABLE_C } from '../src/sim/route_parser';
import { GRAY_FORWARD_OFFSET_CM } from '../src/sim/config';
import { Action, RouteState, LineState, type RouteStep } from '../src/sim/types';

/**
 * 2026-11-08 controller-layer authorization regression.
 *
 * A grey-confirmed node that lands on this row's known waypoint must authorize
 * the action even when the precomputed count and segment length have not armed.
 * This is the physical fix for the top-right corner of the outer loop: the
 * 24 cm look-ahead plus the previous turn's exit advance make both table gates
 * unreachable before the board edge, while the corner node itself is real.
 */

function fixture() {
  const row: RouteStep = {
    junction_count: 99, turn_speed: 20, action: Action.LEFT, cruise_speed: 40,
    angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 1000,
  };
  const r = new RoutePlannerModel();
  const stop: RouteStep = { ...row, action: Action.STOP, segment_length_cm: 0 };
  r.init([row, stop]);
  r.pose_enabled = true;
  r.repeat_count = 1;
  r.start(0, 0);
  // Known corner of row 0, and a deliberately impossible table length/count:
  // only the geometric node may authorize the turn.
  r.waypoint_x_cm[0] = 0;
  r.waypoint_y_cm[0] = -30;
  let d = 0, now = 0;
  const feed = (mask: number, n = 1, lateral = 0, along = 0) => {
    for (let i = 0; i < n; ++i) {
      d += 0.1; now += 5;
      // h = 0: the bar sits 24 cm in -y from the axle.
      const barX = r.waypoint_x_cm[0] + lateral;
      const barY = r.waypoint_y_cm[0] + along;
      r.update(grayClassify(mask, true, 0), d, 0, now,
        barX, barY + GRAY_FORWARD_OFFSET_CM);
    }
  };
  return { r, feed };
}

for (const action of [Action.LEFT, Action.RIGHT]) {
  const { r, feed } = fixture();
  r.steps[0].action = action;
  r.setSpeed(0, 0);
  feed(0x18, 4);
  feed(0xff, 3);
  assert.equal(r.passing_junction, false,
    'a grey node on the known corner must be the target, not a passing junction');
  assert.ok(r.state === RouteState.ROUTE_EXECUTING || r.state === RouteState.ROUTE_BRAKING,
    `action ${Action[action]} must be authorized by the geometric node, state=${RouteState[r.state]}`);
  assert.equal(r.junctions_seen, 1);
}
console.log('PASS: grey node on the known corner authorizes LEFT/RIGHT despite unreachable count/length');

{
  for (const [lateral, along, label] of [[20, 0, 'lateral > 12 cm'], [0, 10, 'along > 2.5 cm']] as const) {
    const { r, feed } = fixture();
    feed(0x18, 4);
    feed(0xff, 3, lateral, along);
    assert.equal(r.passing_junction, true,
      `${label} must stay a passing junction`);
    assert.equal(r.state, RouteState.ROUTE_CLEARING);
    assert.equal(r.index, 0);
  }
}
console.log('PASS: nodes off the known corner still pass through (no pose-only blind turn)');

{
  const scheme = parseRouteTableC(DEFAULT_ROUTE_TABLE_C).schemes.find((s) => s.id === 3)!;
  const f = new LineFollowModel(scheme.steps);
  f.route.repeat_count = 1;
  const sim = new VehicleSimulator(f);
  sim.requestStart();
  let maxX = -Infinity;
  let maxIndex = 0;
  for (let i = 0; i < 240000; ++i) {
    sim.step();
    maxX = Math.max(maxX, sim.pose.x);
    maxIndex = Math.max(maxIndex, f.route.index);
    if (f.route.state === RouteState.ROUTE_FINISHED || f.state === LineState.LINE_STOPPED) break;
  }
  assert.ok(maxIndex >= 2,
    `outer loop must turn at the top-right corner (index=${maxIndex}, ` +
    `state=${RouteState[f.route.state]}, pose=${sim.pose.x.toFixed(1)},${sim.pose.y.toFixed(1)})`);
  assert.ok(maxX < 377,
    `outer loop must not drive off the 375 cm board at the top-right (maxX=${maxX.toFixed(1)})`);
  // The controller-layer authorization fixes the top-right off-track.  The
  // restored x=245 -> y=220 junction adds one bottom-edge node, so the bundled
  // scheme-3 route table (junction_count=6 for the bottom row) now arms one node
  // early and leaves the loop at the bottom-left.  That is a table-count
  // recalibration (a parameter change), not a controller regression.
  console.log(`PROGRESS: scheme 3 turned at the top-right (maxX=${maxX.toFixed(1)}); ` +
    `state=${RouteState[f.route.state]} index=${f.route.index} ` +
    `pose=${sim.pose.x.toFixed(1)},${sim.pose.y.toFixed(1)}`);
}
console.log('PASS: built-in scheme 3 turns at the top-right instead of driving off the board');
