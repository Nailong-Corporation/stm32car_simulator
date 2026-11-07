import assert from 'node:assert/strict';
import { RoutePlannerModel } from '../src/sim/route';
import { grayClassify } from '../src/sim/gray_sensor';
import { GRAY_FORWARD_OFFSET_CM } from '../src/sim/config';
import { Action, RouteState, type RouteStep } from '../src/sim/types';

function fixture(action = Action.LEFT, count = 4, length = 105, short = 0) {
  const row: RouteStep = { junction_count: count, turn_speed: 12, action,
    cruise_speed: 30, angle_deg: action === Action.HEXAGON ? 360 : 0, corner_arc_enabled: false,
    segment_length_cm: length, short_link_cm: short };
  const r = new RoutePlannerModel();
  const stop = { ...row, action: Action.STOP, junction_count: 1, segment_length_cm: 100, short_link_cm: 0 };
  r.init(action === Action.STOP ? [row] : short > 0
    ? [{ ...row, short_link_cm: 0 }, row, stop] : [row, stop]);
  r.repeat_count = 1; r.start(0, 0);
  if (short > 0) r.index = 1;
  assert.equal(r.valid, true);
  let d = 0, now = 0;
  // h=0 points north: place the bar on the waypoint or far from it.
  const feed = (mask: number, n = 1, near = false, advance = .1) => {
    for (let i = 0; i < n; ++i) {
      d += advance; now += 5;
      r.update(grayClassify(mask, true, 0), d, 0, now,
        r.waypoint_x_cm[r.index] + (near ? 0 : 100),
        r.waypoint_y_cm[r.index] + GRAY_FORWARD_OFFSET_CM);
    }
  };
  return { r, feed, distance: (v: number) => { d = v; } };
}

for (const action of [Action.LEFT, Action.RIGHT, Action.STOP]) {
  for (const mask of [0, 0x18]) {
    const { r, feed, distance } = fixture(action);
    distance(150); feed(mask, 4, true);
    assert.equal(r.state, RouteState.ROUTE_FOLLOW, 'ordinary pose proximity/fallback cannot authorize action');
    assert.equal(r.junctions_seen, 0);
  }
}
console.log('PASS: ordinary rows require a grey node even at waypoint or past distance fallback');

for (const countReady of [false, true]) {
  const { r, feed, distance } = fixture();
  if (countReady) r.junctions_seen = 3;
  else distance(105);
  feed(0x18, 4, true); feed(0xff, 3, true);
  assert.equal(r.state, RouteState.ROUTE_EXECUTING, 'confirmed target uses count OR distance');
  assert.equal(r.passing_junction, false);
  const far = fixture();
  far.r.junctions_seen = 3;
  far.feed(0x18, 4); far.feed(0xff, 3);
  assert.equal(far.r.passing_junction, false, 'drifted pose cannot veto a grey-led pivot');
  assert.equal(far.r.state, RouteState.ROUTE_EXECUTING);
}
console.log('PASS: grey-led pivots honor confirmed count OR distance despite pose drift');

for (const special of [Action.LEFT, Action.HEXAGON]) {
  const short = special === Action.LEFT ? 30 : 0;
  const { r, feed } = fixture(special, 1, 100, short);
  feed(0x18, 4); feed(0xff, 3);
  assert.equal(r.state, RouteState.ROUTE_CLEARING, 'special rows retain pose gate on grey confirmation');
  assert.equal(r.passing_junction, true);
  const near = fixture(special, 1, 100, short);
  near.feed(0, 1, true);
  assert.equal(near.r.state, RouteState.ROUTE_EXECUTING, 'special rows retain pose trigger without grey');
}
console.log('PASS: short-link and HEXAGON retain parallel pose trigger and grey pose gate');

for (const passing of [true, false]) {
  const { r, feed } = fixture(passing ? Action.LEFT : Action.STRAIGHT, passing ? 4 : 1);
  // A non-target node must also be geometrically away from this row's known
  // corner; otherwise the 2026-11-08 controller-layer authorization correctly
  // promotes it to the target.
  feed(0x18, 4, !passing); feed(0xff, 3, !passing);
  if (!passing) feed(0x18, 3, true);
  assert.equal(r.state, RouteState.ROUTE_CLEARING);
  assert.equal(r.passing_junction, passing);
  // Front bar, hollow body, rear bar: >2 cm of ambiguous/no-line evidence.
  feed(0, 6, false, .5); feed(0x81, 4, false, .2); feed(0xff, 4, false, .2);
  assert.equal(r.state, RouteState.ROUTE_CLEARING, 'pose distance cannot unlock a component crossing');
  assert.equal(r.index, 0);
  assert.equal(r.junctions_seen, 1, 'front and rear bars count once');
  feed(0x18, 45);
  assert.equal(r.state, RouteState.ROUTE_FOLLOW, 'real narrow exit releases the lock');
  assert.equal(r.index, passing ? 0 : 1);
}
console.log('PASS: passing and target STRAIGHT components stay locked through rear bar until narrow exit');

{
  const { r, feed } = fixture();
  r.state = RouteState.ROUTE_CLEARING;
  r.passing_junction = false;
  r.clear_cm = 0; r.phase_ms = 0;
  feed(0, 1, false, 2.1);
  assert.equal(r.index, 1, 'completed turn retains 2 cm pose clearance without grey');
  assert.equal(r.state, RouteState.ROUTE_FOLLOW);
}
console.log('PASS: completed turn retains pose clearance');

{
  const { r, feed } = fixture(Action.LEFT, 2);
  r.steps[0].corner_arc_enabled = true;
  feed(0x18, 1, true);
  assert.equal(r.state, RouteState.ROUTE_FOLLOW,
    'positive-count rolling arc rows still require a grey node');
  const arc = fixture(Action.LEFT, 0);
  arc.r.steps[0].corner_arc_enabled = true;
  arc.feed(0, 1, true);
  assert.equal(arc.r.state, RouteState.ROUTE_EXECUTING,
    'count-zero rolling arc keeps the geometric entry');
}
console.log('PASS: geometric entry only for count-zero rolling arcs; positive-count rows stay grey-led');
