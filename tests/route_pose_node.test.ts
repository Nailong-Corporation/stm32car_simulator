import assert from 'node:assert/strict';
import { RoutePlannerModel } from '../src/sim/route';
import { grayClassify } from '../src/sim/gray_sensor';
import { GRAY_FORWARD_OFFSET_CM } from '../src/sim/config';
import { Action, RouteState, type RouteStep } from '../src/sim/types';

// Pose may guide steering, but FOLLOW/CANDIDATE authorization requires a grey
// node after the positive segment distance, or the count for a zero-length row.
function fixture(action: Action, pose: boolean, arc = false, count = 4, length = 105) {
  const row: RouteStep = {
    junction_count: count, turn_speed: 12, action, cruise_speed: 30,
    angle_deg: action === Action.HEXAGON ? 360 : action === Action.CUSTOM_TURN ? 62 : 0,
    corner_arc_enabled: arc, segment_length_cm: length,
  };
  const r = new RoutePlannerModel();
  r.init(action === Action.STOP ? [row] : [row, { ...row, action: Action.STOP,
    angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 100 }]);
  r.pose_enabled = pose; r.repeat_count = 1; r.start(0, 0); r.setSpeed(0, 0);
  assert.equal(r.valid, true);
  let d = 0, now = 0;
  const feed = (mask: number, n = 1, near = true, advance = .1) => {
    for (let i = 0; i < n; ++i) {
      d += advance; now += 5;
      r.update(grayClassify(mask, true, 0), d, 0, now,
        r.waypoint_x_cm[0] + (near ? 0 : 100),
        r.waypoint_y_cm[0] + GRAY_FORWARD_OFFSET_CM);
    }
  };
  return { r, feed, distance: (v: number) => { d = v; } };
}

const actions = [Action.LEFT, Action.RIGHT, Action.CUSTOM_TURN, Action.HEXAGON,
  Action.STRAIGHT, Action.STOP];
for (const pose of [false, true]) for (const action of actions) {
  for (const mask of [0, 0x18]) {
    const { r, feed, distance } = fixture(action, pose);
    r.junctions_seen = 99; distance(150); feed(mask, 5);
    assert.equal(r.state, RouteState.ROUTE_FOLLOW, 'waypoint/fallback/count cannot replace a grey node');
    assert.equal(r.junctions_seen, 99, 'pose cannot manufacture a count');
  }
  {
    const { r, feed } = fixture(action, pose);
    r.junctions_seen = 99; feed(0x18, 4); feed(0xff, 3);
    assert.equal(r.state, RouteState.ROUTE_CLEARING, 'even a node on the waypoint must wait for distance');
    assert.equal(r.passing_junction, true);
    assert.equal(r.index, 0);
  }
  for (const near of [true, false]) {
    const { r, feed, distance } = fixture(action, pose);
    distance(105); feed(0x18, 4, near); feed(0xff, 3, near);
    assert.equal(r.passing_junction, false, 'drifted waypoint must not veto the grey target');
    assert.equal(r.state, action === Action.STOP ? RouteState.ROUTE_FINISHED : RouteState.ROUTE_EXECUTING);
    assert.equal(r.junctions_seen, 1, 'distance authorizes despite insufficient count');
  }
}
console.log('PASS: all actions in pose and grey modes require distance then grey proof, without waypoint trigger or veto');

for (const count of [0, 2]) {
  const { r, feed, distance } = fixture(Action.RIGHT, true, true, count);
  distance(150); feed(0, 4);
  assert.equal(r.state, RouteState.ROUTE_FOLLOW, 'rolling arcs also require grey, including count zero');
  feed(0x18, 4); feed(0xff, 3);
  assert.equal(r.state, RouteState.ROUTE_EXECUTING);
  assert.equal(r.passing_junction, false);
}
console.log('PASS: rolling arc entry requires a confirmed grey node even for count zero');

{
  const { r, feed, distance } = fixture(Action.RIGHT, true);
  feed(0x18, 4); feed(0xff, 1);
  assert.equal(r.state, RouteState.ROUTE_CANDIDATE);
  distance(105); feed(0, 1);
  assert.equal(r.state, RouteState.ROUTE_FOLLOW, 'distance cannot promote an interrupted candidate');
  assert.equal(r.junctions_seen, 0);
  feed(0x18, 4); feed(0xff, 3, false);
  assert.equal(r.state, RouteState.ROUTE_EXECUTING);
}
for (const pose of [false, true]) {
  const { r, feed } = fixture(Action.RIGHT, pose, false, 2, 0);
  feed(0x18, 4); feed(0xff, 3);
  assert.equal(r.state, RouteState.ROUTE_CLEARING);
  assert.equal(r.passing_junction, true);
  feed(0x18, 40); feed(0xff, 3);
  assert.equal(r.state, RouteState.ROUTE_EXECUTING);
  assert.equal(r.junctions_seen, 2);
}
console.log('PASS: interrupted candidates cannot blind-turn; zero-length rows retain grey count authorization');
