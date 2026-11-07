import assert from 'node:assert/strict';
import { RoutePlannerModel } from '../src/sim/route';
import { grayClassify } from '../src/sim/gray_sensor';
import { Action, RouteState, RouteFailure, type RouteStep } from '../src/sim/types';
import { COMPONENT_RECOVERY_MAX_CM, COMPONENT_RECOVERY_TIMEOUT_MS,
  JUNCTION_ACTION_TIMEOUT_MS, JUNCTION_TURN_MAX_RAD, JUNCTION_WRONG_WAY_RAD } from '../src/sim/config';

function fixture(action: Action, angle: number, pose: boolean, arc = false) {
  const row: RouteStep = { action, angle_deg: angle, junction_count: 1,
    segment_length_cm: 100, turn_speed: 28, cruise_speed: 100, corner_arc_enabled: arc };
  const r = new RoutePlannerModel();
  r.init([row, { ...row, action: Action.STOP, corner_arc_enabled: false }]);
  r.pose_enabled = pose; r.repeat_count = 1; r.start(0, 0);
  r.state = RouteState.ROUTE_EXECUTING;
  r.entry_heading = 0; r.phase_ms = 0; r.turn_approaching = false;
  let now = 0, d = 0;
  const target = r.turnTargetRad();
  const feed = (yaw: number, mask = 0, advance = .1, dt = 5) => {
    now += dt; d += advance;
    r.update(grayClassify(mask, true, 0), d, yaw, now,
      pose ? r.waypoint_x_cm[0] : NaN, pose ? r.waypoint_y_cm[0] : NaN);
  };
  return { r, target, feed, now: () => now, distance: () => d };
}

for (const pose of [false, true]) {
  for (const [action, angle] of [[Action.LEFT, 0], [Action.RIGHT, 0],
    [Action.CUSTOM_TURN, -90], [Action.CUSTOM_TURN, 90],
    [Action.CUSTOM_TURN, -135], [Action.CUSTOM_TURN, 135]] as const) {
    for (const mask of [0, 0x18]) {
      const { r, target, feed, now, distance } = fixture(action, angle, pose);
      // A stable narrow line at 85-99.9% must never steal the final angle.
      for (const fraction of [.85, .95, .98, .999, .999, .999]) feed(target * fraction, mask);
      assert.equal(r.state, RouteState.ROUTE_EXECUTING);
      assert.equal(r.turn_capturing, false);
      assert.equal(r.line_exit_confirmed, false);
      assert.equal(r.index, 0);
      feed(target, 0);
      assert.equal(r.state, RouteState.ROUTE_CLEARING);
      assert.equal(r.turn_exit_search, true);
      assert.equal(r.turn_ended_on_line, false);
      assert.equal(r.line_exit_confirmed, false);
      assert.equal(r.line_exit_heading_rad, target);
      const anchor = r.next_segment_origin_cm;
      feed(target, 0, 2.1);
      assert.equal(r.index, 0, 'pose 2cm clearance cannot bypass line recovery');
      feed(target, 0x18); feed(target, 0x18);
      assert.equal(r.turn_exit_search, true);
      feed(target, 0); // interrupted evidence cannot carry over
      feed(target, 0x18); feed(target, 0x18);
      assert.equal(r.turn_exit_search, true);
      feed(target, 0x18);
      assert.equal(r.turn_exit_search, false);
      assert.equal(r.line_exit_confirmed, true);
      assert.equal(r.index, 0, 'line recovery itself cannot advance the row');
      assert.equal(r.capture_line_ms, now());
      assert.equal(r.capture_line_cm, distance());
      assert.equal(r.clear_cm, distance());
      assert.equal(r.phase_ms, now());
      assert.equal(r.next_segment_origin_cm, anchor, 'preserve full-angle segment anchor');
      assert.equal(r.exit_count, 0);
    }
  }
}
console.log('PASS: strict +/-90/135 yaw gate with pose on/off, early narrow/no line and three-frame exit recovery');

for (const action of [Action.LEFT, Action.RIGHT]) {
  const { r, target, feed } = fixture(action, 0, true, true);
  // Firmware 2026-11-07 (uncommitted route.c): a rolling arc also ends on its
  // own tangent geometry. The encoder heading lags the real yaw, so waiting for
  // the measured angle would over-rotate past the outgoing line and lose it.
  r.corner_arc_complete = true;
  feed(target * .98, 0x18);
  assert.equal(r.state, RouteState.ROUTE_CLEARING);
  assert.equal(r.turn_exit_search, true);
  assert.equal(r.turn_ended_on_line, false);
  assert.equal(r.line_exit_confirmed, false);
  assert.equal(r.arc_just_exited, true);
  assert.equal(r.line_exit_heading_rad, r.arc_desired_heading_rad,
    'arc exit uses the measured tangent, not the nominal table angle');
}
console.log('PASS: completed rolling arcs exit on tangent geometry with the arc desired heading');

{
  const { r, target, feed } = fixture(Action.RIGHT, 0, false);
  feed(target);
  for (const mask of [0xff, 0x03, 0x81]) {
    feed(target, 0x18); feed(target, 0x18); feed(target, mask);
    assert.equal(r.turn_exit_search, true, 'wide, edge and separated lines cannot confirm recovery');
  }
  feed(target, 0x18); feed(target, 0x18);
  feed(target, 0x18, .1, 20);
  assert.equal(r.turn_exit_search, true, 'sample discontinuity restarts the three-frame proof');
  feed(target, 0x18); feed(target, 0x18);
  assert.equal(r.turn_exit_search, false);
}
console.log('PASS: recovery rejects wide/edge/separated lines and discontinuous samples');

for (const axis of ['time', 'forward', 'reverse'] as const) {
  const { r, target, feed } = fixture(Action.RIGHT, 0, true);
  feed(target, 0);
  feed(target, 0x18); feed(target, 0x18);
  feed(target, 0x18,
    axis === 'time' ? 0 : (axis === 'forward' ? COMPONENT_RECOVERY_MAX_CM : -COMPONENT_RECOVERY_MAX_CM - .001) - .2,
    axis === 'time' ? COMPONENT_RECOVERY_TIMEOUT_MS - 10 : 5);
  assert.equal(r.failure, RouteFailure.ROUTE_FAILURE_EXIT_RECAPTURE, `${axis}: deadline beats third frame`);
  assert.equal(r.state, RouteState.ROUTE_FAILED);
  assert.equal(r.index, 0);
  assert.equal(r.recoverNavigation(0, target, 2000), false, 'exit search failure cannot soft recover');
}
console.log('PASS: search time/absolute-distance limits beat same-frame third line sample and forbid soft recovery');

for (const which of ['timeout', 'wrong', 'over'] as const) {
  const { r, target, feed } = fixture(Action.RIGHT, 0, true);
  const yaw = which === 'wrong' ? -JUNCTION_WRONG_WAY_RAD - .01 :
    which === 'over' ? JUNCTION_TURN_MAX_RAD + .01 : target;
  feed(yaw, 0x18, .1, which === 'timeout' ? JUNCTION_ACTION_TIMEOUT_MS : 5);
  assert.equal(r.state, RouteState.ROUTE_FAILED);
  assert.equal(r.turn_exit_search, false);
  assert.equal(r.failure, which === 'timeout' ? RouteFailure.ROUTE_FAILURE_ACTION_TIMEOUT :
    which === 'wrong' ? RouteFailure.ROUTE_FAILURE_WRONG_WAY : RouteFailure.ROUTE_FAILURE_TURN_LIMIT);
}
console.log('PASS: action timeout, wrong direction and maximum angle retain priority over full-angle exit');
