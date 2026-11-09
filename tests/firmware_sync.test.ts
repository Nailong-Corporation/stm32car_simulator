import assert from 'node:assert/strict';
import { LineFollowModel } from '../src/sim/line_follow';
import { RoutePlannerModel } from '../src/sim/route';
import { grayClassify } from '../src/sim/gray_sensor';
import { parseRouteTableC, DEFAULT_ROUTE_TABLE_C } from '../src/sim/route_parser';
import {
  TURN_PIVOT_APPROACH_CM,
  ROUTE_WAYPOINT_MAX_PASSED_LAG_CM,
  LEFT_WHEEL_DIAMETER_CM,
  RIGHT_WHEEL_DIAMETER_CM,
  LEFT_COUNTS_PER_WHEEL_REV,
  RIGHT_COUNTS_PER_WHEEL_REV,
  ODOMETRY_DISTANCE_SCALE,
  GRAY_FORWARD_OFFSET_CM,
  LINE_CONFIRM_SAMPLES,
} from '../src/sim/config';
import { Action, LineState, RouteState } from '../src/sim/types';

/* Firmware sync contract (2026-11-08, firmware b167058 "转弯参数和编码器参数修正").
 *
 * These tests pin the parts of the port that a full-lap run does not exercise:
 *   - the encoder / pivot / waypoint constants equal the firmware values;
 *   - START realigns the IMU heading zero and the odometry pose (rotated POWER-ON
 *     bodies must not hand over the first row at d = 0);
 *   - a row with segment_length_cm == 0 cannot be handed over by the pose;
 *   - a corner more than ROUTE_WAYPOINT_MAX_PASSED_LAG_CM behind the bar is not
 *     "already passed";
 *   - commented-out route_table.c step lines are not parsed as active steps.
 */

// 1) Parameters copied one-for-one from include/motion_config.h.
assert.equal(TURN_PIVOT_APPROACH_CM, 10.0);
assert.equal(ROUTE_WAYPOINT_MAX_PASSED_LAG_CM, 30.0);
assert.equal(LEFT_WHEEL_DIAMETER_CM, 4.8);
assert.equal(RIGHT_WHEEL_DIAMETER_CM, 4.8);
assert.equal(LEFT_COUNTS_PER_WHEEL_REV, 1064.0);
assert.equal(RIGHT_COUNTS_PER_WHEEL_REV, 1064.0);
assert.equal(ODOMETRY_DISTANCE_SCALE, 1.0);
console.log('PASS: firmware motion_config.h values are mirrored (wheel 4.8, 1064 counts, scale 1.0, pivot 10, lag 30)');

// 2) START is the pose origin: a body rotated after power-on must not leave a
//    non-zero heading, and the first row must not be accepted at d ~ 0.
{
  const scheme = parseRouteTableC(DEFAULT_ROUTE_TABLE_C).schemes.find((s) => s.id === 1);
  assert.ok(scheme);
  const f = new LineFollowModel(scheme.steps);
  const obs = grayClassify(0x18, true, 0);
  const counts = { left: 0, right: 0 };
  const powerOnHeading = Math.PI; // body carried/rotated ~180 deg after power-on
  for (let i = 0; i < LINE_CONFIRM_SAMPLES; i++) {
    f.update(obs, counts, i * 5, 0.005, i === LINE_CONFIRM_SAMPLES - 1, true, powerOnHeading);
  }
  assert.equal(f.state, LineState.LINE_TRACKING, 'START must be accepted');
  assert.equal(f.route.state, RouteState.ROUTE_FOLLOW);
  assert.equal(f.route.index, 0);
  assert.ok(Math.abs(f.imu_heading_offset_rad - powerOnHeading) < 1e-9,
    `START must latch the IMU zero, got ${f.imu_heading_offset_rad}`);
  assert.ok(Math.abs(f.odometry.heading_rad) < 1e-9,
    `START must zero the odometry heading, got ${f.odometry.heading_rad}`);
  assert.ok(Math.abs(f.odometry.x_cm) < 1e-9 && Math.abs(f.odometry.y_cm) < 1e-9,
    'START must zero the odometry pose');
  assert.ok(Math.abs(f.odometry.sensor_x_cm - GRAY_FORWARD_OFFSET_CM) < 1e-9 &&
    Math.abs(f.odometry.sensor_y_cm) < 1e-9, 'START must zero the sensor pose');

  // Drive 5.4 cm straight: no action may be handed over (the old bug fired the
  // first CUSTOM_TURN at 0.00 cm).
  let now = LINE_CONFIRM_SAMPLES * 5;
  for (let i = 0; i < 96; i++) {
    now += 5;
    counts.left -= 4; // LEFT sign is -1, so a decreasing raw count is forward
    counts.right += 4;
    f.update(obs, counts, now, 0.005, false, true, powerOnHeading);
    assert.equal(f.route.index, 0);
    assert.notEqual(f.route.state, RouteState.ROUTE_EXECUTING,
      `first row accepted too early at d=${f.odometry.distance_cm.toFixed(2)} cm`);
  }
  assert.ok(f.odometry.distance_cm > 5.0, `expected >5 cm, got ${f.odometry.distance_cm}`);
  assert.ok(f.route.pose_gap_bar_cm > 20,
    `bar-to-corner gap must stay large, got ${f.route.pose_gap_bar_cm}`);
  console.log(`PASS: START realigns IMU/odometry pose and the first row is not fired at d=0 (${f.odometry.distance_cm.toFixed(1)} cm)`);
}

// 3) A segment_length_cm == 0 row has no usable corner: the pose must not hand
//    it over even when the bar sits past the degenerate waypoint.
{
  const steps = [
    { junction_count: 1, turn_speed: 20, action: Action.CUSTOM_TURN, cruise_speed: 40,
      angle_deg: 90, corner_arc_enabled: false, segment_length_cm: 0, short_link_cm: 0 },
    { junction_count: 1, turn_speed: 20, action: Action.STOP, cruise_speed: 40,
      angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 0, short_link_cm: 0 },
  ];
  const r = new RoutePlannerModel();
  r.init(steps);
  r.start(0, 0);
  r.setSpeed(0, 0);
  // Axle exactly on the anchored start pose; the bar is 24 cm ahead of it.
  r.update(grayClassify(0x18, true, 0), 0, 0, 0, 0, 110);
  assert.equal(r.index, 0);
  assert.notEqual(r.state, RouteState.ROUTE_EXECUTING,
    'a zero-length row must not be handed over by the pose');
  console.log('PASS: segment_length_cm == 0 rows are not pose-triggered');
}

// 4) "Already passed" is bounded: a corner far behind the bar cannot fire.
{
  const steps = [
    { junction_count: 4, turn_speed: 20, action: Action.CUSTOM_TURN, cruise_speed: 40,
      angle_deg: 90, corner_arc_enabled: false, segment_length_cm: 100, short_link_cm: 0 },
    { junction_count: 1, turn_speed: 20, action: Action.STOP, cruise_speed: 40,
      angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 0, short_link_cm: 0 },
  ];
  const r = new RoutePlannerModel();
  r.init(steps);
  r.start(0, 0);
  r.setSpeed(0, 0);
  // Start pose (0,110) heading north -> row 0 corner is (0,10). Put the axle
  // 60 cm past it: bar_proj = -84 cm, well beyond the allowed lag.
  r.update(grayClassify(0x18, true, 0), 0, 0, 0, 0, -50);
  assert.equal(r.index, 0);
  assert.notEqual(r.state, RouteState.ROUTE_EXECUTING,
    'a corner beyond ROUTE_WAYPOINT_MAX_PASSED_LAG_CM must not count as passed');
  console.log('PASS: waypoint "already passed" is bounded by ROUTE_WAYPOINT_MAX_PASSED_LAG_CM');
}

// 5) Commented-out step lines in route_table.c are not active table rows.
{
  const parsed = parseRouteTableC(DEFAULT_ROUTE_TABLE_C);
  const scheme0 = parsed.schemes.find((s) => s.id === 0);
  assert.ok(scheme0);
  assert.equal(scheme0.steps.length, 4,
    'scheme 0 has one commented-out row plus four active rows');
  // 2026-11-08 调试路线第 0 行改为 junction_count=5 / arc=true（230 cm），随表同步。
  assert.equal(scheme0.steps[0].junction_count, 5);
  assert.equal(scheme0.steps[0].segment_length_cm, 230);

  const custom = [
    'const RouteStep route[] = {',
    '#if (ROUTE_SCHEME_SELECT == 0)',
    '    //{ 1, 50.0f, CUSTOM_TURN, 50, -90.0f, false, 100.0f, 0.0f},',
    '    /* { 9, 50.0f, STOP, 80, 0.0f, false, 1.0f, 0.0f } */',
    '    { 1, 50.0f, STOP, 80, 0.0f, false, 5.0f, 0.0f}',
    '#endif',
    '};',
  ].join('\n');
  const scheme = parseRouteTableC(custom).schemes.find((s) => s.id === 0);
  assert.ok(scheme);
  assert.equal(scheme.steps.length, 1, 'line and block comments must be ignored');
  assert.equal(scheme.steps[0].junction_count, 1);
  console.log('PASS: commented-out route steps are ignored by the parser');
}

console.log('PASS: firmware sync contract (params, START realign, pose gates, parser comments)');
