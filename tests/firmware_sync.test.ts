import assert from 'node:assert/strict';
import { LineFollowModel } from '../src/sim/line_follow';
import { RoutePlannerModel } from '../src/sim/route';
import { grayClassify } from '../src/sim/gray_sensor';
import { parseRouteTableC, DEFAULT_ROUTE_TABLE_C } from '../src/sim/route_parser';
import {
  TURN_PIVOT_APPROACH_CM,
  ROUTE_WAYPOINT_MAX_PASSED_LAG_CM,
  ROUTE_REPEAT_COUNT,
  LEFT_WHEEL_DIAMETER_CM,
  RIGHT_WHEEL_DIAMETER_CM,
  LEFT_COUNTS_PER_WHEEL_REV,
  RIGHT_COUNTS_PER_WHEEL_REV,
  ODOMETRY_DISTANCE_SCALE,
  ODOMETRY_HEADING_SCALE,
  GRAY_FORWARD_OFFSET_CM,
  GRAY_SENSOR_HALF_SPAN_CM,
  ROUTE_GATE_LOOKAHEAD_CM,
  ROUTE_MAX_STEPS,
  JUNCTION_SETTLE_MIN_DIFF_CM_S,
  JUNCTION_EXIT_SEARCH_SPEED_CM_S,
  JUNCTION_EXIT_SEARCH_MAX_CM,
  JUNCTION_EXIT_SEARCH_TIMEOUT_MS,
  JUNCTION_EXIT_SEARCH_EDGE_STABLE_MS,
  JUNCTION_EXIT_SEARCH_EDGE_STABLE_CM,
  JUNCTION_SHORT_LINK_EXIT_POSITION_LIMIT,
  LINE_CONFIRM_SAMPLES,
} from '../src/sim/config';
import { Action, LineState, RouteState } from '../src/sim/types';

/* Firmware sync contract (2026-11-10, current firmware working tree).
 *
 * These tests pin the parts of the port that a full-lap run does not exercise:
 *   - the encoder / pivot / waypoint constants equal the firmware values;
 *   - START realigns the IMU heading zero and the odometry pose (rotated POWER-ON
 *     bodies must not hand over the first row at d = 0);
 *   - a row with segment_length_cm == 0 cannot be handed over by the pose;
 *   - pose proximity never authorizes ordinary FOLLOW actions (the full
 *     distance/grey contract is in route_pose_node.test.ts);
 *   - commented-out route_table.c step lines are not parsed as active steps.
 */

// 1) Parameters copied one-for-one from include/motion_config.h.
assert.equal(TURN_PIVOT_APPROACH_CM, 24.0); /* bar look-ahead = pivot travel */
assert.equal(ROUTE_WAYPOINT_MAX_PASSED_LAG_CM, 30.0);
assert.equal(ROUTE_REPEAT_COUNT, 1); /* motion_config.h ships one competition lap */
assert.equal(LEFT_WHEEL_DIAMETER_CM, 4.8);
assert.equal(RIGHT_WHEEL_DIAMETER_CM, 4.8);
assert.equal(LEFT_COUNTS_PER_WHEEL_REV, 1064.0);
assert.equal(RIGHT_COUNTS_PER_WHEEL_REV, 1064.0);
assert.equal(ODOMETRY_DISTANCE_SCALE, 1.0);
assert.equal(ODOMETRY_HEADING_SCALE, 0.748); /* 2026-11-10 differential field calibration */
assert.ok(Math.abs(GRAY_SENSOR_HALF_SPAN_CM - 3.85) < 1e-9);
assert.equal(ROUTE_GATE_LOOKAHEAD_CM, GRAY_FORWARD_OFFSET_CM + GRAY_SENSOR_HALF_SPAN_CM);
assert.ok(Math.abs(ROUTE_GATE_LOOKAHEAD_CM - 27.85) < 1e-9);
assert.equal(ROUTE_MAX_STEPS, 64);
assert.equal(JUNCTION_SETTLE_MIN_DIFF_CM_S, 6.0);
assert.equal(JUNCTION_EXIT_SEARCH_SPEED_CM_S, 16.0);
assert.equal(JUNCTION_EXIT_SEARCH_MAX_CM, 12.0);
assert.equal(JUNCTION_EXIT_SEARCH_TIMEOUT_MS, 1200);
assert.equal(JUNCTION_EXIT_SEARCH_EDGE_STABLE_MS, 250);
assert.ok(Math.abs(JUNCTION_EXIT_SEARCH_EDGE_STABLE_CM - 6.0) < 1e-9);
assert.equal(JUNCTION_SHORT_LINK_EXIT_POSITION_LIMIT, 3.0);
console.log('PASS: firmware motion_config.h values are mirrored (wheel 4.8, 1064 counts, scale 1.0, heading 0.748, gate 27.85, max steps 64, edge exit 250 ms/6 cm)');

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

// 4) A waypoint far behind the bar cannot authorize an ordinary action.
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
    'a waypoint cannot authorize an ordinary action without grey confirmation');
  console.log('PASS: a waypoint behind the bar cannot authorize an ordinary action');
}

// 5) Commented-out step lines in route_table.c are not active table rows.
{
  const parsed = parseRouteTableC(DEFAULT_ROUTE_TABLE_C);
  const scheme0 = parsed.schemes.find((s) => s.id === 0);
  assert.ok(scheme0);
  assert.equal(parsed.activeSchemeId, 4, 'bundled default must match firmware ROUTE_SCHEME_SELECT=4');
  assert.equal(scheme0.steps.length, 2, 'only the two active debug rows are parsed');
  const fields = ['junction_count', 'turn_speed', 'action', 'cruise_speed',
    'angle_deg', 'corner_arc_enabled', 'segment_length_cm', 'short_link_cm', 'turn_approach_cm'] as const;
  assert.deepEqual(scheme0.steps.map(step => fields.map(field => step[field])), [
    [2, 20, Action.CUSTOM_TURN, 50, 90, false, 50, 0, 9],
    [3, 20, Action.STOP, 50, 90, false, 40, 0, 0],
  ], 'debug rows must match the current firmware working tree in every field');
  const scheme1 = parsed.schemes.find((s) => s.id === 1);
  assert.ok(scheme1);
  assert.equal(scheme1.steps.length, 30, 'scheme 1 now uses the full 30-row transistor-aware table');
  assert.equal(scheme1.steps[3].turn_approach_cm, 19, 'Q6 row: 14 cm transistor hold + 5 cm calibration');
  assert.equal(scheme1.steps[7].segment_length_cm, 265, 'bottom-right gate calibrated on the car');
  assert.equal(scheme1.steps[29].action, Action.STOP);
  const scheme4 = parsed.schemes.find((s) => s.id === 4);
  assert.ok(scheme4);
  assert.equal(scheme4.steps.length, 29, 'scheme 4 skips the commented HEXAGON row (29 active rows)');
  assert.equal(scheme4.steps[0].junction_count, 1, 'scheme 4 exit search fixture starts with one node');
  assert.equal(scheme4.steps[0].turn_speed, 28);
  assert.equal(scheme4.steps[10].action, Action.CUSTOM_TURN,
    'the commented-out HEXAGON row must not be parsed');
  assert.equal(scheme4.steps[10].junction_count, 2);
  assert.equal(scheme4.steps[10].segment_length_cm, 100,
    'x=345 south leg measures to the bottom edge, not the ring centre');
  assert.equal(scheme4.steps[7].segment_length_cm, 265);
  assert.equal(scheme4.steps[28].action, Action.STOP);
  // ROUTE_REPEAT_COUNT=1 must reach a freshly initialised planner.
  const planner = new RoutePlannerModel();
  planner.init(scheme0.steps);
  assert.equal(planner.repeat_count, 1);

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

// 6) 2026-11-10 field regression (index 9, Q13 left turn -> ring entrance):
//    the post-pivot search holds the exit heading and freezes the grey PID, so
//    the real outgoing line can sit at the outermost pair (0x03, position=-6)
//    for the whole 12 cm budget. A continuously visible narrow edge line is
//    exit evidence after 250 ms / 6 cm; a single-frame stroke is not.
{
  const steps = [
    { junction_count: 1, turn_speed: 28, action: Action.STRAIGHT, cruise_speed: 80,
      angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 50, short_link_cm: 0 },
    { junction_count: 1, turn_speed: 28, action: Action.STRAIGHT, cruise_speed: 80,
      angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 50, short_link_cm: 0 },
    { junction_count: 1, turn_speed: 28, action: Action.STRAIGHT, cruise_speed: 80,
      angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 50, short_link_cm: 0 },
    { junction_count: 3, turn_speed: 28, action: Action.CUSTOM_TURN, cruise_speed: 80,
      angle_deg: 90, corner_arc_enabled: false, segment_length_cm: 185, short_link_cm: 0,
      turn_approach_cm: 19 },
    { junction_count: 1, turn_speed: 28, action: Action.CUSTOM_TURN, cruise_speed: 80,
      angle_deg: 90, corner_arc_enabled: false, segment_length_cm: 25, short_link_cm: 0,
      turn_approach_cm: 9 },
    { junction_count: 1, turn_speed: 28, action: Action.STOP, cruise_speed: 80,
      angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 100, short_link_cm: 0 },
  ];
  const r = new RoutePlannerModel();
  r.init(steps);
  r.start(0, 0);
  r.setSpeed(10, 10);
  // Arm the field fixture directly: index 9 reached full 90 deg and the search
  // is running with the outgoing line at the bar edge.
  r.index = 3;
  r.state = RouteState.ROUTE_EXECUTING;
  r.entry_heading = 0;
  r.phase_ms = 0;
  r.capture_line_ms = 0;
  r.capture_line_cm = 0;
  r.exit_count = 0;
  r.exit_candidate_pending = false;
  r.turn_exit_search = true;
  let now = 0;
  let d = 0;
  const edge = grayClassify(0x03, true, 0); // outermost single region, position -6
  const tick = (o: typeof edge, ds: number) => {
    now += 5;
    d += ds;
    r.update(o, d, 0, now);
  };

  tick(edge, 0.1);
  assert.equal(r.exit_count, 0);
  assert.equal(r.turn_exit_search, true);
  assert.equal(r.line_exit_confirmed, false);
  for (let i = 0; i < 20; i++) tick(grayClassify(0, true, 0), 0.1); // 2 cm gap resets
  assert.equal(r.turn_exit_search, true);
  assert.equal(r.line_exit_confirmed, false);
  tick(edge, 0.1);
  assert.equal(r.exit_count, 0);
  assert.equal(r.turn_exit_search, true);

  let frames = 0;
  while (r.turn_exit_search && frames < 200) {
    tick(edge, 0.1);
    frames++;
  }
  assert.equal(r.turn_exit_search, false, 'stable edge line must confirm the exit');
  assert.equal(r.line_exit_confirmed, true);
  assert.equal(r.capture_line_seen, true);
  assert.equal(r.state, RouteState.ROUTE_CLEARING);
  assert.equal(r.index, 3);
  assert.ok(d - r.capture_line_cm >= 5.5, `edge hand-off must travel >=5.5 cm, got ${(d - r.capture_line_cm).toFixed(2)}`);
  assert.ok(d - r.capture_line_cm < JUNCTION_EXIT_SEARCH_MAX_CM);
  assert.ok(now - 0 < JUNCTION_EXIT_SEARCH_TIMEOUT_MS);
  console.log(`PASS: stable off-centre (0x03) exit line confirms after ${(d - r.capture_line_cm).toFixed(1)} cm / ${now} ms; single-frame strokes are rejected`);
}

console.log('PASS: firmware sync contract (params, START realign, grey authorization, current debug table, default scheme 4 and edge exit)');
