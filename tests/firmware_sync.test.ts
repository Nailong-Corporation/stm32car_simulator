import assert from 'node:assert/strict';
import { LineFollowModel } from '../src/sim/line_follow';
import { RoutePlannerModel, routeGrayShowsBranch } from '../src/sim/route';
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
  JUNCTION_SHORT_LINK_EXIT_EDGE_STABLE_MS,
  JUNCTION_SHORT_LINK_EXIT_EDGE_STABLE_CM,
  JUNCTION_EXIT_SEARCH_EDGE_HEADING_RAD,
  LINE_CONFIRM_SAMPLES,
} from '../src/sim/config';
import { Action, LineState, RouteBarHoldState, RouteState } from '../src/sim/types';

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
assert.equal(JUNCTION_EXIT_SEARCH_EDGE_STABLE_MS, 100);
assert.ok(Math.abs(JUNCTION_EXIT_SEARCH_EDGE_STABLE_CM - 0.5) < 1e-9);
assert.ok(Math.abs(JUNCTION_EXIT_SEARCH_EDGE_HEADING_RAD - 0.30) < 1e-9);
assert.equal(JUNCTION_SHORT_LINK_EXIT_POSITION_LIMIT, 3.0);
assert.equal(JUNCTION_SHORT_LINK_EXIT_EDGE_STABLE_MS, 50);
assert.ok(Math.abs(JUNCTION_SHORT_LINK_EXIT_EDGE_STABLE_CM - 0.5) < 1e-9);
console.log('PASS: firmware motion_config.h values are mirrored (wheel 4.8, 1064 counts, scale 1.0, heading 0.748, gate 27.85, max steps 64, edge exit 100 ms/0.5 cm/0.30 rad, short-link edge 50 ms)');

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
  /* Firmware 2e751de anchors scheme 1 on the ring-test pose as well. That looks
   * like a leftover from the scheme-8 test (row 0 still drives 105 cm north from
   * P); the simulator mirrors the table so parity and the web preview agree, and
   * the defect is reported in the tracking issue. */
  assert.ok(scheme1.start_pose, 'scheme 1 mirrors ROUTE_SCHEME_SELECT==1 in route_scenario_start_pose');
  assert.deepEqual(scheme1.start_pose, { x_cm: 345, y_cm: 110, heading_rad: 1.570796327 });
  const scheme4 = parsed.schemes.find((s) => s.id === 4);
  assert.ok(scheme4);
  assert.equal(scheme4.steps.length, 29, 'scheme 4 skips the commented HEXAGON row (29 active rows)');
  assert.equal(scheme4.steps[0].junction_count, 1, 'scheme 4 exit search fixture starts with one node');
  assert.equal(scheme4.steps[0].turn_speed, 28);
  assert.equal(scheme4.steps[2].cruise_speed, 120, 'firmware 2e751de: x55 south leg cruises at 120');
  assert.equal(scheme4.steps[3].cruise_speed, 160, 'firmware 2e751de: Q6 leg cruises at 160');
  assert.equal(scheme4.steps[5].cruise_speed, 120);
  assert.equal(scheme4.steps[10].action, Action.CUSTOM_TURN,
    'the commented-out HEXAGON row must not be parsed');
  assert.equal(scheme4.steps[10].junction_count, 2);
  assert.equal(scheme4.steps[10].segment_length_cm, 90,
    'firmware 2e751de: x=345 south leg measures 90 cm');
  assert.equal(scheme4.steps[10].turn_approach_cm, 19,
    'x=345 south leg crosses the ring vertex with the 14+5 cm transistor hold');
  assert.equal(scheme4.steps[12].junction_count, 5,
    'firmware 2e751de: x375 north leg counts five nodes (R12/R15/y110/y50)');
  assert.equal(scheme4.steps[13].junction_count, 2,
    'firmware 2e751de: top-edge (275,0) row counts Q16/R17');
  assert.equal(scheme4.steps[14].turn_approach_cm, 19,
    'Q18 row keeps the 14+5 cm transistor hold');
  assert.equal(scheme4.steps[17].junction_count, 7,
    'firmware 59a7268 top-edge Q22 row counts seven nodes');
  assert.equal(scheme4.steps[17].cruise_speed, 160,
    'firmware 2e751de top-edge Q22 row cruises at 160');
  assert.equal(scheme4.steps[7].segment_length_cm, 265);
  assert.equal(scheme4.steps[28].action, Action.STOP);
  /* route_scenario_close_corner(): the R17 row is marked by content, not index,
   * and only for the schemes in the firmware gate {1,4,5,8}. */
  assert.equal(scheme4.steps[13].close_corner, true,
    'R17 row (95 cm, -90 deg, after a 215 cm leg) must take the geometric-corner pivot');
  assert.equal(scheme4.steps.filter((step) => step.close_corner).length, 1,
    'exactly one close-corner row per table');
  const scheme2 = parsed.schemes.find((s) => s.id === 2);
  assert.ok(scheme2);
  assert.equal(scheme2.steps.some((step) => step.close_corner), false,
    'scheme 2 is outside the firmware close-corner gate');
  const scheme8 = parsed.schemes.find((s) => s.id === 8);
  assert.ok(scheme8, 'firmware 2e751de adds the scheme-8 ring test');
  assert.equal(scheme8.steps.length, 23);
  assert.equal(scheme8.steps.filter((step) => step.action === Action.HEXAGON).length, 0,
    'scheme 8 runs the ring by the ordinary table rows');
  assert.deepEqual(scheme8.start_pose, { x_cm: 345, y_cm: 110, heading_rad: 1.570796327 },
    'scheme 8 starts on the x=345 approach heading south');
  assert.equal(scheme8.steps[17].junction_count, 9,
    'scheme 8 top-edge row keeps the nine-node count');
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

// 5b) 2026-11-10 firmware 59a7268: merged branch detection is shared by the
//     route counter and the line-follow steering gate.  A single region with
//     centre+side evidence (field 0x70) must not be handed to the position PID;
//     a plain centre line or an outermost single line must stay usable.
{
  assert.equal(routeGrayShowsBranch(grayClassify(0x70, true, 3)), true,
    '0x70 (centre+side merged branch) must be held on the segment heading');
  assert.equal(routeGrayShowsBranch(grayClassify(0x18, true, 0)), false,
    'a plain centre line is not a branch');
  assert.equal(routeGrayShowsBranch(grayClassify(0x03, true, -6)), false,
    'an outermost single line is an exit edge, not a branch');
  assert.equal(routeGrayShowsBranch(grayClassify(0x00, true, 0)), false,
    'a lost line is not a branch');
  console.log('PASS: route_gray_shows_branch mirrors firmware (0x70 branch, 0x18/0x03/0x00 usable/none)');
}

// 6) 2026-11-11 field regression (index 9, Q13 left turn -> ring entrance; top
//    right corner): the post-pivot search holds the exit heading and freezes the
//    grey PID, so the real outgoing line can sit at the outermost pair
//    (0x03, position=-6) until the 12 cm budget runs out. A continuously visible
//    narrow edge line *aligned with the outgoing axis* is exit evidence after
//    100 ms / 0.5 cm; a single-frame stroke and a line that is not on the exit
//    axis are not.
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
  const exitAxis = Math.PI / 2; // entry 0 + 90 deg turn
  const arm = (planner: RoutePlannerModel) => {
    planner.index = 3;
    planner.state = RouteState.ROUTE_EXECUTING;
    planner.entry_heading = 0;
    planner.phase_ms = 0;
    planner.capture_line_ms = 0;
    planner.capture_line_cm = 0;
    planner.exit_count = 0;
    planner.exit_candidate_pending = false;
    planner.turn_exit_search = true;
  };
  const r = new RoutePlannerModel();
  r.init(steps);
  r.start(0, 0);
  r.setSpeed(10, 10);
  // Arm the field fixture directly: index 9 reached full 90 deg and the search
  // is running with the outgoing line at the bar edge.
  arm(r);
  let now = 0;
  let d = 0;
  const edge = grayClassify(0x03, true, 0); // outermost single region, position -6
  const tick = (o: typeof edge, ds: number, h = exitAxis) => {
    now += 5;
    d += ds;
    r.update(o, d, h, now);
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
  const travelled = Number.isFinite(r.capture_line_cm) ? frames * 0.1 : 0;
  assert.ok(travelled >= JUNCTION_EXIT_SEARCH_EDGE_STABLE_CM,
    `edge hand-off must travel >=${JUNCTION_EXIT_SEARCH_EDGE_STABLE_CM} cm, got ${travelled.toFixed(2)}`);
  assert.ok(travelled < JUNCTION_EXIT_SEARCH_MAX_CM);
  assert.ok(now < JUNCTION_EXIT_SEARCH_TIMEOUT_MS);
  assert.ok(frames * 5 >= JUNCTION_EXIT_SEARCH_EDGE_STABLE_MS,
    `edge hand-off must hold >=${JUNCTION_EXIT_SEARCH_EDGE_STABLE_MS} ms, got ${frames * 5} ms`);
  console.log(`PASS: stable off-centre (0x03) exit line on the exit axis confirms after ${travelled.toFixed(1)} cm / ${frames * 5} ms; single-frame strokes are rejected`);

  /* The same visible edge line while the car is still 1.0 rad off the outgoing
   * axis is not exit evidence (firmware 2e751de JUNCTION_EXIT_SEARCH_EDGE_
   * HEADING_RAD = 0.30): the search keeps running. */
  const hidden = new RoutePlannerModel();
  hidden.init(steps);
  hidden.start(0, 0);
  hidden.setSpeed(10, 10);
  arm(hidden);
  let hnow = 0;
  let hd = 0;
  let offAxisFrames = 0;
  for (; offAxisFrames < 40 && hidden.turn_exit_search; offAxisFrames++) {
    hnow += 5; hd += 0.1;
    hidden.update(edge, hd, exitAxis - 1.0, hnow);
  }
  assert.equal(offAxisFrames, 40, 'an off-axis edge line must not confirm the exit');
  assert.equal(hidden.turn_exit_search, true);
  assert.equal(hidden.line_exit_confirmed, false);
  console.log('PASS: an off-axis edge line (heading error 1.0 rad) never certifies the turn exit');
}

/* 7) Firmware 2e751de: a side-on transistor first covers only part of the bar
 *    (0x1f then 0x0f). That partial bar may hold the steering and grant a
 *    passage, but it must never become a corner anchor; a full 0xFF bar after a
 *    proven centre corridor gets its own anchor. */
{
  const steps = [
    { junction_count: 1, turn_speed: 28, action: Action.CUSTOM_TURN, cruise_speed: 80,
      angle_deg: 90, corner_arc_enabled: false, segment_length_cm: 100, short_link_cm: 0,
      turn_approach_cm: 9 },
    { junction_count: 1, turn_speed: 28, action: Action.STOP, cruise_speed: 80,
      angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 100, short_link_cm: 0 },
  ];
  const r = new RoutePlannerModel();
  r.init(steps);
  r.start(0, 0);
  r.setSpeed(10, 10);
  /* Drive updateBarHold() directly: the public update() also runs the candidate
   * FSM, which unbinds the hold on the same frame and would hide the latch. */
  const barHold = (r as unknown as {
    updateBarHold: (o: ReturnType<typeof grayClassify>, d: number, h: number, now: number) => void;
  }).updateBarHold.bind(r);
  const partial = grayClassify(0x1f, true, 0); // 5 channels, side-on transistor
  barHold(partial, 5, 0, 5);
  assert.equal(r.bar_hold_state, RouteBarHoldState.ROUTE_BAR_HOLD_SIDE,
    'a partial branch bar latches ROUTE_BAR_HOLD_SIDE');
  assert.notEqual(r.bar_hold_state, RouteBarHoldState.ROUTE_BAR_HOLD_PENDING,
    'a partial bar can never be the corner anchor');
  barHold(partial, 5.2, 0, 10);
  assert.equal(r.bar_hold_state, RouteBarHoldState.ROUTE_BAR_HOLD_SIDE,
    'repeated partial frames keep the SIDE latch (a symbol is not a corner)');
  const full = grayClassify(0xff, true, 0);
  r.state = RouteState.ROUTE_CANDIDATE;
  barHold(full, 6, 0, 15);
  assert.equal(r.bar_hold_state, RouteBarHoldState.ROUTE_BAR_HOLD_PENDING,
    'a full bar after the partial edge of the same candidate re-anchors as PENDING');
  console.log('PASS: partial (0x1f) transistor bars only hold/pass; a full 0xFF bar owns the corner anchor');
}

/* 8) Scenario start anchor: the table may start anywhere (scheme 8 ring test at
 *    (345,110) heading south). Pose fields are mileage-relative, so the bar must
 *    sit 24 cm in front along the anchor heading, not along a hard-coded north. */
{
  const parsed = parseRouteTableC(DEFAULT_ROUTE_TABLE_C);
  const scheme8 = parsed.schemes.find((s) => s.id === 8);
  assert.ok(scheme8);
  assert.ok(scheme8.start_pose);
  const pose = scheme8.start_pose;
  const r = new RoutePlannerModel();
  r.init(scheme8.steps, undefined, pose);
  r.start(0, 0);
  r.setSpeed(0, 0);
  r.update(grayClassify(0x18, true, 0), 0, 0, 0, pose.x_cm, pose.y_cm);
  const barAhead = 100 * Math.cos(pose.heading_rad);
  const rowLen = scheme8.steps[0].segment_length_cm + 5;
  const gap = Math.hypot(
    pose.x_cm + GRAY_FORWARD_OFFSET_CM * Math.cos(pose.heading_rad) -
      (pose.x_cm + rowLen * Math.cos(pose.heading_rad)),
    pose.y_cm + GRAY_FORWARD_OFFSET_CM * Math.sin(pose.heading_rad) -
      (pose.y_cm + rowLen * Math.sin(pose.heading_rad)));
  assert.ok(Number.isFinite(barAhead));
  assert.ok(Math.abs(gap - Math.abs(rowLen - GRAY_FORWARD_OFFSET_CM)) < 1e-6,
    `bar-to-corner gap must follow the anchored heading, got ${gap}`);
  assert.ok(Math.abs(r.pose_seg_heading_rad) < 1e-9,
    `scheme 8 row 0 must read as straight ahead, got ${r.pose_seg_heading_rad}`);
  assert.ok(Math.abs(r.pose_bearing_rad) < 1e-9);
  console.log('PASS: scenario start anchor keeps pose headings mileage-relative (scheme 8 southward start)');
}

console.log('PASS: firmware sync contract (params, START realign, grey authorization, close-corner table, scheme 8 anchor and edge exit)');
