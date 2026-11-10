import assert from 'node:assert/strict';
import { LineFollowModel } from '../src/sim/line_follow';
import { grayClassify } from '../src/sim/gray_sensor';
import { routeIsNarrowLine } from '../src/sim/route';
import {
  JUNCTION_CENTER_LIMIT,
  JUNCTION_POSE_GRAY_SPREAD,
  LEFT_COUNTS_PER_WHEEL_REV,
  LEFT_WHEEL_DIAMETER_CM,
  ODOMETRY_DISTANCE_SCALE,
  ROUTE_WAYPOINT_GATE_CM,
} from '../src/sim/config';
import { Action, GrayKind, LineState, RouteState, type RouteStep } from '../src/sim/types';

/* 2026-11-10 gray_plausible() contract (firmware src/line_follow.c, 1:1 here).
 *
 * With a valid pose segment the pose used to veto an off-centre narrow line when
 * |position - (-pose_cross/0.55)| exceeded JUNCTION_POSE_GRAY_SPREAD. A real
 * wire hugging the bar edge on a bend the route table has no action for (the
 * 24 cm bar already sits on the next straight) therefore looked "implausible":
 * the controller held the pose heading with zero differential and the car drove
 * blindly until LINE_STOP_LOST (host probe: position +5 -> 0.00 cm/s difference
 * -> lost 35.1 cm later).
 *
 * New rule: with a valid pose segment every single-region narrow line is
 * trusted; the pose only vetoes wide / multi-region / hollow-artwork readings
 * and a blank bar. Without a valid pose segment the reading is trusted
 * unconditionally, exactly as before.
 */

const CM_PER_COUNT =
  (Math.PI * LEFT_WHEEL_DIAMETER_CM) / LEFT_COUNTS_PER_WHEEL_REV * ODOMETRY_DISTANCE_SCALE;

const steps: RouteStep[] = [
  {
    junction_count: 4, turn_speed: 28, action: Action.STRAIGHT, cruise_speed: 60,
    angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 1000,
  },
  {
    junction_count: 1, turn_speed: 28, action: Action.STOP, cruise_speed: 60,
    angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 100,
  },
];

type Rig = {
  f: LineFollowModel;
  counts: { left: number; right: number };
  feed: (obs: ReturnType<typeof grayClassify>, cm?: number) => void;
  now: () => number;
};

function startDriving(): Rig {
  const f = new LineFollowModel(steps);
  f.route.repeat_count = 1;
  const counts = { left: 0, right: 0 };
  let now = 0;
  const center = grayClassify(0x18, true, 0);
  const feed = (obs: ReturnType<typeof grayClassify>, cm = 0): void => {
    const ticks = Math.round(cm / CM_PER_COUNT);
    counts.left -= ticks; /* ENCODER_LEFT_SIGN = -1 */
    counts.right += ticks; /* ENCODER_RIGHT_SIGN = +1 */
    now += 5;
    f.update(obs, counts, now, 0.005, false, true, 0);
  };
  for (let i = 0; i < 3; i++) {
    counts.left -= 0; counts.right += 0;
    now += 5;
    f.update(center, counts, now, 0.005, i === 2, true, 0);
  }
  assert.equal(f.state, LineState.LINE_TRACKING, 'START must be accepted on a centred line');
  /* Settle at row cruise along the first straight (the row is 1000 cm so the
   * junction / braking envelope cannot interfere with this fixture). */
  for (let i = 0; i < 400; i++) feed(center, 0.9);
  return { f, counts, feed, now: () => now };
}

function edgeMasks(): { mask: number; sign: number }[] {
  return [
    { mask: 0x20, sign: 1 },  /* S5 alone, position +3 */
    { mask: 0xE0, sign: 1 },  /* S5..S7, position +5 */
    { mask: 0x04, sign: -1 }, /* S2 alone, position -3 */
    { mask: 0x07, sign: -1 }, /* S0..S2, position -5 */
  ];
}

for (const { mask, sign } of edgeMasks()) {
  const rig = startDriving();
  const f = rig.f;
  const edge = grayClassify(mask, true, 0);
  assert.equal(edge.kind, GrayKind.GRAY_VALID);
  assert.equal(edge.region_count, 1);
  assert.ok(edge.white_count >= 1 && edge.white_count <= 3);
  assert.ok(Math.abs(edge.position) >= 3 - 1e-9);

  /* A valid pose segment that says "on the segment" is exactly the old veto. */
  assert.ok(f.route.pose_mode, 'fixture must be in pose mode');
  assert.ok(f.route.pose_cross_valid, 'fixture must expose a valid pose segment');
  assert.ok(Math.abs(f.route.pose_cross_cm) <= 2.0,
    `fixture must sit on its segment, got cross ${f.route.pose_cross_cm}`);

  /* Contrast with the retired rule (documented, not executed by the port). */
  const expected = -f.route.pose_cross_cm / 0.55;
  const old_trusted =
    Math.abs(edge.position) <= JUNCTION_CENTER_LIMIT ||
    Math.abs(f.route.pose_cross_cm) > ROUTE_WAYPOINT_GATE_CM ||
    Math.abs(edge.position - expected) <= JUNCTION_POSE_GRAY_SPREAD;
  /* |position| <= JUNCTION_POSE_GRAY_SPREAD (4) was trusted by the old rule too;
   * only a reading hugging the bar edge (position +/-5) was vetoed. That is the
   * case this regression pins: old -> pose hold, new -> grey PID. */
  assert.equal(old_trusted, Math.abs(edge.position) <= JUNCTION_POSE_GRAY_SPREAD,
    `retired rule contrast for position ${edge.position}`);

  for (let i = 0; i < 8; i++) rig.feed(edge, 0.9);
  assert.equal(f.state, LineState.LINE_TRACKING,
    `mask=0x${mask.toString(16)} must be followed by the grey PID, not held by the pose`);
  assert.equal(f.abnormal, false);
  const diff = f.left_target_cm_s - f.right_target_cm_s;
  assert.ok(diff * sign > 0, `differential must steer towards position ${edge.position}, got ${diff}`);
  assert.ok(Math.abs(diff) >= 3.0, `differential must be a real correction, got ${diff}`);
  /* Following the line must not look like a junction to the route. */
  assert.equal(f.route.state, RouteState.ROUTE_FOLLOW);
  assert.equal(f.route.index, 0);
}

/* Guard: the pose still vetoes everything that is NOT a single narrow line.
 * The new rule only widened the "single region narrow" class; wide bars,
 * multi-region readings and a blank bar keep the pose hold / junction handling. */
{
  const wide = grayClassify(0xff, true, 0);
  assert.equal(routeIsNarrowLine(wide), false, 'a full 8-channel bar is not narrow');
  const multi = grayClassify(0x81, true, 0);
  assert.equal(routeIsNarrowLine(multi), false, 'a two-region reading is not narrow');
  const blank = grayClassify(0x00, true, 0);
  assert.equal(routeIsNarrowLine(blank), false, 'a blank bar is not narrow');

  const rig = startDriving();
  for (let i = 0; i < 3; i++) rig.feed(wide, 0.9);
  assert.notEqual(rig.f.state, LineState.LINE_TRACKING, 'a wide bar must not steer by grey');

  const rig2 = startDriving();
  for (let i = 0; i < 3; i++) rig2.feed(multi, 0.9);
  assert.notEqual(rig2.f.state, LineState.LINE_TRACKING, 'a multi-region reading must not steer by grey');

  const rig3 = startDriving();
  for (let i = 0; i < 3; i++) rig3.feed(blank, 0.9);
  assert.equal(rig3.f.state, LineState.LINE_GAP_HOLD, 'a blank bar falls back to the pose hold');
}

console.log('PASS: with a valid pose segment every single-region narrow line (incl. 2-3 edge channels, position +/-5) '
  + 'steers through the grey PID; wide/multi-region readings still fall back to the pose hold');
