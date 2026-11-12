import assert from 'node:assert/strict';
import { RoutePlannerModel } from '../src/sim/route';
import { parseRouteTableC, DEFAULT_ROUTE_TABLE_C } from '../src/sim/route_parser';
import { grayClassify } from '../src/sim/gray_sensor';
import { TURN_PIVOT_APPROACH_CM } from '../src/sim/config';
import { Action, RouteState, type RouteStep } from '../src/sim/types';

// Numeric macros in the ninth column must resolve, while old tables retain 0.
{
  const parsed = parseRouteTableC([
    '#define TURN_RIGHT_ANGLE_STRAIGHT_CM 4.0f',
    '#define TURN_TRANSISTOR_STRAIGHT_CM 6.0f /* absolute distance */',
    'const RouteStep route[] = {',
    '{1,20,CUSTOM_TURN,50,90,false,50,0,TURN_RIGHT_ANGLE_STRAIGHT_CM},',
    '{1,20,CUSTOM_TURN,50,-90,false,50,0,TURN_TRANSISTOR_STRAIGHT_CM},',
    '{1,20,CUSTOM_TURN,50,90,false,50,0,4.0f},',
    '{1,20,CUSTOM_TURN,50,90,false,50,0},',
    '{1,20,STOP,50,0,false,40,0,0}',
    '};',
  ].join('\n'));
  assert.deepEqual(parsed.activeSteps.map(s => s.turn_approach_cm), [4, 6, 4, 0, 0]);
  assert.ok(parsed.activeSteps.every(s => s.short_link_cm === 0));
  console.log('PASS: parser resolves absolute approach macros and preserves missing-column defaults');
}

function steps(approach?: number, sign = 1): RouteStep[] {
  return [
    { junction_count: 2, turn_speed: 20, action: Action.CUSTOM_TURN, cruise_speed: 80,
      angle_deg: 90 * sign, corner_arc_enabled: false, segment_length_cm: 50, short_link_cm: 0,
      turn_approach_cm: approach },
    { junction_count: 3, turn_speed: 20, action: Action.STOP, cruise_speed: 80,
      angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 40, short_link_cm: 0,
      turn_approach_cm: 0 },
  ];
}

// Exercise real grey confirmation, optional entry braking, then straight approach.
function pivotStart(approach?: number, late = 0, brakingCm = 0, sign = 1) {
  const r = new RoutePlannerModel();
  r.init(steps(approach, sign));
  assert.ok(r.valid);
  r.start(0, 0);
  r.setSpeed(brakingCm > 0 ? 60 : 10, brakingCm > 0 ? 60 : 10);
  r.pose_trigger_late_cm = late;
  let d = 0, now = 0, confirmed = -1, brakingEnd = -1;
  const increment = 0.1;
  for (let i = 0; i < 1200; i++) {
    d += increment; now += 5;
    const mask = d >= 50 && d < 53 ? 0x1f : 0x18;
    if (r.state === RouteState.ROUTE_BRAKING && d - confirmed >= brakingCm) {
      r.setSpeed(10, 10);
      brakingEnd = d;
    }
    r.update(grayClassify(mask, true, 0), d, 0, now, 0, 110 - d);
    if (confirmed < 0 && (r.state === RouteState.ROUTE_BRAKING ||
      r.state === RouteState.ROUTE_EXECUTING)) {
      confirmed = r.state === RouteState.ROUTE_BRAKING ? r.entry_cm : r.turn_approach_origin_cm;
    }
    if (r.state === RouteState.ROUTE_EXECUTING && !r.turn_approaching) {
      return { travel: r.entry_cm - confirmed, confirmed, brakingEnd,
        origin: r.turn_approach_origin_cm, configured: r.turn_approach_cm };
    }
  }
  assert.fail('confirmed junction never reached the pivot');
}

for (const sign of [-1, 1]) {
  for (const approach of [4, 6]) {
    for (const late of [0, 2.5, 12]) {
      const pivot = pivotStart(approach, late, 0, sign);
      assert.equal(pivot.configured, approach, 'pose lateness cannot shorten an explicit distance');
      assert.ok(Math.abs(pivot.travel - approach) <= 0.11,
        'explicit approach must be absolute ' + approach + ' cm, got ' + pivot.travel);
    }
    for (const braking of [2, 8]) {
      const pivot = pivotStart(approach, 2.5, braking, sign);
      assert.equal(pivot.origin, pivot.confirmed, 'braking must count from junction confirmation');
      assert.ok(Math.abs(pivot.travel - Math.max(approach, pivot.brakingEnd - pivot.confirmed)) <= 0.11,
        'braking distance must count toward the absolute approach, got ' + pivot.travel);
    }
  }
}
console.log('PASS: absolute 4/6 cm approaches, mirrored turns, pose lateness, and braking travel');

for (const approach of [undefined, 0]) {
  for (const late of [0, 2.5, TURN_PIVOT_APPROACH_CM + 1]) {
    const pivot = pivotStart(approach, late);
    const expected = Math.max(0, TURN_PIVOT_APPROACH_CM - late);
    assert.equal(pivot.configured, expected);
    assert.ok(Math.abs(pivot.travel - expected) <= 0.11, 'legacy approach still shortens for pose lateness');
  }
  const pivot = pivotStart(approach, 2.5, 2);
  assert.equal(pivot.origin, pivot.brakingEnd, 'legacy approach still starts after braking');
  assert.ok(Math.abs(pivot.travel - (pivot.brakingEnd - pivot.confirmed + TURN_PIVOT_APPROACH_CM - 2.5)) <= 0.11);
}
console.log('PASS: zero/omitted approach retains legacy distance, pose shortening and braking origin');

// Short-link turn-in still follows projected corner geometry, regardless of
// the ordinary approach distance on its row.
for (const approach of [0, 4, 6]) {
  const rows = steps(4);
  rows.splice(1, 0, { ...rows[0], junction_count: 1, segment_length_cm: 25,
    short_link_cm: 30, turn_approach_cm: approach });
  const r = new RoutePlannerModel();
  r.init(rows);
  r.pose_enabled = false;
  r.start(0, 0);
  r.setSpeed(10, 10);
  let d = 0, now = 0;
  const sample = (mask: number, ds: number, h: number) => {
    d += ds; now += 5;
    r.update(grayClassify(mask, true, 0), d, h, now);
  };
  for (let i = 0; i < 500; i++) sample(0x18, 0.1, 0);
  for (let i = 0; i < 4; i++) sample(0x1f, 0.1, 0);
  assert.equal(r.turn_approaching, true);
  for (let i = 0; i < 41; i++) sample(0, 0.1, 0);
  const radius = 7;
  for (let i = 1; i <= 100; i++) {
    sample(i >= 98 ? 0x18 : 0, radius * Math.PI / 200, Math.PI * i / 200);
  }
  for (let i = 0; i < 3; i++) sample(0x18, 0, Math.PI / 2);
  assert.equal(r.index, 1);
  assert.equal(r.short_link_active, true);
  for (let i = 0; i < 250 && r.turn_approaching; i++) sample(0, 0.1, Math.PI / 2);
  assert.equal(r.turn_approaching, false);
  const expected = 30 - r.short_link_pivot_lead_cm;
  assert.ok(r.short_link_forward_cm >= expected - 1e-6 && r.short_link_forward_cm < expected + 0.11,
    '30 cm short link must preserve its projected turn-in geometry: approach=' + approach +
    ', forward=' + r.short_link_forward_cm + ', state=' + r.state);
}
console.log('PASS: short-link projected turn-in geometry stays unchanged for 0/4/6 cm row approaches');

// The bundled table keeps every existing route value and only changes column 9.
{
  const parsed = parseRouteTableC(DEFAULT_ROUTE_TABLE_C);
  assert.equal(parsed.activeSchemeId, 4); /* firmware ROUTE_SCHEME_SELECT=4 */
  assert.equal(parsed.schemes.length, 9); /* 0..8: firmware 2e751de adds the scheme-8 ring test */
  for (const scheme of parsed.schemes) {
    for (const s of scheme.steps) {
      assert.equal(s.corner_arc_enabled, false);
      const ordinary = s.action === Action.LEFT || s.action === Action.RIGHT ||
        (s.action === Action.CUSTOM_TURN && Math.abs(s.angle_deg) === 90);
      const approach = s.turn_approach_cm ?? 0;
      assert.ok(ordinary ? [9, 14, 19].includes(approach) : approach === 0,
        `ordinary right angles use 9/14/19 cm (Q6 calibration +5); other actions use zero, got ${approach}`);
    }
  }
  const s1 = parsed.schemes.find(s => s.id === 1)!;
  assert.equal(s1.steps.length, 30);
  const transistorRows = s1.steps.flatMap((s, i) => (s.turn_approach_cm ?? 0) >= 14 ? [i] : []);
  assert.deepEqual(transistorRows, [3, 9, 15, 17, 18, 25]);
  console.log('PASS: bundled schemes use absolute 9/14/19 cm and preserve the six scheme-1 transistor corners');
}
