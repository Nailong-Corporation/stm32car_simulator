import assert from 'node:assert/strict';
import { RoutePlannerModel } from '../src/sim/route';
import { grayClassify } from '../src/sim/gray_sensor';
import { Action, RouteState } from '../src/sim/types';
import { ROUTE_GATE_LOOKAHEAD_CM } from '../src/sim/config';

const steps = [
  { junction_count: 4, turn_speed: 12, action: Action.LEFT, cruise_speed: 30,
    angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 105 },
  { junction_count: 1, turn_speed: 12, action: Action.STOP, cruise_speed: 30,
    angle_deg: 0, corner_arc_enabled: false, segment_length_cm: 0 },
];
let p: RoutePlannerModel;
let d = 0, t = 0;
function reset() {
  p = new RoutePlannerModel(); p.init(steps); p.repeat_count = 1; p.start(0,0);
  assert.equal(p.valid, true); d = 0; t = 0;
}
function samples(mask: number, n: number) {
  for (let i = 0; i < n; i++) {
    d += .06; t += 5; p.update(grayClassify(mask, true, 0), d, 0, t);
  }
}
reset();
// 2026-11-10: the gate is measured at the sensor bar (24 cm ahead of the axle).
assert.equal(p.targetReady(105 - ROUTE_GATE_LOOKAHEAD_CM - 0.01, true), false);
assert.equal(p.targetReady(105 - ROUTE_GATE_LOOKAHEAD_CM, true), true);
d = 105; samples(0x18,8); samples(0,1);
assert.equal(p.state, RouteState.ROUTE_FOLLOW);
assert.equal(p.junctions_seen, 0);
samples(0x18,4); samples(0xff,3);
assert.equal(p.state, RouteState.ROUTE_EXECUTING);
assert.equal(p.junctions_seen,1);
reset(); p.junctions_seen=3;
assert.equal(p.targetReady(10,true),false);
assert.equal(p.targetReady(10,false),false);
d=10; samples(0x18,4); samples(0xff,3);
assert.equal(p.state,RouteState.ROUTE_CLEARING);
assert.equal(p.passing_junction,true);
assert.equal(p.junctions_seen,4);
// Passing nodes do not reset the segment origin. Once distance is satisfied,
// the next confirmed node authorizes the action even if count is insufficient.
samples(0x18,40);
assert.equal(p.state,RouteState.ROUTE_FOLLOW);
assert.equal(p.origin_cm,0);
p.junctions_seen=0; d=105;
samples(0x18,4); samples(0xff,3);
assert.equal(p.state,RouteState.ROUTE_EXECUTING);
assert.equal(p.passing_junction,false);
assert.equal(p.junctions_seen,1);
reset(); p.steps = p.steps.map(s => ({...s})); p.steps[0].segment_length_cm=0;
p.junctions_seen=2;
assert.equal(p.targetReady(1000,true),false);
samples(0x18,4); samples(0xff,3);
assert.equal(p.state,RouteState.ROUTE_CLEARING);
samples(0x18,40); samples(0xff,3);
assert.equal(p.state,RouteState.ROUTE_EXECUTING);
assert.equal(p.junctions_seen,4);
reset(); p.origin_cm=20; p.junctions_seen=99;
assert.equal(p.targetReady(20 + 105 - ROUTE_GATE_LOOKAHEAD_CM - 0.01,true),false);
assert.equal(p.targetReady(20 + 105 - ROUTE_GATE_LOOKAHEAD_CM,false),true);
reset(); samples(0x18,4); samples(0x3c,1); samples(0x24,5);
assert.equal(p.junctions_seen,1);
assert.equal(p.state,RouteState.ROUTE_CLEARING);
samples(0x24,8); assert.equal(p.junctions_seen,1);
console.log('PASS: positive distance gates confirmed nodes before count; zero distance uses count; passing nodes preserve origin and resistor deduplication');
