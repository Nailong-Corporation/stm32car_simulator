import assert from 'node:assert/strict';
import { RoutePlannerModel } from '../src/sim/route';
import { grayClassify } from '../src/sim/gray_sensor';
import { Action, RouteState } from '../src/sim/types';

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
assert.equal(p.targetReady(104.99, true), false);
assert.equal(p.targetReady(105, true), true);
d = 105; samples(0x18,8); samples(0,1);
assert.equal(p.state, RouteState.ROUTE_FOLLOW);
assert.equal(p.junctions_seen, 0);
samples(0x18,4); samples(0xff,3);
assert.equal(p.state, RouteState.ROUTE_EXECUTING);
assert.equal(p.junctions_seen,1);
reset(); p.junctions_seen=3;
assert.equal(p.targetReady(10,true),true);
assert.equal(p.targetReady(10,false),false);
d=10; samples(0x18,4); samples(0xff,3);
assert.equal(p.state,RouteState.ROUTE_EXECUTING);
assert.equal(p.junctions_seen,4);
reset(); samples(0x18,4); samples(0x3c,1); samples(0x24,5);
assert.equal(p.junctions_seen,1);
assert.equal(p.state,RouteState.ROUTE_CLEARING);
samples(0x24,8); assert.equal(p.junctions_seen,1);
console.log('PASS: distance OR maximum count, confirmed node required, resistor counted once');
