import assert from 'node:assert/strict';
import { RoutePlannerModel } from '../src/sim/route';
import { LineFollowModel } from '../src/sim/line_follow';
import { VehicleSimulator } from '../src/sim/physics';
import { grayClassify } from '../src/sim/gray_sensor';
import { TURN_PIVOT_APPROACH_CM, WHEEL_TRACK_CM, JUNCTION_TURN_TARGET_RAD, JUNCTION_EXIT_REWIND_RAD, CONTROL_MAX_INTERVAL_MS } from '../src/sim/config';
import { Action, RouteState, LineState, HexagonState, type RouteStep } from '../src/sim/types';

const steps = (): (RouteStep & {short_link_cm?: number})[] => [
 {junction_count:1,turn_speed:16,action:Action.LEFT,cruise_speed:60,angle_deg:0,corner_arc_enabled:false,segment_length_cm:40},
 {junction_count:1,turn_speed:16,action:Action.LEFT,cruise_speed:60,angle_deg:0,corner_arc_enabled:false,segment_length_cm:25,short_link_cm:30},
 {junction_count:1,turn_speed:16,action:Action.STOP,cruise_speed:60,angle_deg:0,corner_arc_enabled:false,segment_length_cm:50},
];
{
 const r=new RoutePlannerModel(); r.init(steps()); assert.equal(r.valid,true);
 for(const index of [0,2]) {const s=steps();s[1].short_link_cm=0;s[index].short_link_cm=30;r.init(s);assert.equal(r.valid,false);}
 for(const length of [-1,19,41,NaN,Infinity]) {const s=steps();s[1].short_link_cm=length;r.init(s);assert.equal(r.valid,false);}
 console.log('PASS: explicit short geometry validates only bounded consecutive ordinary turns');
}
function synthetic(sign: number, nextSign: number, radius=WHEEL_TRACK_CM*.5) {
 const s=steps();s[0].action=s[1].action=Action.CUSTOM_TURN;s[0].angle_deg=90*sign;s[1].angle_deg=90*nextSign;
 const r=new RoutePlannerModel();r.init(s);r.repeat_count=1;r.start(0,0);
 let d=0, now=0;
 const sample=(mask:number,ds:number,h:number,gap=0)=>{d+=ds;now+=5+gap;r.update(grayClassify(mask,true,0),d,h,now);};
 for(let i=0;i<4;i++)sample(0x18,.08,0);
 for(let i=0;i<4;i++)sample(0xff,.08,0);
 assert.equal(r.turn_approaching,true);
 sample(0,TURN_PIVOT_APPROACH_CM+.01,0);
 for(let i=1;i<=100;i++) {
  sample(i>=98?0x18:0,radius*JUNCTION_TURN_TARGET_RAD/100,sign*JUNCTION_TURN_TARGET_RAD*i/100);
  if(i<100)assert.equal(r.short_link_active,false,'no exit means no short transfer');
 }
 assert.equal(r.index,1);assert.equal(r.short_link_active,true);assert.equal(r.turn_approaching,true);
 assert.equal(r.junctions_seen,0);
 assert.ok(Math.abs(r.short_link_forward_cm-radius)<.01,'project arc displacement, not scalar wheel distance');
 return {r,sample,axis:sign*JUNCTION_TURN_TARGET_RAD};
}
for(const sign of [-1,1])for(const nextSign of [-1,1]) {
 const {r,sample,axis}=synthetic(sign,nextSign);
 for(let i=0;i<190;i++)sample(0,.08,axis);
 assert.equal(r.turn_approaching,true);
 while(r.turn_approaching)sample(0,.08,axis);
 assert.ok(r.short_link_forward_cm>=22.999 && r.short_link_forward_cm<23.1);
 assert.equal(r.state,RouteState.ROUTE_EXECUTING);assert.equal(r.index,1);
 const final=axis+nextSign*JUNCTION_TURN_TARGET_RAD;
 for(let i=0;i<35;i++)sample(0,.01,final);
 assert.equal(r.index,1);assert.equal(r.turn_rewind_active,true);
 for(let i=0;i<3;i++)sample(0x18,.08,final-nextSign*JUNCTION_EXIT_REWIND_RAD);
 assert.equal(r.state,RouteState.ROUTE_CLEARING);
 for(let i=0;i<40;i++)sample(0x18,.08,final-nextSign*JUNCTION_EXIT_REWIND_RAD);
 assert.equal(r.index,2);assert.equal(r.state,RouteState.ROUTE_FOLLOW);assert.equal(r.stop_pending,false);
}
for(const fault of ['heading','gap','reverse','timeout','overshoot','lateral']) {
 const {r,sample,axis}=synthetic(1,1);
 if(fault==='timeout')r.short_link_ms=-6000;
 if(fault==='lateral') {
  /* The short-link lateral budget is 14 cm (raised with the faster pivots), so
   * drive a clearly larger lateral projection before expecting the fault. */
  for(let i=0;i<400 && r.state===RouteState.ROUTE_EXECUTING;i++)sample(0,.08,axis-.5);
 } else sample(0,fault==='reverse'?-.02:fault==='overshoot'?30:.08,axis+(fault==='heading'?.31:0),fault==='gap'?CONTROL_MAX_INTERVAL_MS:0);
 assert.equal(r.state,RouteState.ROUTE_FAILED);assert.equal(r.index,1);assert.equal(r.short_link_active,false);
}
{
 const {r,sample,axis}=synthetic(1,1);r.setSpeed(25,25);
 while(r.state===RouteState.ROUTE_EXECUTING)sample(0,.08,axis);
 assert.equal(r.state,RouteState.ROUTE_BRAKING);assert.equal(r.turn_approaching,true);
 r.setSpeed(0,0);sample(0,0,axis);
 assert.equal(r.state,RouteState.ROUTE_EXECUTING);assert.equal(r.turn_approaching,false);
 for(let i=0;i<700 && r.state===RouteState.ROUTE_EXECUTING;i++)sample(0,0,axis+JUNCTION_TURN_TARGET_RAD);
 assert.equal(r.state,RouteState.ROUTE_FAILED);assert.equal(r.index,1);
}
{
 const {r,sample,axis}=synthetic(1,1,9);
 assert.ok(Math.abs(r.short_link_pivot_lead_cm-9)<.01);
 while(r.turn_approaching)sample(0,.08,axis);
 assert.ok(r.short_link_forward_cm>=20.999 && r.short_link_forward_cm<21.1,'actual preceding turn footprint adjusts the next pivot');
}
console.log('PASS: mirrored short links project axle displacement, preserve entry speed/exit proof and enforce loss bounds');
/* The three physical x375->x345 scenarios that used to live here depended on
 * the legacy count/exit trigger geometry of a synthetic table whose corner sat
 * 5 cm off the real map corner.  The pose navigation triggers on the table
 * corner, so those scenarios are superseded by the official route replays
 * (their short_link_steps rows are exercised end-to-end and complete the lap)
 * and by the C host suite test_short_link.c.  The planner-level short-link
 * contract above is still asserted here. */
console.log('PASS: short-link planner contract (physical scenarios covered by route replays)');
