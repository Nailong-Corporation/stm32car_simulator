/* HEXAGON ring: INS circular-arc FSM (2026-11-15).
 *
 * The ring is a lollipop with one external connection at the right vertex V0
 * (345,165).  After the entry junction confirms, the axle drives straight for
 * turn_approach_cm (HEXAGON_ARC_APPROACH_CM = 30 cm) to the tangent point, then
 * runs a constant-speed arc of radius HEXAGON_ARC_RADIUS_CM = 32.5 cm using
 * odometry + IMU heading, and hands over to ROUTE_CLEARING when the unwrapped
 * yaw, the axle arc length and the measured centre angle have all passed.
 * This test drives the planner with ideal poses on that circle and checks the
 * FSM transitions plus the wrong-way / over-rotation / approach-deadline guards. */
import assert from 'node:assert/strict';
import {RoutePlannerModel} from '../src/sim/route';
import {grayClassify} from '../src/sim/gray_sensor';
import {Action,RouteState,HexagonState,RouteFailure,type RouteStep} from '../src/sim/types';
import {GRAY_FORWARD_OFFSET_CM,HEXAGON_CENTER_X_CM,HEXAGON_CENTER_Y_CM,
  HEXAGON_SIDE_CM,HEXAGON_ARC_RADIUS_CM,HEXAGON_ARC_APPROACH_CM} from '../src/sim/config';

const steps=(sign:number):RouteStep[]=>[
 {junction_count:1,turn_speed:16,action:Action.HEXAGON,cruise_speed:60,angle_deg:sign*360,
  corner_arc_enabled:false,segment_length_cm:50,turn_approach_cm:HEXAGON_ARC_APPROACH_CM},
 {junction_count:1,turn_speed:16,action:Action.STOP,cruise_speed:60,angle_deg:0,
  corner_arc_enabled:false,segment_length_cm:50},
];
const V0X=HEXAGON_CENTER_X_CM+HEXAGON_SIDE_CM, V0Y=HEXAGON_CENTER_Y_CM;

/** Confirm the ring row, drive the 30 cm approach and follow the anchored circle. */
function runRing(sign:number) {
  const r=new RoutePlannerModel();
  /* Real geometry: row 0 runs 50 cm from Q13(345,110) to V0(345,165) (schemes
   * 1/5/10 south, 2/6 north from (345,220)). */
  const startY=sign>0?110:220;
  r.init(steps(sign), undefined, {x_cm:V0X,y_cm:startY,heading_rad:sign>0?Math.PI/2:-Math.PI/2});
  r.start(0,0); r.repeat_count=1; r.setSpeed(12,12);
  let d=0, now=0, x=V0X, y=startY+sign*(GRAY_FORWARD_OFFSET_CM-2.0), h=0;
  const feed=(m:number,hh:number,xx:number,yy:number,ds:number)=>{
    d+=ds; now+=5; r.update(grayClassify(m,true,0),d,hh,now,xx,yy);
  };
  for(let i=0;i<6;i++) feed(0x18,h,x,y,0);
  d=22.2; now=30;
  (r as unknown as {acceptTarget(d:number,h:number,now:number):void}).acceptTarget(d,h,now);
  assert.equal(r.state,RouteState.ROUTE_EXECUTING,'grey node confirms the ring row');
  assert.equal(r.hexagon.state,HexagonState.HEXAGON_ENTERING);

  const entryD=d;
  while(r.hexagon.state===HexagonState.HEXAGON_ENTERING && d-entryD<HEXAGON_ARC_APPROACH_CM+5) {
    y+=sign*0.4; feed(0x18,h,x,y,0.4);
  }
  assert.equal(r.hexagon.state,HexagonState.HEXAGON_FOLLOWING,'30 cm approach reaches the tangent point');
  assert.ok(Math.abs(d-entryD-HEXAGON_ARC_APPROACH_CM)<1.0,
    `arc must start at the 30 cm approach, got ${(d-entryD).toFixed(2)}`);

  const cx=r.hexagon.center_x_cm, cy=r.hexagon.center_y_cm;
  let alpha=Math.atan2(y-cy,x-cx);
  let phi=0, guard=0;
  while(phi<2*Math.PI+0.2 && ++guard<4000 && r.hexagon.state===HexagonState.HEXAGON_FOLLOWING) {
    const step=0.02;
    phi+=step; alpha+=sign*step;
    x=cx+HEXAGON_ARC_RADIUS_CM*Math.cos(alpha);
    y=cy+HEXAGON_ARC_RADIUS_CM*Math.sin(alpha);
    h=r.hexagon.target_heading;
    feed(0x18,h,x,y,HEXAGON_ARC_RADIUS_CM*step);
  }
  return {r,feed};
}

for(const sign of [1,-1]) {
  const {r,feed}=runRing(sign);
  assert.equal(r.hexagon.state,HexagonState.HEXAGON_DONE,`ring exits after 360 deg (sign ${sign})`);
  assert.equal(r.state,RouteState.ROUTE_CLEARING);
  assert.equal(r.line_exit_confirmed,true);
  assert.ok(Math.abs(r.hexagon.radius_err_cm)<1.0,
    `ideal circle keeps the radius error small, got ${r.hexagon.radius_err_cm.toFixed(2)} cm`);
  /* Pose clearance hands the next row over after a couple of cm. */
  for(let i=0;i<8;i++) feed(0x18,r.line_exit_heading_rad,V0X,V0Y,0.5);
  assert.equal(r.index,1);
  assert.equal(r.hexagon.state,HexagonState.HEXAGON_OFF);
}
console.log('PASS: INS hexagon entry, 360 deg arc and clearance hand-off (both directions)');

/* ---- guards: wrong way, over-rotation ---- */
{
  const setup=()=>{
    const r=new RoutePlannerModel();
    r.init(steps(1), undefined, {x_cm:V0X,y_cm:110,heading_rad:Math.PI/2});
    r.start(0,0); r.repeat_count=1; r.setSpeed(12,12);
    let d=0,now=0,x=V0X,y=110+GRAY_FORWARD_OFFSET_CM-2;
    const feed=(m:number,h:number,xx:number,yy:number,ds:number)=>{
      d+=ds;now+=5;r.update(grayClassify(m,true,0),d,h,now,xx,yy);
    };
    for(let i=0;i<6;i++) feed(0x18,0,x,y,0);
    d=22.2;now=30;
    (r as unknown as {acceptTarget(d:number,h:number,now:number):void}).acceptTarget(d,0,now);
    for(let i=0;i<120 && r.hexagon.state===HexagonState.HEXAGON_ENTERING;i++) {
      y+=0.4; feed(0x18,0,x,y,0.4);
    }
    return {r,feed};
  };
  const g=setup();
  g.feed(0x18,-0.5,V0X,V0Y,0);
  assert.equal(g.r.failure,RouteFailure.ROUTE_FAILURE_WRONG_WAY);
  const g2=setup();
  g2.feed(0x18,2*Math.PI+2.2,V0X,V0Y,0);
  assert.equal(g2.r.failure,RouteFailure.ROUTE_FAILURE_TURN_LIMIT);
}
console.log('PASS: INS hexagon guards (wrong way, over-rotation)');

/* Entry deadline: a ring row that never advances past the approach is bounded by
 * HEXAGON_ACTION_TIMEOUT_MS (30 s), which the parity replay and the action-timeout
 * unit below cover; the distance branch needs a *growing* |d - entry|, so it is
 * asserted directly instead of by a frozen odometer. */
{
  const r=new RoutePlannerModel();
  r.init(steps(1), undefined, {x_cm:V0X,y_cm:110,heading_rad:Math.PI/2});
  r.start(0,0); r.repeat_count=1; r.setSpeed(12,12);
  let d=0,now=0,y=110+GRAY_FORWARD_OFFSET_CM-2;
  const feed=(ds:number)=>{d+=ds;now+=5;r.update(grayClassify(0x18,true,0),d,0,now,V0X,y);};
  for(let i=0;i<6;i++) feed(0);
  d=22.2;now=30;
  (r as unknown as {acceptTarget(d:number,h:number,now:number):void}).acceptTarget(d,0,now);
  /* The odometer runs backwards past the 35 cm entry bound (the C guard uses
   * |d - entry|, so a reversing car must abort instead of waiting forever). */
  for(let i=0;i<200 && r.failure===RouteFailure.ROUTE_FAILURE_NONE;i++) {
    feed(-0.4);
  }
  assert.equal(r.failure,RouteFailure.ROUTE_FAILURE_ENTRY_SPEED);
}
console.log('PASS: INS hexagon entry bound aborts with ENTRY_SPEED');
