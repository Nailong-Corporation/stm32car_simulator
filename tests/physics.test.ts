import assert from 'node:assert/strict';
import { advanceWheel, VehicleSimulator } from '../src/sim/physics';
import { LineFollowModel } from '../src/sim/line_follow';
import { MOTOR_MAX_ACCEL_CM_S2, MOTOR_MAX_DECEL_CM_S2, WHEEL_TRACK_CM } from '../src/sim/config';
import { Action } from '../src/sim/types';

// Electrical stop is not a mechanical stop. A 100 cm/s rolling vehicle
// must continue moving; coast and single-wheel short brake are distinct.
for (const dt of [0.001, 0.005, 0.01]) {
 let coast=100,brake=100,dCoast=0,dBrake=0;
 for(let i=0;i<Math.round(.5/dt);i++){
  const c=advanceWheel(coast,0,false,dt),b=advanceWheel(brake,0,true,dt);
  assert.ok(c>0 && c<=coast && b<=brake);
  assert.ok(coast-c<=MOTOR_MAX_DECEL_CM_S2*dt+1e-9);
  dCoast+=(coast+c)*.5*dt;dBrake+=(brake+b)*.5*dt;
  coast=c;brake=b;
 }
 assert.ok(dCoast>20 && dCoast>dBrake);
 assert.ok(advanceWheel(0,500,true,dt)<=MOTOR_MAX_ACCEL_CM_S2*dt+1e-9);
}

// Use fixed driver outputs to exercise the real encoder + pose integration,
// independent of any route/controller success assumptions.
const f=new LineFollowModel([{junction_count:1,turn_speed:16,action:Action.STOP,cruise_speed:60,angle_deg:0,corner_arc_enabled:false,segment_length_cm:0}]);
const s=new VehicleSimulator(f);
f.update=()=>{};
s.actual_left_speed=s.actual_right_speed=100;
f.command={left_pwm:0,right_pwm:0,left_forward:true,right_forward:true,standby:false};
const start={...s.pose};s.step();
assert.ok(s.actual_left_speed>98 && s.actual_right_speed>98,'stop cannot zero physical velocity');
assert.ok(Math.hypot(s.pose.x-start.x,s.pose.y-start.y)>.49);
assert.ok(s.encoder_counts.left!==0 && s.encoder_counts.right!==0,'coasting still produces encoder pulses');
const before={...s.pose};const lv=s.actual_left_speed,rv=s.actual_right_speed;
f.command={left_pwm:0,right_pwm:20,left_forward:true,right_forward:true,standby:true};s.step();
const expected=((lv+s.actual_left_speed)-(rv+s.actual_right_speed))*.5*.005/WHEEL_TRACK_CM;
assert.ok(Math.abs((s.pose.heading-before.heading)-expected)<1e-12);
assert.ok(Math.hypot(s.pose.x-before.x,s.pose.y-before.y)>.45,'steering keeps forward momentum');
s.reset();assert.equal(s.actual_left_speed,0);assert.equal(s.actual_right_speed,0);
console.log('PASS: finite drive/coast/brake inertia, encoder/pose consistency and moving turn onset');
