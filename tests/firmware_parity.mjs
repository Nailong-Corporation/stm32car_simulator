import { createServer } from 'vite';
import { spawnSync } from 'node:child_process';
import { readFileSync, writeFileSync, mkdtempSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import assert from 'node:assert/strict';

const firmware = new URL('../../PlatformIO/Projects/car_stm32/', import.meta.url).pathname;
const temp = mkdtempSync(join(tmpdir(), 'car-parity-'));
const server = await createServer({server:{middlewareMode:true},logLevel:'error',optimizeDeps:{noDiscovery:true,include:[]}});
try {
  const {LineFollowModel} = await server.ssrLoadModule('/src/sim/line_follow.ts');
  const {VehicleSimulator} = await server.ssrLoadModule('/src/sim/physics.ts');
  const {parseRouteTableC} = await server.ssrLoadModule('/src/sim/route_parser.ts');
  const {LineState,RouteState} = await server.ssrLoadModule('/src/sim/types.ts');
  const schemes=parseRouteTableC(readFileSync(join(firmware,'src/route_table.c'),'utf8')).schemes;
  for(const scheme of schemes.filter(s=>s.id>0)) {
    const executable=join(temp,`controller-${scheme.id}`);
    const compile=spawnSync('cc',['-std=c11','-Wall','-Wextra','-Werror','-pedantic',
      '-I'+join(firmware,'include'),'-DROUTE_SCHEME_SELECT='+scheme.id,
      ...['test/replay_controller.c','src/route_table.c','src/line_follow.c','src/route.c',
      'src/gray_sensor.c','src/pid.c','src/odometry.c','src/braking_planner.c','src/speed_control.c'].map(p=>join(firmware,p)),
      '-lm','-o',executable],{encoding:'utf8'});
    assert.equal(compile.status,0,compile.stderr);
    const f=new LineFollowModel(scheme.steps), sim=new VehicleSimulator(f);
    f.route.repeat_count=1;
    const inputs=[], expected=[];
    const original=f.update.bind(f);
    f.update=(o,c,now,dt,start,valid,heading)=>{
      inputs.push([now,dt,o.kind,o.raw_mask,o.line_mask,o.selected_mask,o.white_count,o.region_count,
        o.position,o.prefilter_mask??o.line_mask,o.seam_mask??0,c.left,c.right,+start,+valid,heading].join(' '));
      original(o,c,now,dt,start,valid,heading);
      expected.push([f.route.index,f.route.state,f.route.junctions_seen,f.route.failure,f.state,
        f.left_target_cm_s,f.right_target_cm_s,f.odometry.distance_cm,f.odometry.heading_rad,
        f.route.entry_heading]);
    };
    sim.requestStart();
    for(let tick=0;tick<240000;tick++) {
      sim.step();
      if(f.state===LineState.LINE_STOPPED||f.route.state===RouteState.ROUTE_FINISHED) break;
    }
    if(process.env.PARITY_DUMP) {
      writeFileSync(`/tmp/pd-${scheme.id}.in`,inputs.join('\n')+'\n');
      writeFileSync(`/tmp/pd-${scheme.id}.exp`,expected.map(r=>r.join(' ')).join('\n')+'\n');
    }
    const result=spawnSync(executable,[],{input:inputs.join('\n')+'\n',encoding:'utf8',maxBuffer:64*1024*1024});
    if(process.env.PARITY_DUMP) writeFileSync(`/tmp/pd-${scheme.id}.act`,result.stdout);
    assert.equal(result.status,0,result.stderr);
    const actual=result.stdout.trim().split('\n').map(row=>row.split(' ').map(Number));
    assert.equal(actual.length,expected.length);
    /* Discrete FSM parity: the sequence of (index, route state, junction count,
     * failure, line state) events must be identical, and every event must land
     * within one control period of the simulator's.  A continuous-state
     * comparison alone would flag a benign one-frame threshold crossing (the C
     * odometry is 32-bit float, so a segment-length edge can flip one tick). */
    const events=rows=>{
      const out=[];let prev=null;
      for(const r of rows){const key=r.slice(0,5).join(',');if(key!==prev){out.push({key,t:r[0]});prev=key;}}
      return out;
    };
    const evA=events(actual),evE=events(expected);
    assert.equal(evA.length,evE.length,
      `scheme ${scheme.id}: ${evA.length} vs ${evE.length} FSM events`);
    for(let i=0;i<evA.length;i++) {
      assert.equal(evA[i].key,evE[i].key,
        `scheme ${scheme.id} event ${i} at ${evE[i].t}ms: C=${evA[i].key}, TS=${evE[i].key}`);
      const skew=Math.abs(evA[i].t-evE[i].t);
      assert.ok(skew<=10,
        `scheme ${scheme.id} event ${i} (${evE[i].key}) at ${evE[i].t}ms: C is ${skew}ms off`);
    }
    let maxSpeedError=0,totalSpeedError=0,outlierFrames=0,comparedFrames=0;
    for(let i=0;i<actual.length;i++) {
      if(actual[i].slice(0,5).join(',')!==expected[i].slice(0,5).join(',')) continue;
      ++comparedFrames;
      for(let field=5;field<7;field++) {
        const error=Math.abs(actual[i][field]-expected[i][field]);
        maxSpeedError=Math.max(error,maxSpeedError);
        totalSpeedError+=error;
        // One frame of the startup ramp (160 cm/s^2 * 5 ms = 0.8 cm/s) may
        // differ where the two sides sample the pose-gap pre-slow boundary.
        if(error>=1.05) {
          ++outlierFrames;
          // The steering-feedback clamp (STEERING_FEEDBACK_MIN_HALF_CM_S) is a
          // discontinuous branch: a 1e-3 rad difference at the threshold can
          // hold one inner wheel one frame. The 0.15 rad bang-bang/settling and
          // the steering-feedback clamp are both thresholds; bound how often a
          // single frame may straddle one of them.
          assert.ok(error<18,
            `scheme ${scheme.id} ${i*5+5}ms target error ${error}: C=${actual[i]}, TS=${expected[i]}`);
        }
      }
      // The C odometry runs in 32-bit float while the simulator uses doubles;
      // over a 2.5 km lap the divergence stays below 0.5 cm (2e-4 relative).
      assert.ok(Math.abs(actual[i][7]-expected[i][7])<0.5,`scheme ${scheme.id} odometry drift at ${i}`);
      assert.ok(Math.abs(actual[i][8]-expected[i][8])<0.02,`scheme ${scheme.id} heading drift at ${i}`);
    }
    const meanSpeedError=totalSpeedError/Math.max(comparedFrames*2,1);
    assert.ok(outlierFrames<=12,
      `scheme ${scheme.id}: ${outlierFrames} frames diverge by >=1.05 cm/s (limit 12)`);
    assert.ok(meanSpeedError<0.6,
      `scheme ${scheme.id}: mean target error ${meanSpeedError.toFixed(4)} cm/s exceeds 0.6`);
    console.log(`PASS: scheme ${scheme.id}, ${actual.length} identical-input C/TS frames, max target error ${maxSpeedError.toFixed(4)} cm/s (${outlierFrames} threshold frames), mean ${meanSpeedError.toFixed(4)}; final index ${f.route.index}`);
  }
} finally { await server.close(); rmSync(temp,{recursive:true,force:true}); }
