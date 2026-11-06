// Debug replay: same plant/controller as replay.mjs, but dumps route internals
// per control cycle so a failing step can be diagnosed frame by frame.
//   node tests/debug_replay.mjs <scheme> <outdir> [max_ms]
import { createServer } from 'vite';
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';

const scheme = Number(process.argv[2] ?? 1);
const out = process.argv[3] ?? '/tmp/stm32-debug';
const maxMs = Number(process.argv[4] ?? 120000);
mkdirSync(out, { recursive: true });

const server = await createServer({
  server: { middlewareMode: true },
  logLevel: 'error',
  optimizeDeps: { noDiscovery: true, include: [] },
});
try {
  const { LineFollowModel } = await server.ssrLoadModule('/src/sim/line_follow.ts');
  const { VehicleSimulator } = await server.ssrLoadModule('/src/sim/physics.ts');
  const { parseRouteTableC } = await server.ssrLoadModule('/src/sim/route_parser.ts');
  const { LineState, RouteState, RouteFailure, HexagonState } = await server.ssrLoadModule('/src/sim/types.ts');
  const source = readFileSync('/Users/zhendong/Documents/PlatformIO/Projects/car_stm32/src/route_table.c', 'utf8');
  const steps = parseRouteTableC(source).schemes.find((s) => s.id === scheme).steps;
  const f = new LineFollowModel(steps);
  const s = new VehicleSimulator(f);
  f.route.repeat_count = 1;
  s.requestStart();

  const keys = [
    'ms', 'index', 'state', 'hex', 'count', 'x', 'y', 'h', 'd', 'mask', 'pos', 'width', 'regions',
    'center', 'lt', 'rt', 'lv', 'rv', 'line',
    'entry', 'actionStart', 'turnAppr', 'turnRad', 'turnRewind', 'shortTrack', 'shortActive',
    'shortFwd', 'shortLat', 'exitCand', 'exitCnt', 'captureSeen', 'captureCnt', 'lineExit',
    'approachCenter', 'recov', 'warn', 'fail',
  ];
  const rows = [];
  let prev = '';
  const events = [];
  for (let i = 0; i < 240000; ++i) {
    s.step();
    const r = f.route;
    const o = f.observation;
    const sign = r.turnTargetRad() < 0 ? -1 : 1;
    const row = {
      ms: s.sim_time_ms,
      index: r.index,
      state: RouteState[r.state],
      hex: HexagonState[r.hexagon.state],
      count: r.junctions_seen,
      x: +s.pose.x.toFixed(2),
      y: +s.pose.y.toFixed(2),
      h: +s.pose.heading.toFixed(3),
      d: +f.odometry.distance_cm.toFixed(2),
      mask: o.line_mask,
      pos: o.position,
      width: o.white_count,
      regions: o.region_count,
      center: r.approach_center_count,
      lt: +f.left_target_cm_s.toFixed(2),
      rt: +f.right_target_cm_s.toFixed(2),
      lv: +s.actual_left_speed.toFixed(1),
      rv: +s.actual_right_speed.toFixed(1),
      line: LineState[f.state],
      entry: +r.entry_heading.toFixed(3),
      actionStart: +r.action_start_heading.toFixed(3),
      turnAppr: r.turn_approaching ? 1 : 0,
      turnRad: +(sign * (s.pose.heading - r.entry_heading)).toFixed(3),
      turnRewind: r.turn_rewind_active ? 1 : 0,
      shortTrack: r.short_link_tracking ? 1 : 0,
      shortActive: r.short_link_active ? 1 : 0,
      shortFwd: +r.short_link_forward_cm.toFixed(1),
      shortLat: +r.short_link_lateral_cm.toFixed(1),
      exitCand: r.exit_candidate_pending ? 1 : 0,
      exitCnt: r.exit_count,
      captureSeen: r.capture_line_seen ? 1 : 0,
      captureCnt: r.capture_line_samples,
      lineExit: r.line_exit_confirmed ? 1 : 0,
      approachCenter: r.have_approach_center ? 1 : 0,
      recov: r.navigation_recoveries,
      warn: r.navigation_warning,
      fail: r.failure,
    };
    rows.push(row);
    const k = [r.index, r.state, r.junctions_seen, r.hexagon.state].join(',');
    if (k !== prev) {
      events.push(row);
      console.log(JSON.stringify(row));
      prev = k;
    }
    if (f.state === LineState.LINE_STOPPED || r.state === RouteState.ROUTE_FINISHED) break;
    if (s.sim_time_ms >= maxMs) break;
  }
  writeFileSync(
    `${out}/scheme${scheme}.csv`,
    keys.join(',') + '\n' + rows.map((r) => keys.map((k) => r[k]).join(',')).join('\n'),
  );
  writeFileSync(`${out}/scheme${scheme}.json`, JSON.stringify({ steps, events, final: rows.at(-1) }, null, 2));
} finally {
  await server.close();
}
