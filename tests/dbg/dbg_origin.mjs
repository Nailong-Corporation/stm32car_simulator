import { createServer } from 'vite';
const server = await createServer({server:{middlewareMode:true},logLevel:'error',optimizeDeps:{noDiscovery:true,include:[]}});
try{
 const {LineFollowModel}=await server.ssrLoadModule('/src/sim/line_follow.ts');
 const {VehicleSimulator}=await server.ssrLoadModule('/src/sim/physics.ts');
 const {parseRouteTableC}=await server.ssrLoadModule('/src/sim/route_parser.ts');
 const {LineState}=await server.ssrLoadModule('/src/sim/types.ts');
 const {readFileSync}=await import('node:fs');
 const src=readFileSync('/Users/zhendong/Documents/PlatformIO/Projects/car_stm32/src/route_table.c','utf8');
 const steps=parseRouteTableC(src).schemes.find(s=>s.id===2).steps;
 const f=new LineFollowModel(steps),s=new VehicleSimulator(f);
 f.route.repeat_count=1;s.requestStart();
 let prev=-1,prevOrigin=null;
 for(let i=0;i<240000;i++){
  s.step();
  const r=f.route;
  if(r.index!==prev||r.origin_cm!==prevOrigin){
   console.log(s.sim_time_ms,'idx',r.index,r.state,'d=%.1f'%f.odometry.distance_cm,'origin=%.1f'%r.origin_cm,'len',steps[r.index]?.segment_length_cm,'count',r.junctions_seen,'ready',r.targetReady(f.odometry.distance_cm,false));
   prev=r.index;prevOrigin=r.origin_cm;
  }
  if(f.state===LineState.LINE_STOPPED||r.state===5)break;
 }
}finally{await server.close();}
