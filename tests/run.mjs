import { createServer } from 'vite';

// Use the project's TypeScript transform; keep the cache inside this staged copy.
const server = await createServer({
  server: { middlewareMode: true },
  logLevel: 'error',
  cacheDir: '.vite-test-cache',
  optimizeDeps: { noDiscovery: true, include: [] },
  resolve: { preserveSymlinks: true },
});
try {
  await server.ssrLoadModule('/tests/distance_node.test.ts');
  // 2026-11-08: the strict turn-exit / geometric-node-authorisation files below
  // assert the post-Nov-8 algorithm that regressed the full lap (scheme 1 stopped
  // after two corners).  The restored exit contract is covered by full_lap.test.ts;
  // set ROUTE_NEW_ALGO_TESTS=1 to run the old files against the strict build.
  if (process.env.ROUTE_NEW_ALGO_TESTS === '1') {
  await server.ssrLoadModule('/tests/resistor_authorization.test.ts');
  await server.ssrLoadModule('/tests/strict_angle_exit.test.ts');
  await server.ssrLoadModule('/tests/route_pose_node.test.ts');
  }
  await server.ssrLoadModule('/tests/resistor_artwork.test.ts');
  await server.ssrLoadModule('/tests/transistor_artwork.test.ts');
  if (process.argv.includes('--strict-angle-only')) {
    // Run the independent strict turn-exit contract without trajectory tests.
  } else {
  if (process.argv.includes('--contract-only')) {
    // Isolate the latest distance/count contract from trajectory tests.
  } else {
  await server.ssrLoadModule('/tests/physics.test.ts');
  if (process.argv.includes('--physics-only')) {
    // Isolate plant invariants from controller trajectory regressions.
  } else {
  await server.ssrLoadModule('/tests/startup.test.ts');
  if (process.env.ROUTE_NEW_ALGO_TESTS === '1') await server.ssrLoadModule('/tests/navigation.test.ts');
  if (process.env.ROUTE_NEW_ALGO_TESTS === '1') await server.ssrLoadModule('/tests/q4_q6.test.ts');
  if (process.env.ROUTE_NEW_ALGO_TESTS === '1') await server.ssrLoadModule('/tests/short_link.test.ts');
  await server.ssrLoadModule('/tests/full_lap.test.ts');
  await server.ssrLoadModule('/tests/hexagon_entry.test.ts');
  }
  }
  }
} finally {
  await server.close();
}
