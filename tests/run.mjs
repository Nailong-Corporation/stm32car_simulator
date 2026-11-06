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
  await server.ssrLoadModule('/tests/physics.test.ts');
  if (process.argv.includes('--physics-only')) {
    // Isolate plant invariants from controller trajectory regressions.
  } else {
  await server.ssrLoadModule('/tests/startup.test.ts');
  await server.ssrLoadModule('/tests/navigation.test.ts');
  await server.ssrLoadModule('/tests/q4_q6.test.ts');
  }
} finally {
  await server.close();
}
