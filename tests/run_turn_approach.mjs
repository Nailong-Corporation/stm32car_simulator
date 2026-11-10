import { createServer } from 'vite';

const server = await createServer({
  server: { middlewareMode: true },
  logLevel: 'error',
  cacheDir: '.vite-test-cache',
  optimizeDeps: { noDiscovery: true, include: [] },
  resolve: { preserveSymlinks: true },
});
try {
  await server.ssrLoadModule('/tests/pivot_lead.test.ts');
  await server.ssrLoadModule('/tests/firmware_sync.test.ts');
} finally {
  await server.close();
}
