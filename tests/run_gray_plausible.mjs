import { createServer } from 'vite';
const server = await createServer({server:{middlewareMode:true}, logLevel:'error', optimizeDeps:{noDiscovery:true,include:[]}});
try {
  await server.ssrLoadModule('/tests/gray_plausible.test.ts');
} finally {
  await server.close();
}
