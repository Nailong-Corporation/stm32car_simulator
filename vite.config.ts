import { defineConfig } from 'vite';

export default defineConfig({
  // 使用相对路径，确保构建产物可直接部署到 GitHub Pages 的项目子路径
  // （https://<owner>.github.io/<repo>/），同时兼容本地预览。
  base: './',
});
