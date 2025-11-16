import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Use /ui/ as base in production so built assets resolve under /ui
// Dev remains at / for simplicity; backend proxy handles /api and /ws
export default defineConfig(({ mode }) => ({
  base: mode === 'production' ? '/ui/' : '/',
  plugins: [react()],
  server: {
    port: 3000,
    host: true,
    proxy: {
      // Proxy REST API requests to backend during development
      '/api': {
        target: 'http://localhost:8000',
        changeOrigin: true,
        secure: false,
      },
      // Proxy WebSocket to backend to avoid cross-origin issues
      '/ws': {
        target: 'ws://localhost:8000',
        ws: true,
        changeOrigin: true,
        secure: false,
      },
    },
  }
}))
