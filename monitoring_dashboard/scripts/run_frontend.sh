#!/bin/bash

# Monitoring Dashboard Frontend Startup Script
# Starts Vite development server

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRONTEND_DIR="$SCRIPT_DIR/../frontend"

echo "================================================"
echo "  Multi-Agent Formation Control Monitor"
echo "  Frontend Development Server (React + Vite)"
echo "================================================"
echo ""

# Navigate to frontend directory
cd "$FRONTEND_DIR"

# Check if node_modules exists
if [ ! -d "node_modules" ]; then
    echo "Installing frontend dependencies (this may take a few minutes)..."
    npm install
    echo "✓ Dependencies installed"
    echo ""
fi

# Start development server
echo "Starting frontend server on http://localhost:3000"
echo "Press Ctrl+C to stop"
echo ""

npm run dev
