#!/bin/bash
# Development server script for documentation
# Starts auto-rebuild and web server

set -e

echo "🚀 Starting documentation development server..."

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Activate virtual environment
source venv/bin/activate

# Initial build
echo "📚 Building documentation..."
sphinx-build -b html source build/html

# Start auto-rebuild in background
echo "🔄 Starting auto-rebuild watcher..."
python auto_rebuild.py &
AUTO_REBUILD_PID=$!

# Start web server
echo "🌐 Starting web server on http://localhost:8080"
echo "Press Ctrl+C to stop both services"
cd build/html
python3 -m http.server 8080 &
WEB_SERVER_PID=$!

# Function to cleanup on exit
cleanup() {
    echo -e "\n🛑 Stopping services..."
    kill $AUTO_REBUILD_PID 2>/dev/null || true
    kill $WEB_SERVER_PID 2>/dev/null || true
    exit 0
}

# Set trap to cleanup on script exit
trap cleanup SIGINT SIGTERM

# Wait for processes
wait
