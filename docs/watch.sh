#!/bin/bash

# Auto-rebuild and serve Sphinx documentation
echo "🔨 Setting up auto-rebuild for Sphinx documentation..."
echo "📁 Watching for changes in: docs/source/"
echo "🌐 Documentation will be available at: http://localhost:8000"
echo "🛑 Press Ctrl+C to stop"

# Function to rebuild docs
rebuild_docs() {
    echo "🔄 Changes detected, rebuilding documentation..."
    source venv/bin/activate
    make html
    echo "✅ Documentation rebuilt successfully!"
}

# Initial build
rebuild_docs

# Start server in background
cd build/html
python3 -m http.server 8000 &
SERVER_PID=$!

# Watch for changes and rebuild
while inotifywait -e modify,create,delete -r ../source/ 2>/dev/null; do
    rebuild_docs
done

# Cleanup on exit
kill $SERVER_PID
