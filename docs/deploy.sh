#!/bin/bash

# Build and serve documentation locally
echo "🔨 Building Sphinx documentation..."
source venv/bin/activate
make html

echo "🌐 Starting local server..."
echo "📖 Documentation will be available at: http://localhost:8000"
echo "🛑 Press Ctrl+C to stop the server"

cd build/html
python3 -m http.server 8000
