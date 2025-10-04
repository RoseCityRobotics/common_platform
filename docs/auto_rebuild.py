#!/usr/bin/env python3
"""
Auto-rebuild script for Sphinx documentation.
Watches for changes in source files and automatically rebuilds the documentation.
"""

import os
import sys
import time
import subprocess
from pathlib import Path
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class SphinxRebuildHandler(FileSystemEventHandler):
    def __init__(self, docs_dir):
        self.docs_dir = Path(docs_dir)
        self.last_rebuild = 0
        self.rebuild_delay = 2  # seconds to wait before rebuilding

    def on_modified(self, event):
        if event.is_directory:
            return

        # Only rebuild for markdown files
        if not event.src_path.endswith(('.md', '.py', '.yaml', '.yml')):
            return

        # Debounce rebuilds
        current_time = time.time()
        if current_time - self.last_rebuild < self.rebuild_delay:
            return

        self.last_rebuild = current_time
        print(f"\n🔄 File changed: {event.src_path}")
        self.rebuild_docs()

    def rebuild_docs(self):
        try:
            print("📚 Rebuilding documentation...")
            result = subprocess.run([
                sys.executable, '-m', 'sphinx',
                '-b', 'html',
                'source',
                'build/html'
            ], cwd=self.docs_dir, capture_output=True, text=True)

            if result.returncode == 0:
                print("✅ Documentation rebuilt successfully!")
            else:
                print(f"❌ Build failed: {result.stderr}")

        except Exception as e:
            print(f"❌ Error rebuilding: {e}")

def main():
    docs_dir = Path(__file__).parent
    source_dir = docs_dir / "source"

    if not source_dir.exists():
        print(f"❌ Source directory not found: {source_dir}")
        sys.exit(1)

    print(f"🔍 Watching for changes in: {source_dir}")
    print("📚 Auto-rebuilding documentation when files change...")
    print("Press Ctrl+C to stop")

    event_handler = SphinxRebuildHandler(docs_dir)
    observer = Observer()
    observer.schedule(event_handler, str(source_dir), recursive=True)

    observer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Stopping auto-rebuild...")
        observer.stop()

    observer.join()

if __name__ == "__main__":
    main()
