#!/usr/bin/env python3
"""
Local server for the Pioneer 3-AT modular Webots map viewer.

The WebSocket connection remains on ws://localhost:8765 and is provided
by the Webots controller. This server only serves the HTML/CSS/JS files.
"""

from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
import webbrowser

HOST = "127.0.0.1"
PORT = 8000
ROOT = Path(__file__).resolve().parent


class ViewerHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def log_message(self, format, *args):
        print(f"[HTTP] {self.address_string()} - {format % args}")


def main():
    server = ThreadingHTTPServer((HOST, PORT), ViewerHandler)
    url = f"http://{HOST}:{PORT}/map_viewer2_modular.html"

    print("=" * 60)
    print("Pioneer 3-AT Modular Map Viewer")
    print("=" * 60)
    print(f"Serving: {ROOT}")
    print(f"Viewer:  {url}")
    print("WebSocket: ws://localhost:8765")
    print("Press Ctrl+C to stop the server.")
    print("=" * 60)

    try:
        webbrowser.open(url)
    except Exception:
        pass

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping server...")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
