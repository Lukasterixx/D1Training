"""Localhost HTTP server: a JSON API over `store`, the static page, and repo files."""
from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import errno
import json
import mimetypes
from pathlib import Path
import re
import traceback
import urllib.parse

from . import store

STATIC = Path(__file__).resolve().parent / "static"
TEXT_SUFFIXES = {".md", ".py", ".yaml", ".yml", ".csv", ".txt", ".urdf", ".sh", ".json"}


class Handler(BaseHTTPRequestHandler):
    root: Path = store.ROOT
    server_version = "ThesisRecord/1"

    def log_message(self, format, *args):  # Keep the terminal for errors only.
        pass

    def _send(self, status: int, body: bytes, content_type: str, cache: bool = False):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "max-age=60" if cache else "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, value, status: int = 200):
        self._send(status, json.dumps(value).encode(), "application/json")

    def _file(self, path: Path, cache: bool = False):
        if path.suffix.lower() in TEXT_SUFFIXES:
            content_type = "text/plain; charset=utf-8"
        else:
            content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        self._send(200, path.read_bytes(), content_type, cache)

    def _host_allowed(self) -> bool:
        # Rejects DNS-rebinding requests that reach 127.0.0.1 under a foreign hostname.
        host = (self.headers.get("Host") or "").rsplit(":", 1)[0].strip("[]")
        return host in {"localhost", "127.0.0.1", "::1"}

    def _repo_file(self, rel: str) -> Path | None:
        rel = urllib.parse.unquote(rel)
        if any(part.startswith(".") for part in Path(rel).parts):
            return None
        path = (self.root / rel).resolve()
        try:
            path.relative_to(self.root.resolve())
        except ValueError:
            return None
        return path if path.is_file() else None

    def do_GET(self):
        if not self._host_allowed():
            return self._send(403, b"Forbidden host", "text/plain")
        url = urllib.parse.urlparse(self.path)
        query = urllib.parse.parse_qs(url.query)
        try:
            if url.path in ("/", "/index.html"):
                return self._file(STATIC / "index.html")
            if url.path.startswith("/static/"):
                path = (STATIC / url.path[len("/static/"):]).resolve()
                if path.parent == STATIC and path.is_file():
                    return self._file(path)
            elif url.path.startswith("/files/"):
                path = self._repo_file(url.path[len("/files/"):])
                if path:
                    return self._file(path)
            elif url.path.startswith("/api/"):
                return self._api(url.path[len("/api/"):], query)
            return self._send(404, b"Not found", "text/plain")
        except BrokenPipeError:
            pass
        except Exception as exc:  # Show the failure in the page rather than hanging it.
            traceback.print_exc()
            self._json({"error": f"{type(exc).__name__}: {exc}"}, 500)

    def _api(self, name: str, query: dict):
        cfg = store.load_config(self.root)
        today = store.today_local()
        if name == "stamp":
            return self._json({"stamp": store.stamp(self.root, cfg)})
        if name == "overview":
            return self._json(store.overview(self.root, cfg, today))
        week = re.fullmatch(r"week/(\d+)", name)
        if week and 1 <= int(week.group(1)) <= cfg["weeks"]:
            return self._json(store.week_detail(self.root, cfg, int(week.group(1)), today))
        if name == "findings":
            path = store.results_dir(self.root) / "findings.md"
            return self._json({"html": store.render_file(self.root, path)})
        if name == "doc":
            rel = query.get("path", [""])[0]
            path = self._repo_file(rel)
            if path and path.suffix == ".md":
                text = store.read_text(path)
                return self._json({"path": rel, "title": store.mdrender.title_of(text, path.stem),
                                   "html": store.render_file(self.root, path)})
        if name == "live":
            return self._json({"runs": store.live_runs(self.root, cfg)})
        if name == "scalars":
            return self._scalars(cfg, query)
        return self._json({"error": f"Unknown endpoint {name}"}, 404)

    def _scalars(self, cfg: dict, query: dict):
        run = query.get("run", [""])[0]
        if "week" in query:
            folder = store.week_dir(self.root, int(query["week"][0])) / "runs" / run
            csv_path = folder / "scalars.csv"
            if "/" not in run and csv_path.is_file():
                return self._json({"tags": store.scalars_from_csv(csv_path)})
        elif "live" in query:
            rel = query["live"][0]
            folder = (self.root / rel).resolve()
            roots = [(self.root / r).resolve() for r in cfg.get("log_roots", [])]
            if folder.parent in roots and folder.is_dir():
                return self._json({"tags": store.scalars_from_events(folder)})
        return self._json({"error": "No scalars for that run."}, 404)


def serve(root: Path, host: str, port: int, open_browser: bool) -> None:
    Handler.root = root
    server = None
    for candidate in range(port, port + 20):
        try:
            server = ThreadingHTTPServer((host, candidate), Handler)
            break
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE:
                raise
    if server is None:
        raise SystemExit(f"Ports {port}-{port + 19} are all in use.")
    server.daemon_threads = True
    url = f"http://localhost:{server.server_address[1]}/"
    print(f"Thesis B record: {url}  (Ctrl+C to stop)")
    if open_browser:
        import webbrowser
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
