#!/usr/bin/env python3
"""
CSV viewer -- overlay any number of CSVs, pick a column per axis per file,
shift/scale every axis independently, 2D or 3D.

    python csv_viewer/serve.py              # opens http://127.0.0.1:8765
    python csv_viewer/serve.py --port 9000
    python csv_viewer/serve.py --selftest   # run from the repo root

A browser cannot read a path you type -- only a file you pick from its dialog.
Since the viewer takes typed paths, this process does the reading: it serves
index.html and answers /open?paths=<lines> with the raw text of every file
those lines match. Globs are expanded, relative paths resolve against the
directory you launched from. Nothing is parsed here; the page does that.

Binds to 127.0.0.1 only. There is deliberately no path sandbox -- typing an
absolute path to any CSV on the disk is the feature.
"""

import glob
import http.server
import json
import sys
import threading
import urllib.parse
import webbrowser
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = Path.cwd()


def expand(spec):
    """Textarea contents -> ([Path, ...], [line that matched nothing, ...])."""
    files, missing = [], []
    for line in spec.splitlines():
        line = line.strip()
        if not line:
            continue
        pat = Path(line).expanduser()
        if not pat.is_absolute():
            pat = ROOT / pat
        hits = sorted(p for p in map(Path, glob.glob(str(pat), recursive=True))
                      if p.is_file())
        files += hits
        if not hits:
            missing.append(line)
    return list(dict.fromkeys(files)), missing


def rel(p):
    """Path as typed-ish: relative to the launch dir when it lives under it."""
    try:
        return str(p.relative_to(ROOT))
    except ValueError:
        return str(p)


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=str(HERE), **kw)

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        if u.path != "/open":
            return super().do_GET()
        spec = urllib.parse.parse_qs(u.query).get("paths", [""])[0]
        files, missing = expand(spec)
        out = []
        for p in files:
            try:
                out.append({"path": rel(p), "text": p.read_text(errors="replace")})
            except OSError as e:
                missing.append("%s (%s)" % (rel(p), e.strerror))
        body = json.dumps({"files": out, "missing": missing}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


def selftest():
    files, missing = expand("csv_viewer/*.py\nnope/nothing.csv\n\n")
    assert [p.name for p in files] == ["serve.py"], files
    assert missing == ["nope/nothing.csv"], missing
    assert rel(ROOT / "a" / "b.csv") == "a/b.csv"
    assert rel(Path("/etc/hosts")) == "/etc/hosts"
    print("serve.py selftest ok")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
        sys.exit()
    port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 8765
    url = "http://127.0.0.1:%d/index.html" % port
    with http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler) as srv:
        print("%s\npaths resolve against %s\nCtrl-C to stop" % (url, ROOT))
        threading.Timer(0.5, webbrowser.open, [url]).start()
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            pass
