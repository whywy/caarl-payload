#!/usr/bin/env python3
"""CAARL payload dashboard.

Talks to the ESP32 over USB and serves a live dashboard at http://127.0.0.1:8765.
Uses only Python's standard library, so there is nothing to install.

    python3 dashboard.py               # finds the ESP32 and opens your browser
    python3 dashboard.py --no-browser  # just start the server
    python3 dashboard.py --port /dev/ttyUSB1

The ESP32 must be running firmware/payload-dashboard (921600 baud).
Only one program can use the USB port at a time: close the Arduino Serial
Monitor before starting this.
"""
import argparse
import base64
import glob
import json
import os
import queue
import re
import select
import shutil
import subprocess
import sys
import termios
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.path.join(HERE, "web")
DOWNLOAD_DIR = os.path.join(HERE, "downloads")
BAUD = termios.B921600

# Commands the web page may send. Anything else is refused.
ALLOWED = [
    re.compile(r"INFO"),
    re.compile(r"LS"),
    re.compile(r"LOG START"),
    re.compile(r"LOG STOP"),
    re.compile(r"CAL GYRO"),
    re.compile(r"SERVO MODE (?:MANUAL|AUTO)"),
    re.compile(r"MARK [^\r\n,\"\\]{1,60}"),
    re.compile(r"DEL /[A-Za-z0-9_.\-]{1,39}"),
]
FILE_NAME = re.compile(r"/[A-Za-z0-9_.\-]{1,39}")

# ---------- desk simulation and log replay ----------
# The page never gets its own copy of the flight phase detector: these run the real C++ tools
# (tests/phase_sim.cpp, tools/phase_replay.cpp), which use firmware/payload-dashboard/flight_phase.h,
# so what the page shows is exactly what the payload would decide.
BUILD_DIR = os.path.join(HERE, "build")          # git ignores build/
SIM_DIR = os.path.join(BUILD_DIR, "simlogs")
TOOL_SOURCES = {"phase_sim": "tests/phase_sim.cpp", "phase_replay": "tools/phase_replay.cpp"}
DETECTOR = "firmware/payload-dashboard/flight_phase.h"
LOG_NAME = re.compile(r"[A-Za-z0-9_.\-]{1,40}\.(?:csv|CSV)")
SETTING_VALUE = re.compile(r"-?\d{1,7}(?:\.\d{1,6})?")
tool_lock = threading.Lock()


def build_tool(name):
    """Compile a tool into build/ if it's missing or older than its source or the detector."""
    src = os.path.join(HERE, TOOL_SOURCES[name])
    exe = os.path.join(BUILD_DIR, name)
    newest = max(os.path.getmtime(src), os.path.getmtime(os.path.join(HERE, DETECTOR)))
    if os.path.exists(exe) and os.path.getmtime(exe) >= newest:
        return exe
    gxx = shutil.which("g++")
    if not gxx:
        raise RuntimeError("The C++ compiler (g++) isn't installed. On Ubuntu or Zorin: sudo apt install g++")
    os.makedirs(BUILD_DIR, exist_ok=True)
    r = subprocess.run([gxx, "-std=c++17", "-O2", "-Wall", "-Wno-format-security", "-o", exe, src],
                       capture_output=True, text=True, timeout=180)
    if r.returncode:
        raise RuntimeError(f"Couldn't build {name}:\n{r.stderr[-1500:]}")
    return exe


def run_simulation():
    """Run every simulated flight; returns scenarios with their phase changes and checks."""
    with tool_lock:
        exe = build_tool("phase_sim")
        os.makedirs(SIM_DIR, exist_ok=True)
        r = subprocess.run([exe, "--csv", SIM_DIR], capture_output=True, text=True, timeout=120)
    scenarios, current = [], None
    for line in r.stdout.splitlines():
        m = re.match(r"== (.+?)  \(true apogee (\d+) m at ([\d.]+) s\)", line)
        if m:
            current = {"name": m[1], "apogee_m": int(m[2]), "apogee_s": float(m[3]), "log": None,
                       "events": [], "checks": []}
            scenarios.append(current)
        elif current is None:
            continue
        elif (m := re.match(r"\s+log: (\S+)", line)):
            current["log"] = m[1]
        elif (m := re.match(r"\s+\[(PASS|FAIL)\] (.*)", line)):
            current["checks"].append({"ok": m[1] == "PASS", "text": m[2]})
        elif (m := re.match(r"\s+([\d.]+) s  (.*)", line)):
            current["events"].append({"t": float(m[1]), "text": m[2]})
    return {"type": "sim", "ok": r.returncode == 0, "scenarios": scenarios,
            "result": r.stdout.strip().splitlines()[-1] if r.stdout.strip() else "no output"}


def replay_settings():
    """The detector's settings, defaults and meanings, straight from the replay tool."""
    with tool_lock:
        exe = build_tool("phase_replay")
        r = subprocess.run([exe, "--settings"], capture_output=True, text=True, timeout=30)
    out = []
    for line in r.stdout.splitlines()[1:]:
        parts = line.split(None, 2)
        if len(parts) == 3:
            out.append({"name": parts[0], "default": parts[1], "meaning": parts[2]})
    return out


def list_logs():
    def csvs(folder):
        if not os.path.isdir(folder):
            return []
        return sorted(f for f in os.listdir(folder) if LOG_NAME.fullmatch(f))
    return {"type": "logs", "downloads": csvs(DOWNLOAD_DIR), "sim": csvs(SIM_DIR)}


def log_path(source, name):
    """Only plain file names inside downloads/ or build/simlogs/ are allowed."""
    folder = {"downloads": DOWNLOAD_DIR, "sim": SIM_DIR}.get(source)
    if not folder or not isinstance(name, str) or not LOG_NAME.fullmatch(name):
        raise ValueError("bad log name")
    path = os.path.join(folder, name)
    if os.path.dirname(os.path.realpath(path)) != os.path.realpath(folder) or not os.path.isfile(path):
        raise ValueError("log not found")
    return path


def run_replay(source, name, settings):
    path = log_path(source, name)
    if settings is None:
        settings = {}
    if not isinstance(settings, dict) or len(settings) > 50:
        raise ValueError("settings must be a list of NAME: value pairs")
    known = {s["name"] for s in replay_settings()}
    args = []
    for key, value in settings.items():
        value = str(value).strip()
        if value == "":
            continue
        if key not in known or not SETTING_VALUE.fullmatch(value):
            raise ValueError(f"bad setting {key}={value}")
        args.append(f"{key}={value}")
    with tool_lock:
        exe = build_tool("phase_replay")
        r = subprocess.run([exe, path, *args], capture_output=True, text=True, timeout=60)
    if r.returncode:
        raise RuntimeError(r.stderr.strip() or "replay failed")
    result = {"type": "replay", "log": name, "settings": args, "replayed": [], "recorded": None, "summary": []}
    section = None
    for line in r.stdout.splitlines():
        if line.startswith("Replayed"):
            section = "replayed"
        elif line.startswith("Recorded"):
            section, result["recorded"] = "recorded", []
        elif line.startswith("Summary"):
            section = "summary"
            result["summary"].append(line[len("Summary: "):])
        elif section == "summary" and line.strip():
            result["summary"].append(line.strip())
        elif section in ("replayed", "recorded") and (m := re.match(r"\s+([\d.]+) s  (.*)", line)):
            result[section].append({"t": float(m[1]), "text": m[2]})
    return result


class Board:
    """The USB serial link to the ESP32, with automatic reconnect."""

    def __init__(self, port=None):
        self.wanted_port = port
        self.port = None
        self.fd = None
        self.info = None
        self.last = None
        self.cmd_lock = threading.Lock()
        self.waiting = False
        self.replies = queue.Queue()
        self.listeners = []
        self.listeners_lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    # ---- events for the web page ----
    def subscribe(self):
        q = queue.Queue(maxsize=100)
        with self.listeners_lock:
            self.listeners.append(q)
        return q

    def unsubscribe(self, q):
        with self.listeners_lock:
            if q in self.listeners:
                self.listeners.remove(q)

    def publish(self, msg):
        with self.listeners_lock:
            for q in self.listeners:
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    pass  # a slow browser tab just misses a reading

    def status(self):
        return {"type": "conn", "connected": self.fd is not None, "port": self.port}

    # ---- serial port ----
    def _find_port(self):
        if self.wanted_port:
            return self.wanted_port if os.path.exists(self.wanted_port) else None
        ports = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
        return ports[0] if ports else None

    def _open(self, path):
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        cc = attrs[6]
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        # raw 8N1, no modem hang-up (so closing the port doesn't reset the board)
        termios.tcsetattr(fd, termios.TCSANOW,
                          [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0, BAUD, BAUD, cc])
        termios.tcflush(fd, termios.TCIOFLUSH)
        return fd

    def _close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None
        self.info = None
        self.publish(self.status())

    def write_line(self, text):
        if self.fd is None:
            raise ConnectionError("The payload isn't connected.")
        data = (text + "\n").encode()
        while data:
            try:
                n = os.write(self.fd, data)
                data = data[n:]
            except BlockingIOError:
                time.sleep(0.01)

    def _run(self):
        buf = b""
        last_info_ask = 0
        while True:
            if self.fd is None:
                path = self._find_port()
                if not path:
                    time.sleep(1)
                    continue
                try:
                    self.fd = self._open(path)
                    self.port = path
                    buf = b""
                    print(f"Connected to {path}")
                    self.publish(self.status())
                    last_info_ask = 0
                except OSError as e:
                    print(f"Can't open {path}: {e}  (is the Serial Monitor open?)")
                    time.sleep(2)
                    continue
            try:
                ready, _, _ = select.select([self.fd], [], [], 0.5)
                if ready:
                    chunk = os.read(self.fd, 65536)
                    if not chunk:
                        raise OSError("port closed")
                    buf += chunk
            except OSError:
                print("Payload disconnected")
                self._close()
                continue

            *lines, buf = buf.split(b"\n")
            if len(buf) > 100000:
                buf = b""
            for raw in lines:
                self._handle(raw)

            # ask what's connected until the board answers (after boot or reconnect)
            if self.info is None and time.time() - last_info_ask > 2 and not self.waiting:
                last_info_ask = time.time()
                try:
                    self.write_line("INFO")
                except (ConnectionError, OSError):
                    pass

    def _handle(self, raw):
        line = raw.decode("utf-8", "replace").strip()
        if not line.startswith("{"):
            return  # boot messages and noise
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            return
        kind = msg.get("type")
        if kind == "d":
            self.last = msg
            self.publish(msg)
            return
        if kind == "info":
            self.info = msg
            self.publish(msg)
        if kind == "err" and msg.get("cmd") == "LOG":
            self.publish(msg)  # recording stopped on its own (card pulled out?)
        if self.waiting:
            self.replies.put(msg)

    # ---- commands ----
    def command(self, text, timeout=6):
        """Send one command and return its reply."""
        with self.cmd_lock:
            self._drain()
            self.waiting = True
            try:
                self.write_line(text)
                end = time.time() + timeout
                while time.time() < end:
                    try:
                        msg = self.replies.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if msg.get("type") in ("ok", "err", "ls", "info"):
                        return msg
                return {"type": "err", "msg": "The payload didn't answer. Is it running the dashboard firmware?"}
            finally:
                self.waiting = False

    def get_file(self, name):
        """Copy a file from the SD card. Returns its bytes, or raises with a message."""
        with self.cmd_lock:
            self._drain()
            self.waiting = True
            try:
                self.write_line("GET " + name)
                first = self._next_reply(6)
                if first.get("type") == "err":
                    raise RuntimeError(first.get("msg", "error"))
                if first.get("type") != "get":
                    raise RuntimeError("Unexpected reply from the payload.")
                size = int(first.get("size", 0))
                data = bytearray()
                last_pub = 0
                while True:
                    msg = self._next_reply(6)
                    if msg.get("type") == "chunk":
                        data += base64.b64decode(msg["d"])
                        if time.time() - last_pub > 0.2:
                            last_pub = time.time()
                            self.publish({"type": "progress", "name": name, "done": len(data), "size": size})
                    elif msg.get("type") == "end":
                        break
                self.publish({"type": "progress", "name": name, "done": len(data), "size": size})
                if len(data) != size:
                    raise RuntimeError(f"Transfer incomplete: got {len(data)} of {size} bytes. Try again.")
                return bytes(data)
            finally:
                self.waiting = False

    def _next_reply(self, timeout):
        try:
            return self.replies.get(timeout=timeout)
        except queue.Empty:
            raise RuntimeError("The payload stopped answering.")

    def _drain(self):
        while not self.replies.empty():
            self.replies.get_nowait()


board = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # keep the terminal quiet

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path in ("/", "/index.html"):
            with open(os.path.join(WEB_DIR, "index.html"), "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif url.path == "/events":
            self._events()
        elif url.path == "/api/logs":
            self._json(list_logs())
        elif url.path == "/api/settings":
            try:
                self._json({"type": "settings", "settings": replay_settings()})
            except (RuntimeError, OSError, subprocess.SubprocessError) as e:
                self._json({"type": "err", "msg": str(e)}, 500)
        elif url.path == "/api/simfile":
            try:
                path = log_path("sim", parse_qs(url.query).get("name", [""])[0])
            except ValueError as e:
                return self._json({"type": "err", "msg": str(e)}, 400)
            with open(path, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/csv")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif url.path == "/api/file":
            name = parse_qs(url.query).get("name", [""])[0]
            if not FILE_NAME.fullmatch(name):
                return self._json({"type": "err", "msg": "bad file name"}, 400)
            try:
                data = board.get_file(name)
            except (RuntimeError, ConnectionError, OSError) as e:
                return self._json({"type": "err", "msg": str(e)}, 409)
            os.makedirs(DOWNLOAD_DIR, exist_ok=True)
            with open(os.path.join(DOWNLOAD_DIR, name.lstrip("/")), "wb") as f:
                f.write(data)
            self.send_response(200)
            self.send_header("Content-Type", "text/csv" if name.upper().endswith(".CSV") else "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("X-Saved-To", os.path.join(DOWNLOAD_DIR, name.lstrip("/")))
            self.end_headers()
            self.wfile.write(data)
        else:
            self.send_error(404)

    def _same_origin(self):
        """Refuse requests that another website in the same browser tries to send here."""
        host = self.headers.get("Host", "")
        origin = self.headers.get("Origin")
        ok_hosts = {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
        return host in ok_hosts and (origin is None or origin.split("://", 1)[-1] in ok_hosts)

    def do_POST(self):
        path = urlparse(self.path).path
        if path not in ("/api/cmd", "/api/sim", "/api/replay"):
            return self.send_error(404)
        if not self._same_origin():
            return self._json({"type": "err", "msg": "request not from the dashboard page"}, 403)
        try:
            length = int(self.headers.get("Content-Length", 0))
            if length > 10000:
                raise ValueError("too big")
            body = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(body, dict):
                raise ValueError("not an object")
        except (ValueError, json.JSONDecodeError):
            return self._json({"type": "err", "msg": "bad request"}, 400)
        if path == "/api/sim":
            try:
                return self._json(run_simulation())
            except (RuntimeError, OSError, subprocess.SubprocessError) as e:
                return self._json({"type": "err", "msg": str(e)}, 500)
        if path == "/api/replay":
            try:
                return self._json(run_replay(body.get("source"), body.get("name"), body.get("settings")))
            except ValueError as e:
                return self._json({"type": "err", "msg": str(e)}, 400)
            except (RuntimeError, OSError, subprocess.SubprocessError) as e:
                return self._json({"type": "err", "msg": str(e)}, 500)
        cmd = body.get("cmd", "")
        if not isinstance(cmd, str) or not any(p.fullmatch(cmd) for p in ALLOWED):
            return self._json({"type": "err", "msg": "command not allowed"}, 400)
        try:
            self._json(board.command(cmd, timeout=10 if cmd == "CAL GYRO" else 6))
        except (ConnectionError, OSError) as e:
            self._json({"type": "err", "msg": str(e)})

    def _events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        q = board.subscribe()
        try:
            first = [board.status()] + ([board.info] if board.info else [])
            for msg in first:
                self.wfile.write(f"data: {json.dumps(msg)}\n\n".encode())
            self.wfile.flush()
            while True:
                try:
                    msg = q.get(timeout=10)
                    self.wfile.write(f"data: {json.dumps(msg)}\n\n".encode())
                except queue.Empty:
                    self.wfile.write(b": still here\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            board.unsubscribe(q)


def main():
    global board
    ap = argparse.ArgumentParser(description="CAARL payload live dashboard")
    ap.add_argument("--port", help="serial port (default: first /dev/ttyUSB* found)")
    ap.add_argument("--http", type=int, default=8765, help="web page port (default 8765)")
    ap.add_argument("--no-browser", action="store_true", help="don't open a browser")
    args = ap.parse_args()

    url = f"http://127.0.0.1:{args.http}"
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)
    except OSError:
        print(f"The dashboard is already running in another terminal. Open {url} in your browser,\n"
              f"or stop the other copy (Ctrl+C in its terminal) and try again.")
        if not args.no_browser:
            webbrowser.open(url)
        sys.exit(1)
    server.daemon_threads = True
    board = Board(args.port)
    print(f"CAARL dashboard running at {url}  (press Ctrl+C to stop)")
    if not args.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(0)


if __name__ == "__main__":
    main()
