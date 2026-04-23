#!/usr/bin/env python3
"""
Simple local HTTP server for HomeNode telemetry debugging.

Expected request from MCU (current firmware):
GET /telemetry?t=25.0&h=60.0&fan=1&duty=60 HTTP/1.1
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict
from urllib.parse import parse_qs, urlparse


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HomeNode telemetry test server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="Bind port (default: 8080)")
    parser.add_argument("--path", default="/telemetry", help="Expected path (default: /telemetry)")
    parser.add_argument(
        "--log-file",
        default="telemetry_log.jsonl",
        help="JSONL output file (default: telemetry_log.jsonl)",
    )
    return parser.parse_args()


class TelemetryHandler(BaseHTTPRequestHandler):
    server_version = "HomeNodeTelemetry/1.0"

    def _write_json(self, code: int, payload: Dict[str, object]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _append_log(self, event: Dict[str, object]) -> None:
        log_path = Path(self.server.log_file)
        with log_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(event, ensure_ascii=False) + "\n")

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path != self.server.expected_path:
            self._write_json(404, {"ok": False, "error": "path_not_found", "path": parsed.path})
            return

        qs = parse_qs(parsed.query, keep_blank_values=True)
        flat = {k: v[-1] if len(v) > 0 else "" for k, v in qs.items()}

        event = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "client_ip": self.client_address[0],
            "path": parsed.path,
            "query": flat,
        }

        self._append_log(event)
        print(
            f"[{event['timestamp']}] {event['client_ip']} "
            f"t={flat.get('t', '?')} h={flat.get('h', '?')} "
            f"fan={flat.get('fan', '?')} duty={flat.get('duty', '?')}"
        )
        self._write_json(200, {"ok": True, "received": flat})

    def log_message(self, fmt: str, *args: object) -> None:
        # Silence BaseHTTPRequestHandler default logs.
        return


def main() -> None:
    args = _parse_args()
    server = ThreadingHTTPServer((args.host, args.port), TelemetryHandler)
    server.expected_path = args.path  # type: ignore[attr-defined]
    server.log_file = args.log_file  # type: ignore[attr-defined]

    print(f"Listening on http://{args.host}:{args.port}{args.path}")
    print(f"Logging telemetry to {Path(args.log_file).resolve()}")
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()

