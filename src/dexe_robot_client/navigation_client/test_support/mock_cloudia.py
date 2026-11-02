"""轻量迦智网关模拟器。

只覆盖第一版单机任务、任务状态和地图查询，不包含调度接口。测试可使用
``server.task_port`` 生成 ``InterfaceConfig``。
"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class MockCloudiaServer:

    def __init__(self, *, token="test-token", finish_after_polls=2):
        self.token = token
        self.finish_after_polls = finish_after_polls
        self._task_id = 0
        self._task = None
        self._poll_count = 0
        self._server = None
        self._thread = None

    @property
    def host(self):
        return "127.0.0.1"

    @property
    def task_port(self):
        return self._server.server_port if self._server else 0

    def start(self):
        owner = self

        class Handler(BaseHTTPRequestHandler):

            def log_message(self, *_args):
                return

            def _json(self, status, payload):
                encoded = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def _body(self):
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length else b"{}"
                return json.loads(raw.decode("utf-8"))

            def _authorized(self):
                return self.headers.get("X-Robot-Control-Token") == owner.token

            def do_POST(self):  # noqa: N802
                if not self._authorized():
                    self._json(200, {"code": 1006, "message": "unauthorized"})
                    return
                body = self._body()
                if self.path == "/api/v1/task/start":
                    owner._task_id += 1
                    owner._poll_count = 0
                    owner._task = {
                        "taskId": owner._task_id,
                        "taskType": body.get("taskType"),
                        "taskStatus": "running",
                        "error": {
                            "errorCode": 0,
                            "errorMessage": ""
                        },
                    }
                    self._json(200, {
                        "code": 0,
                        "message": "success",
                        "taskId": owner._task_id,
                    })
                    return
                if self.path == "/api/v1/task/stop":
                    if owner._task is not None:
                        owner._task["taskStatus"] = "failed"
                        owner._task["error"] = {
                            "errorCode": 1001,
                            "errorMessage": "stopped",
                        }
                    self._json(200, {"code": 0, "message": "success"})
                    return
                self._json(404, {"code": 404, "message": "not found"})

            def do_GET(self):  # noqa: N802
                if not self._authorized():
                    self._json(200, {"code": 1006, "message": "unauthorized"})
                    return
                if self.path == "/api/v1/robot/status":
                    if owner._task is not None:
                        owner._poll_count += 1
                        if owner._poll_count > owner.finish_after_polls:
                            owner._task["taskStatus"] = "finished"
                    self._json(
                        200, {
                            "status": {
                                "robotStatus": {
                                    "state": 2 if owner._task else 1,
                                    "informationType": 2 if owner._task else 1,
                                    "obstacleStop": False,
                                    "manual_mode": 0,
                                },
                                "error": {
                                    "errorCode": 0,
                                    "errorMessage": ""
                                },
                                "task": owner._task,
                            }
                        })
                    return
                if self.path == "/api/v1/map/info":
                    self._json(
                        200, {
                            "code": 0,
                            "message": "success",
                            "data": {
                                "nodes": [
                                    {
                                        "node_id": 1,
                                        "pose": {
                                            "position": {
                                                "x": 0.0,
                                                "y": 0.0
                                            },
                                            "orientation": {
                                                "w": 1.0,
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0
                                            },
                                        }
                                    },
                                    {
                                        "node_id": 2,
                                        "pose": {
                                            "position": {
                                                "x": 1.0,
                                                "y": 0.0
                                            },
                                            "orientation": {
                                                "w": 1.0,
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0
                                            },
                                        }
                                    },
                                ],
                                "edges": [{
                                    "source_node": 1,
                                    "dest_node": 2
                                }],
                            },
                        })
                    return
                self._json(404, {"code": 404, "message": "not found"})

        self._server = ThreadingHTTPServer((self.host, 0), Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="mock-cloudia",
            daemon=True,
        )
        self._thread.start()
        return self

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
