"""轻量迦智调度（CLOUDIA）模拟器：missions / mscmds / robots。

任务创建后 GET 轮询 ``finish_after_polls`` 次进入终态（默认 5 正常完成）；
``final_state`` 可设为 7 模拟出错。取消立即把任务置为 6。
``robcmd_code`` 非 0 时 robcmd 返回失败；收到的 robcmd 记在 ``robcmds``，``calls`` 按序记录 POST 路径。
"""

from __future__ import annotations

import json
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class MockCloudiaSchedServer:

    def __init__(self, *, finish_after_polls=2, final_state=5, map_name="m1"):
        self.finish_after_polls = finish_after_polls
        self.final_state = final_state
        self.map_name = map_name
        self.created = []  # 收到的 POST /missions body
        self.missions = {}
        self.robcmds = []  # 收到的 POST /robcmd body
        self.robcmd_code = 0
        self.calls = []  # POST 路径，按到达顺序
        self._polls = {}
        self._server = None
        self._thread = None

    @property
    def url(self):
        return f"http://127.0.0.1:{self._server.server_port}"

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

            def do_POST(self):
                body = self._body()
                owner.calls.append(self.path)
                if self.path == "/api/v1/missions":
                    owner.created.append(body)
                    mission_id = len(owner.created)
                    owner.missions[mission_id] = {"id": mission_id, "state": 0}
                    owner._polls[mission_id] = 0
                    self._json(201, owner.missions[mission_id])
                elif self.path == "/api/v1/mscmds":
                    mission = owner.missions.get(body.get("mission_id"))
                    if mission is None:
                        self._json(404, {"err_msg": "no such mission"})
                        return
                    mission["state"] = 6
                    self._json(200, {"id": 1, "result": 1, "err_msg": ""})
                elif self.path == "/api/v1/robcmd":
                    owner.robcmds.append(body)
                    self._json(
                        200, {
                            "code":
                                owner.robcmd_code,
                            "msg":
                                "mock robcmd failure"
                                if owner.robcmd_code else ""
                        })
                else:
                    self._json(404, {})

            def do_GET(self):
                match = re.fullmatch(r"/api/v1/missions/(\d+)", self.path)
                if match:
                    mission_id = int(match.group(1))
                    mission = owner.missions.get(mission_id)
                    if mission is None:
                        self._json(404, {})
                        return
                    if mission["state"] in (0, 1):
                        owner._polls[mission_id] += 1
                        if owner._polls[mission_id] >= owner.finish_after_polls:
                            mission["state"] = owner.final_state
                            if owner.final_state == 7:
                                mission["err_msg"] = "mock failure"
                        else:
                            mission["state"] = 1
                    self._json(200, mission)
                elif re.fullmatch(r"/api/v1/robots/\d+", self.path):
                    self._json(200, {"map_name": owner.map_name})
                else:
                    self._json(404, {})

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        daemon=True)
        self._thread.start()
        return self

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
