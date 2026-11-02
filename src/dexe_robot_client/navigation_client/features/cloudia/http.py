"""迦智 HTTP 传输层。

查询类 GET 允许有限重试；任务提交/停止等 POST 不自动重试，避免网络超时
后重复创建或停止任务。
"""

from __future__ import annotations

from typing import Any

try:
    import requests
except ImportError:  # pragma: no cover - only affects an uninstalled optional backend
    requests = None


class CloudiaApiError(RuntimeError):
    """迦智请求失败，保留原始 code 和响应。"""

    def __init__(self, message: str, *, code: Any = None, payload=None):
        super().__init__(message)
        self.code = code
        self.payload = payload


class CloudiaHttpClient:

    def __init__(self, node, cfg):
        if requests is None:
            raise CloudiaApiError("迦智后端需要 requests，请先安装项目依赖。")
        self._node = node
        self._host = (cfg.cloudia_host or "").strip().rstrip("/")
        self._task_port = cfg.cloudia_task_port
        self._token = cfg.cloudia_token or ""
        self._timeout_sec = 3.0
        self._query_retries = 2
        self._session = requests.Session()

    def _url(self, port: int, path: str) -> str:
        return f"http://{self._host}:{port}{path}"

    def _request(self,
                 method: str,
                 path: str,
                 *,
                 port: int,
                 body=None,
                 authenticated: bool = False):
        if not self._host:
            raise CloudiaApiError("CLOUDIA_HOST 未配置")
        headers = {}
        if body is not None:
            headers["Content-Type"] = "application/json"
        if authenticated:
            if not self._token:
                raise CloudiaApiError("CLOUDIA_TOKEN 未配置", code=1006)
            headers["X-Robot-Control-Token"] = self._token

        attempts = 1 if method.upper() != "GET" else self._query_retries + 1
        last_error = None
        for _ in range(attempts):
            try:
                response = self._session.request(
                    method,
                    self._url(port, path),
                    json=body,
                    headers=headers,
                    timeout=self._timeout_sec,
                )
                try:
                    payload = response.json()
                except ValueError as exc:
                    raise CloudiaApiError(
                        f"响应不是 JSON：HTTP {response.status_code}") from exc
                if response.status_code != 200:
                    raise CloudiaApiError(f"HTTP {response.status_code}",
                                          payload=payload)
                return payload
            except requests.RequestException as exc:
                last_error = exc
            except CloudiaApiError:
                raise
        raise CloudiaApiError(f"迦智请求失败：{last_error}")

    @staticmethod
    def _check_task_response(payload):
        code = payload.get("code")
        if code != 0:
            raise CloudiaApiError(
                payload.get("message", "迦智任务请求被拒绝"),
                code=code,
                payload=payload,
            )
        return payload

    def start_task(self, task_type: str, **fields):
        payload = self._request(
            "POST",
            "/api/v1/task/start",
            port=self._task_port,
            body={
                "taskType": task_type,
                **fields
            },
            authenticated=True,
        )
        return self._check_task_response(payload)

    def stop_task(self):
        payload = self._request(
            "POST",
            "/api/v1/task/stop",
            port=self._task_port,
            authenticated=True,
        )
        return self._check_task_response(payload)

    def get_robot_status(self):
        # 文档中的状态响应以 status/task 为主体，不依赖顶层 code。
        return self._request(
            "GET",
            "/api/v1/robot/status",
            port=self._task_port,
            authenticated=True,
        )

    def get_map_info(self):
        payload = self._request(
            "GET",
            "/api/v1/map/info",
            port=self._task_port,
            authenticated=True,
        )
        if payload.get("code") != 0:
            raise CloudiaApiError(
                payload.get("message", "地图查询失败"),
                code=payload.get("code"),
                payload=payload,
            )
        return payload
