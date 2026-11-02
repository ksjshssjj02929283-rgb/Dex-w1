"""把迦智车端当前在跑的路网转换后同步到自研调度（dexe_scheduler）

在 pc2 上直接跑（不需要 source ROS，也不用装任何包）：

    python3 ~/w1/dexe_robot_client/src/dexe_robot_client/navigation_client/features/cloudia/sync_roadnet.py
    python3 .../sync_roadnet.py --dry-run          # 只导出和检查，不上传
    python3 .../sync_roadnet.py --zone-id <区域id>  # 不按机器人找区域

三步，每步之间人为确认：

  1. 车端：问底盘 `/api/v1/map/info` 拿当前加载的工程 / 图层组 / 路网，
     再 ssh 把车上那份工程库 `/home/jz/map/<工程>.db` 拷回来（栅格图只在 .db 里）。
  2. 云端：确定调度地址 / 园区 / 机器人，按机器人查它所在区域和当前生效地图并打印。
  3. 导出三件套（png / yaml.json / graph.json）→ 本地检查 → 确认 → `POST /api/maps/import`。

## 与 jiazhi_client 的关系

本文件是 `jiazhi_client.tools.sync_roadnet` 的单文件精简版（另含 export_map /
export_graph / upload_map / routing.unified_graph 里 sync 用得到的部分），**两边独立维护**。
只覆盖 sync 的固定口径：单位变换（不和我方底盘共图）、点导成 STATION（充电点 CHARGE）。
**graph.json / yaml.json / PNG 的口径改动必须两边一起改**，改完用同一份 .db +
map_info 两边各导一次，产物应逐字节一致。

依赖只有标准库 + yaml / numpy / PIL（pc2 系统 Python 自带），故意不做包内导入：
按文件路径直接跑时不会经过 `navigation_client/__init__` 那条 rclpy 导入链。

## 身份从哪来

调度上的身份从 pc2 的 traffic_control.yaml 取 workspace_id / entity_id / mqtt.host，
再按 entity_id 查它所在的区域。**本机就有这份文件时直接读（即在 pc2 上跑）**，
否则 ssh 到 `--pc2`。优先级：命令行 > pc2。底盘 IP / token 取 `--car` /
`--car-token`，缺省用环境变量 `CLOUDIA_HOST` / `CLOUDIA_TOKEN`（pc2 的 ~/.dex_env.sh 里有）。

**调度地址**：`--cloud-host` > 环境变量 `CLOUDIA_SCHED_IP`（pc2 ~/.dex_env.sh，
迦智调度与自研调度同机）> `CLOUD_HOST_DEFAULT`。pc2 traffic_control.yaml 的
mqtt.host 与它不一致就说明那份配置过时了，只提示、不采信。

## 站点编号与名称

节点导成 STATION，编号 `S{迦智点号:03d}`；名称取工程库 `topology_node.name`，
没有名字就用点号本身。node_map.yaml 跟三件套放在一起。

**充电点例外，导成 CHARGE、编号 `C{迦智点号:03d}`**：调度只按节点号 `C` 前缀认
充电站（dexe_scheduler `core/charging_station.py`），导成 S 就派不了回充。
充电点取工程库 `topology_node.node_type` 的 `CHARGE_NODE_BIT`（Carly 里把点设成
「充电点」后 128 → 132，2026-10-28 车上 1016_1.db 点 4 实测）。交管按节点 `name`
（= 迦智点号）下发，不解析 no 前缀，所以换前缀不影响下发。

## 覆盖提醒

导入会把新图设为该区域的生效路网，原来生效的那张即被替换。当前生效地图如果
不是本工具导出的（名字不以 `JZ_` 开头），会醒目提示并单独确认一次（`-y` 时照样放行）。

## 口令

全部用默认口令，不询问：底盘 SSH 用迦智出厂默认、pc2 用工控机默认、调度用
admin/admin123。改过了就设环境变量覆盖：`JZ_CAR_PASS` / `PC2_PASS` / `CLOUD_PASSWORD`
（账号 `CLOUD_USER`）。
"""

from __future__ import annotations

import os
import sys

if __name__ == "__main__":
    # 按文件路径直接跑时本目录会排在 sys.path 首位，同目录的 http.py 会遮住标准库
    # http，urllib.request 随之导入失败。必须在下面的 import 之前摘掉。
    _here = os.path.dirname(os.path.abspath(__file__))
    sys.path[:] = [
        p for p in sys.path if os.path.abspath(p or os.curdir) != _here
    ]

import argparse
import datetime as _dt
import hashlib
import io
import json
import math
import mimetypes
import shlex
import shutil
import sqlite3
import struct
import subprocess
import urllib.error
import urllib.request
import uuid
from typing import Any

import yaml

CLOUD_PORT = 3001
# 当前调度地址（2026-10-16 起 .19 → .210，2026-10-30 起 → .99）。
# 优先取环境变量 CLOUDIA_SCHED_IP，都没有才用这里
CLOUD_HOST_DEFAULT = "192.168.15.99"
CAR_USER = "jz"
# topology_node.node_type 里「充电点」那一位：普通点 128，设成充电点后 132（2026-10-28 实测）
CHARGE_NODE_BIT = 4
CAR_TASK_PORT = 8002
# 迦智出厂默认口令（NERA-P200-DS3 用户手册「远程登录」一节）
CAR_PASS_DEFAULT = "robot123"
CAR_TOKEN_DEFAULT = "run_robot_run"
PC2_PASS_DEFAULT = "123456"
# 调度默认账号：旧版只有 admin/admin123（新版是 super_admin/super_admin123）
CLOUD_USER_DEFAULT = "admin"
CLOUD_PASS_DEFAULT = "admin123"
CAR_MAP_DIR = "/home/jz/map"
PC2_USER = "dexforce"
TC_YAML_CANDIDATES = (
    "/home/dexforce/workspace/dexe_application/services/traffic_control/config/traffic_control.yaml",
    "/home/dexforce/workspace/dexe_traffic_control_service/config/traffic_control.yaml",
)
OWN_PREFIX = "JZ_"
DATA_DIR_DEFAULT = os.path.join(os.path.expanduser("~"),
                                "dexe_robot_client_data", "jiazhi_roadnet")
CROSS_CHECK_TOL = 0.005  # 工程库与接口的坐标一致性阈值（米）


class Fail(Exception):
    """流程性失败：打印一行原因，不打堆栈。"""


def info(msg: str = "") -> None:
    print(msg, flush=True)


def warn(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def step(n: int, title: str) -> None:
    info(f"\n\033[1m── {n}. {title} ──\033[0m")


def confirm(question: str, assume_yes: bool) -> bool:
    if assume_yes:
        info(f"{question} [y/N] y（-y）")
        return True
    try:
        return input(f"{question} [y/N] ").strip().lower() in ("y", "yes")
    except EOFError:
        return False


def secret(env: str, default: str) -> str:
    return os.environ.get(env) or default


def _ssh_opts() -> list[str]:
    return [
        "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null",
        "-o", "LogLevel=ERROR", "-o", "ConnectTimeout=8"
    ]


def _need_sshpass() -> None:
    if shutil.which("sshpass") is None:
        raise Fail("找不到 sshpass（sudo apt install sshpass）")


# --------------------------------------------------------------------------- #
# HTTP（标准库，不依赖 requests / aiohttp）
# --------------------------------------------------------------------------- #
def _request(url: str,
             *,
             data: bytes | None = None,
             method: str = "GET",
             headers: dict[str, str] | None = None,
             timeout: float = 30.0):
    req = urllib.request.Request(url,
                                 data=data,
                                 method=method,
                                 headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = resp.read().decode("utf-8", "replace")
            return resp.status, body
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read().decode("utf-8", "replace")
    except urllib.error.URLError as exc:
        raise Fail(f"连不上 {url}：{exc.reason}") from exc
    except OSError as exc:
        raise Fail(f"连不上 {url}：{exc}") from exc


def login(base: str, username: str, password: str, timeout: float) -> str:
    body = json.dumps({"username": username, "password": password}).encode()
    status, text = _request(f"{base}/api/auth/login",
                            data=body,
                            method="POST",
                            headers={"Content-Type": "application/json"},
                            timeout=timeout)
    if status != 200:
        raise Fail(f"登录失败 HTTP {status}：{text[:200]}")
    body_ = json.loads(text) or {}
    # 旧版调度把响应包在 {code, message, data} 里，新版直接给 access_token
    if isinstance(body_, dict) and isinstance(body_.get("data"), dict):
        body_ = body_["data"]
    token = body_.get("access_token")
    if not token:
        raise Fail(f"登录响应里没有 access_token：{text[:200]}")
    return str(token)


def build_multipart(files: dict[str, str],
                    fields: dict[str, str]) -> tuple[bytes, str]:
    boundary = f"----jiazhi-upload-map-{uuid.uuid4().hex}"
    body = bytearray()
    for field, path in files.items():
        filename = os.path.basename(path)
        ctype = mimetypes.guess_type(filename)[0] or "application/octet-stream"
        body.extend(
            f'--{boundary}\r\nContent-Disposition: form-data; name="{field}"; '
            f'filename="{filename}"\r\nContent-Type: {ctype}\r\n\r\n'.encode())
        with open(path, "rb") as fh:
            body.extend(fh.read())
        body.extend(b"\r\n")
    for field, value in fields.items():
        body.extend(
            f'--{boundary}\r\nContent-Disposition: form-data; name="{field}"'
            f"\r\n\r\n{value}\r\n".encode())
    body.extend(f"--{boundary}--\r\n".encode())
    return bytes(body), boundary


# --------------------------------------------------------------------------- #
# 车端：map/info 与工程库
# --------------------------------------------------------------------------- #
def fetch_map_info(host: str, token: str, timeout: float) -> dict[str, Any]:
    """8002 任务接口：一律 HTTP 200，业务结果看响应体 code（int，0 为成功）。"""
    url = f"http://{host}:{CAR_TASK_PORT}/api/v1/map/info"
    status, text = _request(url,
                            headers={"X-Robot-Control-Token": token},
                            timeout=timeout)
    if status != 200:
        raise Fail(f"问底盘 {url} 失败：HTTP {status} {text[:200]}")
    try:
        payload = json.loads(text)
    except ValueError as exc:
        raise Fail(f"问底盘 {url} 返回的不是 JSON：{text[:200]}") from exc
    if payload.get("code") != 0:
        raise Fail(f"问底盘 {url} 失败：code={payload.get('code')} "
                   f"{payload.get('message') or ''}")
    data = payload.get("data") or {}
    if not data.get("map_project_name") or not data.get("layer_group_name"):
        raise Fail(f"map/info 没给工程名/图层组名：{json.dumps(payload)[:200]}")
    return payload


def fetch_car_db(host: str, project: str, out_dir: str) -> tuple[str, str]:
    """scp 车上的工程库回来，返回 (本地路径, md5)。

    车上那份会被网页编辑随时改写，所以拷完再跟车上 md5 比一次，
    对不上说明拷的过程中被改了，宁可报错重跑。
    """
    _need_sshpass()
    password = secret("JZ_CAR_PASS", CAR_PASS_DEFAULT)
    remote = f"{CAR_MAP_DIR}/{project}.db"
    base = ["sshpass", "-p", password]
    r = subprocess.run(base + [
        "ssh", *_ssh_opts(), f"{CAR_USER}@{host}",
        f"stat -c '%y' {shlex.quote(remote)} && md5sum {shlex.quote(remote)}"
    ],
                       capture_output=True,
                       text=True,
                       timeout=30)
    if r.returncode != 0:
        raise Fail(f"车上读不到 {remote}：{(r.stderr or r.stdout).strip()[:200]}")
    mtime, md5_line = r.stdout.strip().splitlines()[-2:]
    car_md5 = md5_line.split()[0]

    os.makedirs(out_dir, exist_ok=True)
    local = os.path.join(out_dir, f"{project}.{_dt.date.today():%Y%m%d}.db")
    r = subprocess.run(
        base + ["scp", *_ssh_opts(), f"{CAR_USER}@{host}:{remote}", local],
        capture_output=True,
        text=True,
        timeout=120)
    if r.returncode != 0:
        raise Fail(f"scp {remote} 失败：{r.stderr.strip()[:200]}")
    with open(local, "rb") as fh:
        md5 = hashlib.md5(fh.read()).hexdigest()
    if md5 != car_md5:
        raise Fail(f"拷回的工程库 md5 {md5} 与车上 {car_md5} 不符（拷贝期间被改写？），请重跑")
    info(f"工程库   : {CAR_USER}@{host}:{remote}（车上修改时间 {mtime[:19]}）")
    info(f"           -> {local}  md5 {md5}")
    return local, md5


def node_names(db_path: str, node_ids: list[int]) -> dict[int, str]:
    """工程库里的点位名；没有名字的点用点号本身。"""
    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    try:
        rows = dict(
            conn.execute("select node_id, name from topology_node").fetchall())
    finally:
        conn.close()
    return {n: (str(rows.get(n) or "").strip() or str(n)) for n in node_ids}


def charge_nodes(db_path: str, node_ids: list[int]) -> set[int]:
    """工程库里被设成「充电点」的点位（`node_type` 带 CHARGE_NODE_BIT）。"""
    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    try:
        rows = dict(
            conn.execute(
                "select node_id, node_type from topology_node").fetchall())
    finally:
        conn.close()
    return {n for n in node_ids if int(rows.get(n) or 0) & CHARGE_NODE_BIT}


def node_no(node_id: int, charges: set[int]) -> str:
    return f"{'C' if node_id in charges else 'S'}{node_id:03d}"


def check_unique_names(names: dict[int, str]) -> None:
    dupes = sorted(
        {v for v in names.values() if list(names.values()).count(v) > 1})
    if dupes:
        raise Fail(f"工程库里有重名点位 {dupes}，调度站点名会冲突，先在迦智侧改名")


def print_roadnet(
    payload: dict[str, Any],
    names: dict[int, str],
    charges: set[int] = frozenset()) -> None:
    data = payload["data"]
    nodes = {
        int(n["node_id"]): n["pose"]["position"]
        for n in data.get("nodes") or []
    }
    edges = data.get("edges") or []
    info(
        f"车端路网 : 工程 \033[1m{data['map_project_name']}\033[0m  "
        f"图层组 \033[1m{data['layer_group_name']}\033[0m  —— {len(nodes)} 点 / {len(edges)} 边"
    )
    info(f"  {'点号':>4}  {'调度编号':>6}  {'名称':<10} {'x':>9} {'y':>9}")
    for i in sorted(nodes):
        p = nodes[i]
        tag = "  充电点" if i in charges else ""
        info(
            f"  {i:>6}  {node_no(i, charges)}      {names[i]:<10} {p['x']:>9.3f} {p['y']:>9.3f}{tag}"
        )
    info("  边: " +
         "  ".join(f"{e['source_node']}-{e['dest_node']}" for e in edges))


# --------------------------------------------------------------------------- #
# 栅格图（CLOUDIA 工程库）
#
# 迦智栅格图的横轴是世界 y、纵轴是世界 x。设 laser_map_png.pose 前两个数为
# (ox, oy)、分辨率 r，原 PNG 宽高 W/H：
#     col = ox − y / r        row = oy − x / r
# 转成 ROS 约定要顺时针旋转 90°（numpy.rot90(a, k=-1)），宽高互换，且
#     Ox = (oy − H) · r       Oy = (ox − W) · r
# 用 laser_submap_info.max 独立复核：Ox == max_x − W'·r、Oy == max_y − H'·r。
# 推导见 jiazhi_client tools/export_map.py 模块文档 §2。
# --------------------------------------------------------------------------- #
class MapLayer:
    """工程库里一个图层组的栅格图 + 拓扑。`png` 是库里原样的字节。"""

    def __init__(self,
                 group_name: str,
                 png: bytes,
                 width: int,
                 height: int,
                 resolution: float,
                 origin_px: tuple[float, float],
                 nodes: dict[int, tuple[float, float]],
                 submap_max: tuple[float, float] | None = None):
        self.group_name = group_name
        self.png = png
        self.width = width  # 原 PNG 宽 = 世界 y 方向格数
        self.height = height  # 原 PNG 高 = 世界 x 方向格数
        self.resolution = resolution
        self.origin_px = origin_px
        self.nodes = nodes
        self.submap_max = submap_max  # Cartographer MapLimits.max，用于复核

    @property
    def ros_size(self) -> tuple[int, int]:
        """旋转后的 (宽, 高)：宽高互换。"""
        return self.height, self.width

    @property
    def origin(self) -> tuple[float, float]:
        """ROS 约定的 origin：图像左下角的世界坐标。"""
        ox, oy = self.origin_px
        return ((oy - self.height) * self.resolution,
                (ox - self.width) * self.resolution)

    def ros_image(self) -> bytes:
        """顺时针旋转 90°，重新编码成 PNG。"""
        try:
            import numpy as np
            from PIL import Image
        except ImportError as exc:
            raise Fail(f"旋转栅格图需要 numpy 和 Pillow：{exc}") from exc
        arr = np.array(Image.open(io.BytesIO(self.png)))
        out = Image.fromarray(np.rot90(arr, k=-1))
        buf = io.BytesIO()
        out.save(buf, format="PNG", optimize=True)
        return buf.getvalue()

    def check_against_submap(self) -> list[str]:
        """用 Cartographer 的 max 复核 origin。对不上说明这套推导在这张图上不成立。"""
        if self.submap_max is None:
            return ["工程库里没有 laser_submap_info，origin 无法独立复核"]
        max_x, max_y = self.submap_max
        w, h = self.ros_size
        want = (max_x - w * self.resolution, max_y - h * self.resolution)
        got = self.origin
        bad = []
        for axis, g, e in (("x", got[0], want[0]), ("y", got[1], want[1])):
            if abs(g - e) > 1e-6:
                bad.append(f"origin_{axis} 由 pose 算出 {g:.4f}，"
                           f"由 Cartographer max 算出 {e:.4f}，差 {g - e:+.4f} m")
        return bad


def read_layers(db_path: str) -> dict[str, MapLayer]:
    """读出工程库里所有带栅格图的图层组。

    layer_group_id / layer_id 都不连号，所以一律从 layer_group 出发查，不按 id 做算术。
    """
    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    conn.row_factory = sqlite3.Row
    layers: dict[str, MapLayer] = {}
    try:
        groups = conn.execute(
            "select * from layer_group where laser_layer_id > 0").fetchall()
        for group in groups:
            png_row = conn.execute(
                "select * from laser_map_png where laser_layer_id = ?",
                (group["laser_layer_id"],)).fetchone()
            if png_row is None:
                continue
            pose = [float(v) for v in str(png_row["pose"]).split()]
            sub = conn.execute(
                "select max from laser_submap_info where laser_layer_id = ?",
                (group["laser_layer_id"],)).fetchone()
            submap_max = None
            if sub is not None and sub["max"]:
                mx = [float(v) for v in str(sub["max"]).split()[:2]]
                submap_max = (mx[0], mx[1])
            nodes: dict[int, tuple[float, float]] = {}
            node_ids = [
                r[0] for r in conn.execute(
                    "select node_id from topo_node_info where topo_layer_id = ?",
                    (group["topology_layer_id"],))
            ]
            for node_id in node_ids:
                row = conn.execute(
                    "select pose from topology_node where node_id = ?",
                    (node_id,)).fetchone()
                if row is None:
                    continue
                xy = [float(v) for v in str(row["pose"]).split()[:2]]
                nodes[int(node_id)] = (xy[0], xy[1])
            layers[str(group["layer_group_name"])] = MapLayer(
                group_name=str(group["layer_group_name"]),
                png=bytes(png_row["map_png"]),
                width=int(png_row["width"]),
                height=int(png_row["height"]),
                resolution=float(png_row["resolution"]),
                origin_px=(pose[0], pose[1]),
                nodes=nodes,
                submap_max=submap_max)
    finally:
        conn.close()
    return layers


def png_size(blob: bytes) -> tuple[int, int]:
    """从 IHDR 读 PNG 宽高，用来核对工程库里记的 width/height 没写错。"""
    if blob[:8] != b"\x89PNG\r\n\x1a\n":
        raise Fail("laser_map_png.map_png 不是 PNG")
    width, height = struct.unpack(">II", blob[16:24])
    return int(width), int(height)


def build_map_meta(name: str, zone_id: str, workspace_id: str, layer: MapLayer,
                   png_bytes: bytes, png_md5: str) -> dict[str, Any]:
    """MapFile 风格的地图元信息（yaml.json），字段与 jiazhi_client export_map 一致。"""
    origin_x, origin_y = layer.origin
    width, height = layer.ros_size
    return {
        "dataType": "MAP_FILE",
        "id": str(uuid.uuid5(uuid.NAMESPACE_URL, f"map:{zone_id}:{name}")),
        "workspaceId": workspace_id or "",
        "name": name,
        "parentZoneIds": [],
        "zoneId": zone_id,
        "mapFileType": "RADAR",
        "uri": f"{name}.png",
        "size": len(png_bytes),
        "md5": png_md5,
        "origin": {
            "x": round(origin_x, 4),
            "y": round(origin_y, 4)
        },
        "resolution": round(layer.resolution, 6),
        "width": width,
        "height": height,
        "default": True,
        "disabled": False,
        "deleted": False,
        "isNew": False,
        "imageData": f"{name}.png",
    }


# --------------------------------------------------------------------------- #
# 路网（graph.json）
#
# graph.json 会被调度原样（tc_graph）转交给交管，所以：
# - 双向路段两个方向都显式写出来，交管不会自己补；
# - 每条边都带 `s`（路段正向起点），缺了交管 get_lines 会 KeyError: 's'；
# - graph 段的连通块字段交管直接下标取，缺了第一条 MOVE 就 KeyError。
# 边按无向处理（map/info 每条路段只给一个方向，实测双向可走）。
# --------------------------------------------------------------------------- #
def wrap_angle(angle: float) -> float:
    """把角度归一化到 (-pi, pi]"""
    out = math.fmod(angle + math.pi, 2.0 * math.pi)
    if out <= 0.0:
        out += 2.0 * math.pi
    return out - math.pi


def quaternion_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def parse_map_info(
    payload: dict[str, Any]
) -> tuple[dict[int, tuple[float, float, float]], dict[int, list[int]]]:
    """返回 (节点 {id: (x, y, yaw)}, 有向邻接 {src: [dst...]})，邻接保持接口给出的顺序。"""
    data = payload.get("data") if "data" in payload else payload
    data = data or {}
    nodes: dict[int, tuple[float, float, float]] = {}
    for item in data.get("nodes") or []:
        node_id = item.get("node_id")
        if node_id is None:
            continue
        pose = item.get("pose") or {}
        pos = pose.get("position") or {}
        ori = pose.get("orientation") or {}
        nodes[int(node_id)] = (float(pos.get("x") or
                                     0.0), float(pos.get("y") or 0.0),
                               quaternion_to_yaw(float(ori.get("w") or 1.0),
                                                 float(ori.get("x") or 0.0),
                                                 float(ori.get("y") or 0.0),
                                                 float(ori.get("z") or 0.0)))
    adjacency: dict[int, set[int]] = {}
    for item in data.get("edges") or []:
        src, dst = item.get("source_node"), item.get("dest_node")
        if src is None or dst is None:
            continue
        adjacency.setdefault(int(src), set()).add(int(dst))
    return nodes, {u: sorted(vs) for u, vs in adjacency.items()}


def connected_components(nodes: dict[int, Any],
                         adjacency: dict[int, list[int]]) -> list[list[int]]:
    """按**无向**连通性分组（按强连通算会把双向路段拆成一堆单点块）。"""
    undirected: dict[int, set[int]] = {n: set() for n in nodes}
    for u, vs in adjacency.items():
        for v in vs:
            undirected.setdefault(u, set()).add(v)
            undirected.setdefault(v, set()).add(u)
    seen: set[int] = set()
    groups: list[list[int]] = []
    for start in sorted(undirected):
        if start in seen:
            continue
        stack, group = [start], []
        seen.add(start)
        while stack:
            cur = stack.pop()
            group.append(cur)
            for nxt in undirected.get(cur, ()):
                if nxt not in seen:
                    seen.add(nxt)
                    stack.append(nxt)
        groups.append(sorted(group))
    return sorted(groups, key=len, reverse=True)


def _edge_dict(s: int, u: int, v: int, name: str, length: float,
               dir_: float) -> dict[str, Any]:
    return {
        "type": "NORMAL",
        "vertex": [],
        "length": round(length, 4),
        "weight": round(length, 4),
        "s": s,  # 路段正向起点，正反两条边一样
        "u": u,
        "v": v,
        "source": u,
        "target": v,
        "name": name,
        "dir": round(dir_, 6),
    }


def build_graph(
    payload: dict[str, Any],
    names: dict[int, str],
    graph_name: str,
    charges: set[int] = frozenset()
) -> tuple[dict[str, Any], dict[str, Any]]:
    """map/info + 点位名 -> (graph.json, node_map.yaml)：STATION S{id:03d}，充电点 CHARGE C{id:03d}。"""
    nodes, adjacency = parse_map_info(payload)
    no_of = {i: node_no(i, charges) for i in nodes}

    node_dicts = []
    for i in sorted(nodes):
        x, y, yaw = nodes[i]
        node_dicts.append({
            "no": no_of[i],
            "name": names[i],
            "type": "CHARGE" if i in charges else "STATION",
            "coord": [round(x, 4), round(y, 4)],
            "angle": round(wrap_angle(yaw), 6),
            "parkable": True,
            "conflicts": [],
            "narrow": False,
            "n": i,
            "id": i,
        })

    forward: list[tuple[int, int]] = []
    seen: set[tuple[int, int]] = set()
    for u, vs in adjacency.items():
        for v in vs:
            key = tuple(sorted((u, v)))
            if key in seen:
                continue
            seen.add(key)
            forward.append((u, v))

    edges, written = [], set()
    segs = []
    for u, v in forward:
        (ux, uy, _), (vx, vy, _) = nodes[u], nodes[v]
        name = (f"{names.get(u, no_of.get(u, str(u)))}__"
                f"{names.get(v, no_of.get(v, str(v)))}")
        length = math.hypot(vx - ux, vy - uy)
        dir_ = math.atan2(vy - uy, vx - ux)
        segs.append((u, v, name, length, dir_))
        edges.append(_edge_dict(u, u, v, name, length, dir_))
        written.add((u, v))
    for u, v, name, length, dir_ in segs:
        if (v, u) not in written:
            back = math.atan2(math.sin(dir_ + math.pi),
                              math.cos(dir_ + math.pi))
            edges.append(_edge_dict(u, v, u, name, length, back))
            written.add((v, u))

    comps = connected_components(nodes, adjacency) or [sorted(nodes)]
    idx = {str(n): i for i, cc in enumerate(comps) for n in cc}
    # 交管用 md5 判路网是否变了
    digest = json.dumps(
        {
            "name": graph_name,
            "nodes": node_dicts,
            "edges": edges
        },
        ensure_ascii=False,
        sort_keys=True)
    graph = {
        "directed": True,
        "multigraph": False,
        "graph": {
            "name": graph_name,
            "conflicts": {},
            "restriction": [],
            "md5": hashlib.md5(digest.encode("utf-8")).hexdigest(),
            "number_connected_components": len(comps),
            "connected_components": comps,
            "connected_components_to_idx": idx,
            "holdDirection": False,
        },
        "nodes": node_dicts,
        "edges": edges,
    }
    node_map = {
        "version":
            1,
        "graph_name":
            graph_name,
        # 与 jiazhi_client NodeMap.to_dict 同序：先按前缀、再按点号
        "nodes": [{
            "unified_node": no_of[i],
            "jiazhi_node": i,
            "name": names[i]
        } for i in sorted(nodes, key=lambda i: (no_of[i][0], i))],
    }
    return graph, node_map


def cross_check(payload: dict[str, Any], layer: MapLayer,
                tol: float) -> list[str]:
    """工程库拓扑 vs 接口路网。只报告不修正：哪边对需要人判断。"""
    api_nodes, _ = parse_map_info(payload)
    problems: list[str] = []
    only_api = sorted(set(api_nodes) - set(layer.nodes))
    only_db = sorted(set(layer.nodes) - set(api_nodes))
    if only_api:
        problems.append(f"接口有而工程库没有的节点: {only_api}")
    if only_db:
        problems.append(f"工程库有而接口没有的节点: {only_db}")
    for node_id in sorted(set(api_nodes) & set(layer.nodes)):
        ax, ay, _ = api_nodes[node_id]
        bx, by = layer.nodes[node_id]
        dist = math.hypot(ax - bx, ay - by)
        if dist > tol:
            problems.append(
                f"节点 {node_id} 坐标差 {dist * 1000:.1f}mm "
                f"(接口 {ax:+.4f},{ay:+.4f} / 工程库 {bx:+.4f},{by:+.4f})")
    return problems


def export_files(payload: dict[str, Any], db_path: str, names: dict[int, str],
                 name: str, zone_id: str, workspace_id: str,
                 out_dir: str) -> dict[str, str]:
    """导出 png / yaml.json / graph.json / node_map.yaml / map_info.json，返回路径。"""
    data = payload["data"]
    group = str(data["layer_group_name"])
    layers = read_layers(db_path)
    layer = layers.get(group)
    if layer is None:
        raise Fail(f"工程库里没有图层组 {group!r}。可选: {sorted(layers)}")
    real_w, real_h = png_size(layer.png)
    if (real_w, real_h) != (layer.width, layer.height):
        warn(f"[注意] 工程库记的尺寸 {layer.width}x{layer.height} 与 PNG 实际 "
             f"{real_w}x{real_h} 不符，按 PNG 实际值走。")
        layer.width, layer.height = real_w, real_h

    problems = cross_check(payload, layer, CROSS_CHECK_TOL)
    if problems:
        warn(f"[注意] 工程库图层组 {group!r} 与接口路网不一致：")
        for line in problems:
            warn(f"       - {line}")
        warn("       可能是工程库不是车上当前那份。")
    for line in layer.check_against_submap():
        warn(f"[注意] origin 复核不通过：{line}")

    os.makedirs(out_dir, exist_ok=True)
    paths = {
        k: os.path.join(out_dir, f"{name}.{ext}")
        for k, ext in (("png_file", "png"), ("yaml_file", "yaml.json"),
                       ("graph_file", "graph.json"),
                       ("node_map", "node_map.yaml"), ("map_info",
                                                       "map_info.json"))
    }

    with open(paths["map_info"], "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=1)
    ros_png = layer.ros_image()
    with open(paths["png_file"], "wb") as fh:
        fh.write(ros_png)
    meta = build_map_meta(name, zone_id, workspace_id, layer, ros_png,
                          hashlib.md5(ros_png).hexdigest())
    with open(paths["yaml_file"], "w", encoding="utf-8") as fh:
        json.dump(meta, fh, ensure_ascii=False, indent=4)
    charges = charge_nodes(db_path, sorted(names))
    graph, node_map = build_graph(payload, names, name, charges)
    with open(paths["graph_file"], "w", encoding="utf-8") as fh:
        json.dump(graph, fh, ensure_ascii=False, indent=1)
    with open(paths["node_map"], "w", encoding="utf-8") as fh:
        yaml.safe_dump(node_map, fh, allow_unicode=True, sort_keys=False)

    ox, oy = layer.origin
    out_w, out_h = layer.ros_size
    info(f"地图名   : {name}")
    info(f"栅格图   : 库里 {layer.width}x{layer.height}（横轴 y 纵轴 x）"
         f" -> 顺时针 90° -> {out_w}x{out_h} @ {layer.resolution:.4f} m/px，"
         f"origin ({ox:.4f}, {oy:.4f})")
    info(f"路网     : 节点 {len(graph['nodes'])}，边 {len(graph['edges'])}（含反向）")
    comps = graph["graph"]["connected_components"]
    if len(comps) > 1:
        warn(f"[注意] 路网有 {len(comps)} 个连通块（大小 {[len(c) for c in comps]}），"
             f"跨块无法导航。")
    info("已写入:\n  " + "\n  ".join(paths.values()))
    return paths


# --------------------------------------------------------------------------- #
# 本地检查
# --------------------------------------------------------------------------- #
def preflight(png: str, meta_path: str, graph_path: str, name: str,
              zone_id: str) -> tuple[dict, list[str]]:
    """三件套自洽性检查。返回 (摘要, 问题列表)。"""
    problems: list[str] = []
    with open(meta_path, "r", encoding="utf-8") as fh:
        meta = json.load(fh)
    with open(graph_path, "r", encoding="utf-8") as fh:
        graph = json.load(fh)

    # 1) yaml.json 指向的图片名要和实际上传的 png 对得上
    uri = meta.get("uri") or meta.get("imageData") or ""
    if uri and uri != os.path.basename(png):
        problems.append(f"yaml.json 的 uri={uri!r} 与 png 文件名 "
                        f"{os.path.basename(png)!r} 不一致")
    if meta.get("zoneId") and meta["zoneId"] != zone_id:
        problems.append(f"yaml.json 的 zoneId={meta['zoneId']} 与本次上传的 "
                        f"zone_id={zone_id} 不一致")

    # 2) md5 / size 过期说明 png 换过而 yaml.json 没重新生成
    with open(png, "rb") as fh:
        blob = fh.read()
    if meta.get("md5") and meta["md5"] != hashlib.md5(blob).hexdigest():
        problems.append("yaml.json 的 md5 与 png 实际内容不符（png 改过没重新导出）")
    if meta.get("size") and int(meta["size"]) != len(blob):
        problems.append(f"yaml.json 的 size={meta['size']} 与 png 实际 "
                        f"{len(blob)} 字节不符")

    # 3) 路网节点要落在栅格图范围内
    width, height = int(meta.get("width") or 0), int(meta.get("height") or 0)
    res = float(meta.get("resolution") or 0.05)
    origin = meta.get("origin") or {}
    ox, oy = float(origin.get("x") or 0.0), float(origin.get("y") or 0.0)
    nodes = graph.get("nodes") or []
    outside = []
    for node in nodes:
        coord = node.get("coord") or [0.0, 0.0]
        col = (float(coord[0]) - ox) / res
        row = (height - 1) - (float(coord[1]) - oy) / res
        if not (0 <= col < width and 0 <= row < height):
            outside.append(node.get("no"))
    if outside:
        problems.append(f"{len(outside)}/{len(nodes)} 个节点落在栅格图之外: "
                        f"{outside[:10]}")

    # 4) 交管直接下标取的字段：缺了导入不报错，等第一条 MOVE 才 KeyError
    required = {
        "md5", "number_connected_components", "connected_components",
        "connected_components_to_idx"
    }
    missing = sorted(required - set(graph.get("graph") or {}))
    if missing:
        problems.append(f"graph.json 的 graph 段缺少交管必需的键: {missing}")
    edges = graph.get("edges") or []
    no_s = [f"{e.get('u')}-{e.get('v')}" for e in edges if "s" not in e]
    if no_s:
        problems.append(f"{len(no_s)} 条边缺 s 字段（交管 KeyError: 's'）: {no_s[:10]}")
    if not nodes:
        problems.append("graph.json 里一个节点都没有")
    if not edges:
        problems.append("graph.json 里一条边都没有，交管无法规划")
    stations = [n for n in nodes if n.get("type") == "STATION"]
    if not stations:
        problems.append("graph.json 里没有 STATION，调度无法把任务派到站点")

    summary = {
        "name": name,
        "zone_id": zone_id,
        "png": f"{os.path.basename(png)}  {len(blob)} 字节  {width}x{height}",
        "origin": f"({ox}, {oy})  resolution={res}",
        "nodes": f"{len(nodes)}（站点 {len(stations)}）",
        "edges": str(len(edges)),
    }
    return summary, problems


# --------------------------------------------------------------------------- #
# 云端身份
# --------------------------------------------------------------------------- #
def _parse_tc_yaml(content: str) -> dict[str, str]:
    cfg = yaml.safe_load(content) or {}
    return {
        "cloud_host": str((cfg.get("mqtt") or {}).get("host") or ""),
        "workspace_id": str(cfg.get("workspace_id") or ""),
        "entity_id": str(cfg.get("entity_id") or "")
    }


def read_pc2_config(
        host: str,
        candidates: tuple[str, ...] = TC_YAML_CANDIDATES) -> dict[str, str]:
    """pc2 的 traffic_control.yaml：调度地址 / 园区 / 机器人。本机有就直接读（在 pc2 上跑）。"""
    local = next((p for p in candidates if os.path.isfile(p)), None)
    if local:
        with open(local, "r", encoding="utf-8") as fh:
            out = _parse_tc_yaml(fh.read())
        info(f"pc2 配置 : 本机 {local}")
    else:
        _need_sshpass()
        password = secret("PC2_PASS", PC2_PASS_DEFAULT)
        paths = " ".join(shlex.quote(p) for p in candidates)
        remote = (
            f"for f in {paths}; do if [ -f \"$f\" ]; then echo \"$f\"; cat \"$f\"; exit 0; fi; "
            f"done; exit 3")
        r = subprocess.run([
            "sshpass", "-p", password, "ssh", *_ssh_opts(),
            f"{PC2_USER}@{host}", remote
        ],
                           capture_output=True,
                           text=True,
                           timeout=30)
        if r.returncode == 3:
            raise Fail(f"pc2 上找不到 traffic_control.yaml：{candidates}")
        if r.returncode != 0:
            raise Fail(f"读 pc2({host}) 配置失败：{r.stderr.strip()[:200]}")
        path, _, content = r.stdout.partition("\n")
        out = _parse_tc_yaml(content)
        info(f"pc2 配置 : {PC2_USER}@{host}:{path}")
    for key, value in out.items():
        info(f"  {key:12s} = {value}")
    return out


class Cloud:

    def __init__(self, base: str, user: str, timeout: float):
        self.base, self.user, self.timeout = base, user, timeout
        self.token = ""

    def login(self) -> None:
        password = secret("CLOUD_PASSWORD", CLOUD_PASS_DEFAULT)
        self.token = login(self.base, self.user, password, self.timeout)
        info(f"已登录   : {self.base}（{self.user}）")

    def get(self, path: str) -> Any:
        status, text = _request(
            f"{self.base}{path}",
            timeout=self.timeout,
            headers={"Authorization": f"Bearer {self.token}"})
        if status != 200:
            raise Fail(f"GET {path} -> HTTP {status}：{text[:200]}")
        body = json.loads(text)
        # 新版后端包一层 {code, data}，列表接口包一层 {items}
        if isinstance(body, dict) and {"code", "data"} <= body.keys():
            body = body["data"]
        if isinstance(body, dict) and "items" in body:
            body = body["items"]
        return body

    def active_map(self, zone_id: str) -> dict[str, Any] | None:
        for m in self.get("/api/maps/"):
            if m.get("zone_id") == zone_id and m.get("is_active") == "enable":
                return m
        return None


def graph_summary(m: dict[str, Any]) -> str:
    g = m.get("graph_json") or {}
    if isinstance(g, str):
        g = json.loads(g)
    return f"{len(g.get('nodes') or [])} 点 / {len(g.get('edges') or [])} 边"


def list_robots(cloud: Cloud) -> str:
    try:
        robots = cloud.get("/api/robots/")
    except Fail as exc:
        return f"    （列不出来：{exc}）"
    zones = {z.get("id"): z.get("name") for z in cloud.get("/api/cbs-zones/")}
    return "\n".join(
        f"    {r.get('robot_name') or r.get('name')!s:<12} {r.get('robot_id') or r.get('id')}"
        f"  区域 {zones.get(r.get('zone_id'), r.get('zone_id'))}  状态 {r.get('status')}"
        for r in robots) or "    （一台都没有）"


def resolve_zone(cloud: Cloud, entity_id: str, zone_id: str,
                 workspace_id: str) -> tuple[str, str, str]:
    """返回 (zone_id, zone_name, workspace_id)，顺带把归属打印出来给人看。"""
    if entity_id and not zone_id:
        try:
            robot = cloud.get(f"/api/robots/{entity_id}")
        except Fail as exc:
            raise Fail(f"调度上查不到机器人 {entity_id}（{exc}）\n"
                       f"  pc2 里的 entity_id 可能是旧调度的。调度上现有机器人：\n" +
                       list_robots(cloud) +
                       "\n  用 --entity-id <id> 或 --zone-id <区域id> 重跑") from exc
        info(
            f"  机器人   : {robot.get('robot_name') or robot.get('name')}  ({entity_id})"
            f"  状态 {robot.get('status')}，最后心跳 {robot.get('last_heartbeat')}")
        zone_id = str(robot.get("zone_id") or robot.get("zoneId") or "")
        robot_ws = str(robot.get("workspace_id") or "")
        if not zone_id:
            raise Fail(f"机器人 {entity_id} 在调度上没绑定区域；先在前端指定，或用 --zone-id")
        if workspace_id and robot_ws and robot_ws != workspace_id:
            info(
                f"  [注意] 配置的 workspace_id({workspace_id}) 与调度记录({robot_ws}) 不一致，以调度为准"
            )
        workspace_id = robot_ws or workspace_id
    elif zone_id:
        info("  区域由 --zone-id 指定，不按机器人查")
    else:
        raise Fail("既没有 entity_id 也没有 --zone-id，不知道往哪个区域传")

    zone = cloud.get(f"/api/cbs-zones/{zone_id}")
    workspace_id = str(zone.get("workspace_id") or workspace_id)
    try:
        ws_name = cloud.get(f"/api/workspaces/{workspace_id}").get("name")
    except Fail:
        ws_name = "?"
    info(f"  园区     : {ws_name}  ({workspace_id})")
    info(f"  区域     : {zone.get('name')}  ({zone_id})")
    return zone_id, str(zone.get("name")), workspace_id


def check_replace(active: dict[str, Any] | None, new_name: str,
                  assume_yes: bool) -> None:
    if active is None:
        info("  当前地图 : 该区域还没有生效地图")
        return
    info(f"  当前地图 : {active.get('name')}  [生效中]  {graph_summary(active)}"
         f"  更新于 {str(active.get('updated_at'))[:19]}")
    name = str(active.get("name") or "")
    if name == new_name or name.startswith(OWN_PREFIX):
        return
    info(f"  \033[33m[注意]\033[0m 当前生效的 {name} 不是本工具导出的（非 {OWN_PREFIX} 前缀），"
         f"导入会把它顶掉；同区域若还有别的车在用它，那些车的指令会错派且不报错。")
    if not confirm(f"  确认替换 {name}？", assume_yes):
        raise Fail("已取消；要传到别的区域用 --zone-id")


# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="sync_roadnet",
        description="把迦智车端当前路网同步到自研调度",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--car",
                    default=os.environ.get("CLOUDIA_HOST", ""),
                    help="底盘 IP，默认环境变量 CLOUDIA_HOST")
    ap.add_argument("--car-token",
                    default=os.environ.get("CLOUDIA_TOKEN") or
                    CAR_TOKEN_DEFAULT,
                    help="8002 接口 token，默认环境变量 CLOUDIA_TOKEN")
    ap.add_argument("--pc2",
                    default=os.environ.get("PC2_HOST", "192.168.20.21"),
                    help="本机没有 traffic_control.yaml 时，从这台 pc2 ssh 读")
    ap.add_argument("--no-pc2", action="store_true", help="不读 pc2，身份全靠命令行")
    ap.add_argument("--cloud-host",
                    default=os.environ.get("CLOUDIA_SCHED_IP", "").strip(),
                    help="调度 IP（端口固定 3001），默认取环境变量 CLOUDIA_SCHED_IP，"
                    f"没有则 {CLOUD_HOST_DEFAULT}")
    ap.add_argument("--workspace-id", default="")
    ap.add_argument("--entity-id", default="", help="调度侧机器人 id，用来查所在区域")
    ap.add_argument("--zone-id", default="", help="直接指定区域，不按机器人查")
    ap.add_argument("--name", default="", help="调度地图名，默认 JZ_<工程>_<图层组>")
    ap.add_argument("--cloud-user",
                    default=os.environ.get("CLOUD_USER", CLOUD_USER_DEFAULT))
    ap.add_argument("--data-dir",
                    default=DATA_DIR_DEFAULT,
                    help="工程库放 <data-dir>/car/，三件套放 <data-dir>/<zone_id>/")
    ap.add_argument("--dry-run", action="store_true", help="只导出和检查，不上传")
    ap.add_argument("-y", "--yes", action="store_true", help="所有确认自动为「是」")
    ap.add_argument("--timeout", type=float, default=30.0)
    args = ap.parse_args(argv)

    try:
        # ── 1. 车端 ──
        step(1, "车端当前使用的路网")
        if not args.car:
            raise Fail("不知道底盘 IP：--car 或环境变量 CLOUDIA_HOST")
        payload = fetch_map_info(args.car, args.car_token, 5.0)
        data = payload["data"]
        project, group = data["map_project_name"], data["layer_group_name"]
        db_path, _ = fetch_car_db(args.car, project,
                                  os.path.join(args.data_dir, "car"))
        node_ids = sorted(int(n["node_id"]) for n in data.get("nodes") or [])
        names = node_names(db_path, node_ids)
        check_unique_names(names)
        print_roadnet(payload, names, charges=charge_nodes(db_path, node_ids))
        if not confirm(f"\n是否同步当前 \033[1m{project}/{group}\033[0m 路网？",
                       args.yes):
            raise Fail("已取消")

        # ── 2. 云端 ──
        step(2, "调度上的园区 / 区域")
        src: dict[str, str] = {}
        if not args.no_pc2:
            try:
                src.update({
                    k: v for k, v in read_pc2_config(args.pc2).items() if v
                })
            except (Fail, subprocess.TimeoutExpired) as exc:
                info(f"  \033[33m[注意]\033[0m pc2 配置读不到（{exc}）")
                if not confirm("  不带 pc2 身份继续（需 --entity-id / --zone-id）？",
                               args.yes):
                    raise Fail(
                        "已取消；可用 --cloud-host / --entity-id / --zone-id 手动指定"
                    ) from exc
        cloud_host = args.cloud_host or CLOUD_HOST_DEFAULT
        cfg_host = src.get("cloud_host") or ""
        if cfg_host and cfg_host != cloud_host:
            info(
                f"  \033[33m[注意]\033[0m pc2 配置里的调度地址 {cfg_host} ≠ 当前调度 {cloud_host}："
                f"pc2 的配置已过时，按 {cloud_host} 走")
        info(f"  调度地址 = {cloud_host}")
        workspace_id = args.workspace_id or src.get("workspace_id") or ""
        entity_id = args.entity_id or ("" if args.zone_id else
                                       src.get("entity_id") or "")
        cloud = Cloud(f"http://{cloud_host}:{CLOUD_PORT}", args.cloud_user,
                      args.timeout)
        cloud.login()
        zone_id, zone_name, workspace_id = resolve_zone(cloud, entity_id,
                                                        args.zone_id,
                                                        workspace_id)
        name = args.name or f"{OWN_PREFIX}{project}_{group}"
        check_replace(cloud.active_map(zone_id), name, args.yes)

        # ── 3. 导出 + 上传 ──
        step(3, "导出三件套并同步到调度")
        out_dir = os.path.join(args.data_dir, zone_id)
        files = export_files(payload, db_path, names, name, zone_id,
                             workspace_id, out_dir)
        upload = {k: files[k] for k in ("graph_file", "yaml_file", "png_file")}
        summary, problems = preflight(upload["png_file"], upload["yaml_file"],
                                      upload["graph_file"], name, zone_id)
        if problems:
            raise Fail("本地检查未通过：\n  - " + "\n  - ".join(problems))
        info("\n本地检查通过。")

        if args.dry_run:
            info(f"\n--dry-run：产物在 {out_dir}/，未上传。")
            return 0

        info(f"\n即将上传到 \033[1m{cloud.base}\033[0m")
        info(f"  区域   : {zone_name}  ({zone_id})")
        info(f"  地图名 : {name}   —— 同区域同名地图会被覆盖，并设为生效路网")
        for key, value in summary.items():
            info(f"  {key:6s} : {value}")
        if not confirm("\n是否将当前转换的路网同步到调度？", args.yes):
            raise Fail(f"已取消，产物保留在 {out_dir}/")

        body, boundary = build_multipart(upload, {
            "map_name": name,
            "zone_id": zone_id
        })
        status, text = _request(
            f"{cloud.base}/api/maps/import",
            data=body,
            method="POST",
            timeout=120.0,
            headers={
                "Authorization": f"Bearer {cloud.token}",
                "Content-Type": f"multipart/form-data; boundary={boundary}"
            })
        if status != 200:
            raise Fail(f"导入失败 HTTP {status}：{text[:400]}")
        resp = json.loads(text)
        resp = resp.get("data", resp) if isinstance(resp, dict) else resp
        info(f"\n\033[32m✓ 已导入调度\033[0m: {resp.get('name')} ({resp.get('id')})")
        tc = resp.get("tc_sync") or {}
        if tc.get("ok"):
            info(f"\033[32m✓ 交管路网已更新\033[0m: {tc.get('message')}")
        elif tc:
            info(
                f"\033[33m! 交管路网更新失败\033[0m: {tc.get('message')} —— 地图已在调度上，交管还在用旧路网"
            )

        active = cloud.active_map(zone_id)
        if not active or active.get("name") != name:
            raise Fail(f"导入后区域生效地图是 {active and active.get('name')}，不是 {name}")
        info(f"复核     : 区域 {zone_name} 生效地图 = {name}，{graph_summary(active)}")
        info(f"产物     : {out_dir}/（node_map.yaml 记录调度站点编号 ↔ 迦智点号）")
        return 0
    except Fail as exc:
        print(f"\n\033[31m错误\033[0m: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n已中断", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
