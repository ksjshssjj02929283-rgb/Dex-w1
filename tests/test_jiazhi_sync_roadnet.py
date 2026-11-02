"""迦智路网同步脚本（features/cloudia/sync_roadnet.py）测试：合成工程库，无需车、调度和 ROS。

按文件路径加载模块，不经过 navigation_client/__init__ 的 rclpy 导入链。
"""

import importlib.util
import io
import json
import math
import pathlib
import sqlite3

import yaml

import pytest

np = pytest.importorskip("numpy")
Image = pytest.importorskip("PIL.Image")

_PATH = (pathlib.Path(__file__).resolve().parents[1] / "src" /
         "dexe_robot_client" / "navigation_client" / "features" / "cloudia" /
         "sync_roadnet.py")
_spec = importlib.util.spec_from_file_location("jiazhi_sync_roadnet", _PATH)
sr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sr)

RES = 0.05
W, H = 40, 30  # 原 PNG 宽（世界 y）/ 高（世界 x），故意非方形
OX, OY = 30.0, 20.0  # laser_map_png.pose 前两个数


def _quat(yaw):
    return {"w": math.cos(yaw / 2), "x": 0.0, "y": 0.0, "z": math.sin(yaw / 2)}


def _payload():
    pts = {
        2: (0.1, 0.2, 0.0),
        3: (0.5, 0.3, math.pi / 2),
        4: (0.6, -0.4, -math.pi),
        9: (0.2, -0.1, 0.0)
    }
    return {
        "code": 0,
        "data": {
            "map_project_name":
                "proj",
            "layer_group_name":
                "g1",
            "nodes": [{
                "node_id": i,
                "pose": {
                    "position": {
                        "x": x,
                        "y": y,
                        "z": 0
                    },
                    "orientation": _quat(yaw)
                }
            } for i, (x, y, yaw) in pts.items()],
            # 3-2 与 2-3 是同一路段；9 孤立 -> 两个连通块
            "edges": [{
                "source_node": 2,
                "dest_node": 3
            }, {
                "source_node": 3,
                "dest_node": 2
            }, {
                "source_node": 3,
                "dest_node": 4
            }],
        },
    }


def _png(w, h):
    arr = np.zeros((h, w), dtype=np.uint8)
    arr[0, 0] = 255  # 左上角做记号，用来验证旋转方向
    buf = io.BytesIO()
    Image.fromarray(arr).save(buf, format="PNG")
    return buf.getvalue()


@pytest.fixture
def db(tmp_path):
    path = tmp_path / "proj.db"
    conn = sqlite3.connect(path)
    conn.executescript("""
        create table layer_group(layer_group_name text, laser_layer_id int, topology_layer_id int);
        create table laser_map_png(laser_layer_id int, map_png blob, width int, height int,
                                   resolution real, pose text);
        create table laser_submap_info(laser_layer_id int, max text);
        create table topo_node_info(topo_layer_id int, node_id int);
        create table topology_node(node_id int, name text, pose text, node_type int);
    """)
    conn.execute("insert into layer_group values ('g1', 5, 7)")
    conn.execute("insert into laser_map_png values (5, ?, ?, ?, ?, ?)",
                 (_png(W, H), W, H, RES, f"{OX} {OY} 0"))
    # Cartographer max：max_y / r == pose[0]、max_x / r == pose[1]
    conn.execute("insert into laser_submap_info values (5, ?)",
                 (f"{OY * RES} {OX * RES}",))
    for i, name, (x, y) in ((2, "L1", (0.1, 0.2)), (3, "L2", (0.5, 0.3)),
                            (4, "", (0.6, -0.4)), (9, None, (0.2, -0.1))):
        conn.execute("insert into topo_node_info values (7, ?)", (i,))
        # 点 4 是充电点：Carly 设成「充电点」后 node_type 128 -> 132
        conn.execute("insert into topology_node values (?, ?, ?, ?)",
                     (i, name, f"{x} {y} 0", 132 if i == 4 else 128))
    conn.commit()
    conn.close()
    return str(path)


def _export(db, tmp_path):
    payload = _payload()
    ids = sorted(int(n["node_id"]) for n in payload["data"]["nodes"])
    names = sr.node_names(db, ids)
    files = sr.export_files(payload, db, names, "JZ_proj_g1", "zone-1", "ws-1",
                            str(tmp_path / "out"))
    return files, names


def test_names_fall_back_to_node_id(db):
    assert sr.node_names(db, [2, 3, 4, 9]) == {2: "L1", 3: "L2", 4: "4", 9: "9"}


def test_charge_node_exported_as_c_prefix(db, tmp_path):
    files, _ = _export(db, tmp_path)
    with open(files["graph_file"], encoding="utf-8") as fh:
        graph = json.load(fh)
    types = {n["no"]: n["type"] for n in graph["nodes"]}
    assert types == {
        "S002": "STATION",
        "S003": "STATION",
        "C004": "CHARGE",
        "S009": "STATION"
    }
    with open(files["node_map"], encoding="utf-8") as fh:
        node_map = yaml.safe_load(fh)
    assert {
        "unified_node": "C004",
        "jiazhi_node": 4,
        "name": "4"
    } in node_map["nodes"]


def test_duplicate_names_rejected():
    with pytest.raises(sr.Fail):
        sr.check_unique_names({1: "A", 2: "A"})


def test_png_rotated_clockwise_and_origin(db, tmp_path):
    files, _ = _export(db, tmp_path)
    img = np.array(Image.open(files["png_file"]))
    assert img.shape == (W, H)  # 宽高互换：新宽 = H，新高 = W
    assert img[0, H - 1] == 255  # 原左上角顺时针转到右上角
    meta = json.loads(pathlib.Path(files["yaml_file"]).read_text())
    assert (meta["width"], meta["height"]) == (H, W)
    assert meta["origin"] == {
        "x": round((OY - H) * RES, 4),
        "y": round((OX - W) * RES, 4)
    }
    assert meta["uri"] == "JZ_proj_g1.png" and meta["zoneId"] == "zone-1"


def test_origin_agrees_with_submap_max(db):
    layer = sr.read_layers(db)["g1"]
    assert layer.check_against_submap() == []


def test_graph_edges_have_s_and_both_directions(db, tmp_path):
    files, _ = _export(db, tmp_path)
    graph = json.loads(pathlib.Path(files["graph_file"]).read_text())
    nodes = {n["n"]: n for n in graph["nodes"]}
    assert nodes[3]["no"] == "S003" and nodes[3]["type"] == "STATION"
    assert nodes[4]["name"] == "4"
    pairs = {(e["u"], e["v"]): e for e in graph["edges"]}
    assert set(pairs) == {(2, 3), (3, 2), (3, 4), (4, 3)}  # 2-3 不重复
    assert all("s" in e for e in graph["edges"])
    assert pairs[(3, 2)]["s"] == pairs[(2, 3)]["s"] == 2
    assert pairs[(2, 3)]["name"] == "L1__L2"
    g = graph["graph"]
    assert g["number_connected_components"] == 2
    assert g["connected_components"] == [[2, 3, 4], [9]]
    assert g["connected_components_to_idx"]["9"] == 1


def test_preflight_passes_on_export(db, tmp_path):
    files, _ = _export(db, tmp_path)
    _, problems = sr.preflight(files["png_file"], files["yaml_file"],
                               files["graph_file"], "JZ_proj_g1", "zone-1")
    assert problems == []


def test_pc2_config_read_locally_without_ssh(tmp_path, monkeypatch):
    tc = tmp_path / "traffic_control.yaml"
    tc.write_text(
        "mqtt:\n  host: 192.168.15.210\nworkspace_id: ws\nentity_id: rb\n")

    def _no_ssh(*_a, **_k):
        raise AssertionError("本机有配置时不应 ssh")

    monkeypatch.setattr(sr.subprocess, "run", _no_ssh)
    out = sr.read_pc2_config("192.168.20.21",
                             candidates=("/nonexistent.yaml", str(tc)))
    assert out == {
        "cloud_host": "192.168.15.210",
        "workspace_id": "ws",
        "entity_id": "rb"
    }


def test_fetch_map_info_rejects_business_error(monkeypatch):
    monkeypatch.setattr(
        sr, "_request", lambda *a, **k:
        (200, '{"code": 1006, "message": "token"}'))
    with pytest.raises(sr.Fail, match="1006"):
        sr.fetch_map_info("127.0.0.1", "bad", 1.0)


def test_runs_by_file_path_despite_sibling_http_module():
    # 同目录有 http.py：直接按路径跑时若不摘掉本目录，urllib.request 会导入失败
    import subprocess
    import sys
    r = subprocess.run([sys.executable, str(_PATH), "--help"],
                       capture_output=True,
                       text=True,
                       cwd=str(_PATH.parent),
                       timeout=30)
    assert r.returncode == 0, r.stderr
