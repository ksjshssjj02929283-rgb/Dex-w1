#!/usr/bin/python3
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import stat
import sys
import tempfile
import yaml

CONFIG_PATH = os.path.expanduser("~/dexe_robot_server_data/robot_config.yaml")


def finite_number(value):
    try:
        number = float(value)
    except ValueError:
        raise argparse.ArgumentTypeError("必须是数字")
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("必须是有限数值，不能是 NaN 或无穷大")
    return number


def nonnegative_mass(value):
    number = finite_number(value)
    if number < 0:
        raise argparse.ArgumentTypeError("质量不能为负数")
    return number


def mapping_fields(node, path):
    if not isinstance(node, yaml.MappingNode):
        raise ValueError(f"{path} 必须是 YAML 映射")
    fields = {}
    for key, value in node.value:
        if not isinstance(key, yaml.ScalarNode) or key.tag != "tag:yaml.org,2002:str":
            raise ValueError(f"{path} 必须使用普通字符串键，不支持 YAML 合并键")
        if key.value in fields:
            raise ValueError(f"{path} 存在重复键: {key.value}")
        fields[key.value] = value
    return fields


def parse_edit_document(source):
    # YAML marks locate only the requested scalar values; surrounding text is untouched.
    # Anchors/aliases can share values across arms, so reject them rather than edit shared data.
    for token in yaml.scan(source):
        if isinstance(token, (yaml.tokens.AnchorToken, yaml.tokens.AliasToken)):
            raise ValueError("不支持 YAML 锚点或别名，请先展开为独立配置")
    root = yaml.compose(source, Loader=yaml.SafeLoader)
    return root


def prepare_cli_update(source, args):
    root = parse_edit_document(source)
    fields = mapping_fields(root, "配置根节点")
    table_name = f"tool_dynamics_{args.arm}"
    table = mapping_fields(fields.get(table_name), table_name)
    replacements = []
    changed = {}

    def replace_scalar(node, value, path):
        if not isinstance(node, yaml.ScalarNode) or node.style in ("|", ">"):
            raise ValueError(f"{path} 必须是单行标量")
        if node.start_mark.line != node.end_mark.line:
            raise ValueError(f"{path} 必须是单行标量")
        if node.start_mark.index == node.end_mark.index:
            raise ValueError(f"{path} 缺少数值，请先修复配置格式")
        text = str(value).lower() if isinstance(value, bool) else repr(value)
        replacements.append((node.start_mark.index, node.end_mark.index, text))

    if args.enabled is not None:
        enabled = args.enabled == "true"
        replace_scalar(table.get("enabled"), enabled, f"{table_name}.enabled")
        changed["enabled"] = enabled

    if args.tool is not None:
        if args.tool == "enabled" or args.tool not in table:
            names = ", ".join(name for name in table if name != "enabled")
            raise ValueError(f"{table_name} 中不存在工具 {args.tool!r}；可选工具: {names}")
        path = f"{table_name}.{args.tool}"
        params = mapping_fields(table[args.tool], path)
        mass_node = params.get("mass_kg")
        com_node = params.get("com_m")
        if not isinstance(mass_node, yaml.ScalarNode):
            raise ValueError(f"{path}.mass_kg 缺失或不是标量")
        if (not isinstance(com_node, yaml.SequenceNode) or len(com_node.value) != 3
                or not all(isinstance(node, yaml.ScalarNode) for node in com_node.value)):
            raise ValueError(f"{path}.com_m 必须是三个标量组成的序列")
        if args.mass_kg is not None:
            replace_scalar(mass_node, args.mass_kg, f"{path}.mass_kg")
            changed["mass_kg"] = args.mass_kg
        if args.com_m is not None:
            for node, value in zip(com_node.value, args.com_m):
                replace_scalar(node, value, f"{path}.com_m")
            changed["com_m"] = args.com_m

    updated = source
    for start, end, value in sorted(replacements, reverse=True):
        updated = updated[:start] + value + updated[end:]
    data = yaml.safe_load(updated)
    if args.tool is not None and not valid_tool_params(data[table_name][args.tool]):
        raise ValueError(f"{table_name}.{args.tool} 修改后的质量/质心无效，请同时提供合法值")
    return updated, changed


def prepare_tool_delete(source, arm, name):
    if name in ("Default", "enabled"):
        raise ValueError("不能删除 Default 或 enabled")
    root = parse_edit_document(source)
    table_name = f"tool_dynamics_{arm}"
    table = mapping_fields(root, "配置根节点").get(table_name)
    fields = mapping_fields(table, table_name)
    if name not in fields:
        raise ValueError(f"{table_name} 中不存在工具 {name!r}")
    index = next(i for i, (key, _) in enumerate(table.value) if key.value == name)
    key, value = table.value[index]

    if table.flow_style:
        # Remove the entry and one separating comma, leaving all other text in place.
        start, end = key.start_mark.index, value.end_mark.index
        if index + 1 < len(table.value):
            next_start = table.value[index + 1][0].start_mark.index
            comma = next(token for token in yaml.scan(source)
                         if isinstance(token, yaml.tokens.FlowEntryToken)
                         and end <= token.start_mark.index < next_start)
            end = comma.end_mark.index
        elif index > 0:
            previous_end = table.value[index - 1][1].end_mark.index
            comma = next(token for token in yaml.scan(source)
                         if isinstance(token, yaml.tokens.FlowEntryToken)
                         and previous_end <= token.start_mark.index < start)
            start = comma.start_mark.index
    else:
        start = source.rfind("\n", 0, key.start_mark.index) + 1
        if source[start:key.start_mark.index].strip():
            raise ValueError("删除工具需要普通 YAML 键格式")

        def content_end(node):
            # Block container end marks may include comments belonging to the next entry.
            if isinstance(node, yaml.MappingNode) and not node.flow_style and node.value:
                return content_end(node.value[-1][1])
            if isinstance(node, yaml.SequenceNode) and not node.flow_style and node.value:
                return content_end(node.value[-1])
            return node.end_mark.index

        end = content_end(value)
        if not source[:end].endswith("\n"):
            newline = source.find("\n", end)
            end = len(source) if newline < 0 else newline + 1
    updated = source[:start] + source[end:]
    expected = yaml.safe_load(source)
    del expected[table_name][name]
    if yaml.safe_load(updated) != expected:
        raise ValueError("无法在保留其他配置的情况下删除该工具，本次未保存")
    return updated, {"deleted": name}


def save_update(config_path, prepare):
    path = Path(config_path).expanduser().resolve(strict=True)
    # Both menu and CLI writers share this lock; always edit the latest file contents.
    with open(str(path) + ".lock", "a", encoding="utf-8") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        original = path.read_bytes()
        updated, changed = prepare(original.decode("utf-8"))
        temp_path = None
        try:
            with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.",
                                             delete=False) as temp:
                temp_path = temp.name
                os.fchmod(temp.fileno(), stat.S_IMODE(path.stat().st_mode))
                temp.write(updated.encode("utf-8"))
                temp.flush()
                os.fsync(temp.fileno())
            if path.read_bytes() != original:
                raise ValueError("配置已被其他程序修改，本次未保存，请重试")
            os.replace(temp_path, path)
            temp_path = None
        finally:
            if temp_path is not None:
                os.unlink(temp_path)
    return path, changed, yaml.safe_load(updated)


def update_from_cli(args):
    path, changed, _ = save_update(args.config, lambda source: prepare_cli_update(source, args))
    print(json.dumps({"ok": True, "config": str(path), "arm": args.arm,
                      "tool": args.tool, "updated": changed}, ensure_ascii=False))


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def load_config():
    if not os.path.exists(CONFIG_PATH):
        print(f"Config not found: {CONFIG_PATH}")
        sys.exit(1)
    with open(CONFIG_PATH, "r") as f:
        return yaml.safe_load(f)


def save_interactive(arm, tool=None, mass_kg=None, com_m=None, enabled=None, delete=False):
    global cfg
    args = argparse.Namespace(arm=arm, tool=tool, mass_kg=mass_kg, com_m=com_m,
                              enabled=None if enabled is None else str(enabled).lower())
    prepare = (lambda source: prepare_tool_delete(source, arm, tool)) if delete else (
        lambda source: prepare_cli_update(source, args))
    try:
        _, _, saved = save_update(CONFIG_PATH, prepare)
    except (ValueError, OSError, yaml.YAMLError) as exc:
        print(f"[ERROR] 保存失败: {exc}")
        input("按 Enter 继续...")
        return False
    cfg = saved
    return True


def print_restart_hint():
    print("[提示] 配置修改完成，请运行 ./set_w1.sh restart 重启服务后生效。")
    input("按 Enter 继续...")


def get_td(config, arm):
    key = f"tool_dynamics_{arm}"
    if not isinstance(config.get(key), dict):
        config[key] = {"enabled": True,
                       "Default": {"mass_kg": 0.0, "com_m": [0.0, 0.0, 0.0]}}
    return config[key]


def valid_tool_params(params):
    if not isinstance(params, dict):
        return False
    com = params.get("com_m")
    if not isinstance(com, list) or len(com) != 3:
        return False
    try:
        if any(isinstance(v, bool) for v in [params["mass_kg"], *com]):
            return False
        mass = float(params["mass_kg"])
        return math.isfinite(mass) and mass >= 0.0 and all(math.isfinite(float(v)) for v in com)
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def resolve_tool_name(td, hand_type):
    if hand_type and hand_type != "null" and valid_tool_params(td.get(hand_type)):
        return hand_type
    if valid_tool_params(td.get("Default")):
        return "Default"
    return None


def tool_enabled(td):
    value = td.get("enabled", False)
    if isinstance(value, bool):
        return value
    return isinstance(value, str) and value.lower() in ("true", "yes", "on", "y")


def describe_effective_tool(td, hand_type):
    if not tool_enabled(td):
        return "已禁用"
    tool_name = resolve_tool_name(td, hand_type)
    if not tool_name:
        return "无有效配置，使用零质量"
    params = td[tool_name]
    detail = f"mass_kg={params['mass_kg']}, com_m={params['com_m']}"
    if float(params["mass_kg"]) == 0.0:
        detail += "，零质量不产生补偿"
    if tool_name == "Default":
        return f"Default（手型为空/null、未匹配或参数无效；{detail}）"
    return f"{tool_name}（{detail}）"


def print_header():
    print("\n========= 末端工具配置 =========")
    for arm, config_key in (("primary", "left_hand_type"), ("secondary", "right_hand_type")):
        td = get_td(cfg, arm)
        print(f"\n  tool_dynamics_{arm}.enabled: {tool_enabled(td)}")
        print(f"  当前生效工具: {describe_effective_tool(td, cfg.get(config_key))}")
        print("  本侧工具参数:")
        for name, params in td.items():
            if name != "enabled":
                print(f"    {name}: {params}")
    print("")


def edit_enable(which):
    td = get_td(cfg, which)
    cur = tool_enabled(td)
    print(f"\n当前: tool_dynamics_{which}.enabled = {cur}")
    raw = input("新值 (true/false): ").strip().lower()
    if raw in ("true", "1", "yes", "on"):
        enabled = True
    elif raw in ("false", "0", "no", "off"):
        enabled = False
    else:
        print("无效输入，取消操作")
        return
    if not save_interactive(which, enabled=enabled):
        return
    print(f"[OK] tool_dynamics_{which}.enabled = {enabled}")
    print_restart_hint()


def edit_tool_params(td, name, params, arm, allow_delete=True):
    while True:
        # A successful save refreshes cfg from disk, including unrelated external edits.
        params = cfg[f"tool_dynamics_{arm}"].get(name)
        if not isinstance(params, dict):
            print(f"工具配置 '{arm}/{name}' 已不存在或无效")
            return
        clear_screen()
        print(f"\n========= 编辑末端工具: {arm}/{name} =========")
        print(f"  1: mass_kg = {params.get('mass_kg', '?')}")
        print(f"  2: com_m   = {params.get('com_m', '?')}")
        if allow_delete:
            print("  3: 删除此工具")
        print(f"  b: 返回上级")
        print("  q: 退出程序")
        print("===========================")

        cmd = input("请输入选项: ").strip()

        if cmd.lower() == "b":
            break
        elif cmd.lower() == "q":
            print("退出")
            sys.exit(0)
        elif cmd == "1":
            v = input(f"新 mass_kg (当前={params.get('mass_kg')}): ").strip()
            try:
                mass = float(v)
                if not math.isfinite(mass) or mass < 0.0:
                    print("质量必须是有限的非负数")
                    continue
                if not save_interactive(arm, tool=name, mass_kg=mass):
                    continue
                print(f"[OK] mass_kg = {mass}")
                print_restart_hint()
            except ValueError:
                print("无效数字")
        elif cmd == "2":
            print(f"当前 com_m = {params.get('com_m')}")
            raw = input("新 com_m (x y z): ").strip()
            try:
                parts = [float(x) for x in raw.split()]
                if len(parts) != 3 or not all(math.isfinite(v) for v in parts):
                    print("需要 3 个有限数值")
                    continue
                if not save_interactive(arm, tool=name, com_m=parts):
                    continue
                print(f"[OK] com_m = {parts}")
                print_restart_hint()
            except ValueError:
                print("无效数字")
        elif cmd == "3" and allow_delete:
            confirm = input(f"确认删除 '{name}'? (y/N): ").strip().lower()
            if confirm == "y":
                if not save_interactive(arm, tool=name, delete=True):
                    continue
                print(f"[OK] 已删除 '{name}'")
                print_restart_hint()
                break
        else:
            print("无效输入")


def edit_current_tool():
    current_tools = [
        ("primary", "left_hand_type"),
        ("secondary", "right_hand_type"),
    ]

    print("\n选择当前工具:")
    for i, (arm, config_key) in enumerate(current_tools):
        print(f"  {i+1}: {arm} ({config_key}={cfg.get(config_key, '未配置')})")
    print("  b: 返回")

    raw = input("请输入选项: ").strip()
    if raw.lower() == "b":
        return

    try:
        idx = int(raw) - 1
        if idx < 0 or idx >= len(current_tools):
            print("无效选择")
            return
    except ValueError:
        print("无效输入")
        return

    arm, config_key = current_tools[idx]
    td = get_td(cfg, arm)
    hand_type = cfg.get(config_key)
    tool_name = resolve_tool_name(td, hand_type)
    if tool_name == "Default":
        print(f"[INFO] {config_key}={hand_type or '未配置'} 为空/null、未匹配或参数无效，{arm} 当前使用本侧 Default")
    elif not tool_name:
        print(f"{config_key}={hand_type or '未配置'} 未匹配，且没有 Default 配置")
        return

    params = td.get(tool_name)
    if not isinstance(params, dict):
        print(f"工具配置 '{tool_name}' 无效")
        return
    edit_tool_params(td, tool_name, params, arm, allow_delete=False)


def edit_tool():
    tools = []
    for arm in ("primary", "secondary"):
        td = get_td(cfg, arm)
        tools.extend((arm, name, params) for name, params in td.items() if name != "enabled")

    if not tools:
        print("\n无已配置的末端工具，请先添加。")
        return

    print("\n选择要编辑的末端工具:")
    for i, (arm, name, params) in enumerate(tools):
        print(f"  {i+1}: {arm}/{name}")
    print("  b: 返回")

    raw = input("请输入选项: ").strip()
    if raw.lower() == "b":
        return

    try:
        idx = int(raw) - 1
        if idx < 0 or idx >= len(tools):
            print("无效选择")
            return
    except ValueError:
        print("无效输入")
        return

    arm, name, params = tools[idx]
    td = get_td(cfg, arm)
    if not isinstance(params, dict):
        print(f"工具配置 '{arm}/{name}' 无效")
        return
    edit_tool_params(td, name, params, arm, allow_delete=name != "Default")


def run_interactive():
    while True:
        print_header()
        print("1: 编辑当前生效的末端工具")
        print("2: 编辑选择的末端工具")
        print("3: 启用/禁用 primary 工具补偿")
        print("4: 启用/禁用 secondary 工具补偿")
        print("q: 退出")
        print("===========================")

        cmd = input("请输入选项: ").strip()
        clear_screen()

        if cmd == "1":
            edit_current_tool()
        elif cmd == "2":
            edit_tool()
        elif cmd == "3":
            edit_enable("primary")
        elif cmd == "4":
            edit_enable("secondary")
        elif cmd.lower() == "q":
            print("退出")
            break
        else:
            print("无效输入，请重新选择")


def main():
    global CONFIG_PATH
    parser = argparse.ArgumentParser(description="查看/编辑工具重力补偿；带修改参数时保存后直接退出")
    parser.add_argument("--config", default=CONFIG_PATH, help="配置文件路径（默认: %(default)s）")
    parser.add_argument("--list", action="store_true", help="仅显示配置")
    parser.add_argument("--arm", choices=("primary", "secondary"), help="primary 左手 / secondary 右手")
    parser.add_argument("--enabled", choices=("true", "false"), help="本侧补偿开关")
    parser.add_argument("--tool", help="本侧已有工具名称，区分大小写，例如 Default 或 BrainCo_Revo1_R")
    parser.add_argument("--mass-kg", type=nonnegative_mass, help="有限非负质量，单位 kg")
    parser.add_argument("--com-m", nargs=3, type=finite_number, metavar=("X", "Y", "Z"),
                        help="EE/法兰坐标系下的质心，单位 m，三个空格分隔的数值")
    args = parser.parse_args()

    modifying = any(value is not None for value in
                    (args.arm, args.enabled, args.tool, args.mass_kg, args.com_m))
    if modifying:
        if args.list:
            parser.error("--list 不能与修改参数同时使用")
        if args.arm is None:
            parser.error("修改配置必须提供 --arm")
        has_params = args.mass_kg is not None or args.com_m is not None
        if has_params and args.tool is None:
            parser.error("修改质量/质心必须提供 --tool")
        if args.tool is not None and not has_params:
            parser.error("--tool 必须与 --mass-kg 或 --com-m 一起使用")
        if args.enabled is None and not has_params:
            parser.error("请提供 --enabled、--mass-kg 或 --com-m")
        try:
            update_from_cli(args)
        except (ValueError, yaml.YAMLError) as exc:
            print(f"[ERROR] {exc}", file=sys.stderr)
            return 2
        except OSError as exc:
            print(f"[ERROR] {exc}", file=sys.stderr)
            return 1
        return 0

    CONFIG_PATH = os.path.expanduser(args.config)
    global cfg
    cfg = load_config()
    # 编辑器只依赖左右独立的工具参数表, 不依赖具体配置版本号。
    if not isinstance(cfg, dict) or not all(
        isinstance(cfg.get(f"tool_dynamics_{arm}"), dict) for arm in ("primary", "secondary")
    ):
        print("配置缺少左右独立的工具参数表，请重启 w1 服务更新配置或检查配置内容后再编辑。")
        sys.exit(1)

    if args.list:
        print_header()
        return

    run_interactive()


if __name__ == "__main__":
    sys.exit(main())
