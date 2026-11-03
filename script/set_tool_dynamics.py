#!/usr/bin/python3
import os
import sys
import yaml

CONFIG_PATH = os.path.expanduser("~/dexe_robot_server_data/robot_config.yaml")


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def load_config():
    if not os.path.exists(CONFIG_PATH):
        print(f"Config not found: {CONFIG_PATH}")
        sys.exit(1)
    with open(CONFIG_PATH, "r") as f:
        return yaml.safe_load(f)


def save_config(data):
    with open(CONFIG_PATH, "w") as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)


def print_restart_hint():
    print("[提示] 配置修改完成，请运行 ./set_w1.sh restart 重启服务后生效。")
    input("按 Enter 继续...")


def get_td(cfg):
    if "tool_dynamics" not in cfg:
        cfg["tool_dynamics"] = {
            "tool_primary_enabled": True,
            "tool_secondary_enabled": True,
        }
    return cfg["tool_dynamics"]


def resolve_tool_name(td, hand_type):
    if not hand_type or hand_type == "null":
        return None
    for tool_name in (hand_type, "Default"):
        params = td.get(tool_name)
        if not isinstance(params, dict):
            continue
        if is_urdf_tool(params) or (
            "mass_kg" in params
            and isinstance(params.get("com_m"), list)
            and len(params["com_m"]) == 3
        ):
            return tool_name
    return None


def is_urdf_tool(params):
    return params.get("use_urdf", False)


def resolve_tool_urdf(hand_type, side):
    version_family = cfg["version"][:3] + "x"
    return os.path.join(
        os.path.expanduser("~/.cache/dexe_data/extracted/end_effectors"),
        version_family, hand_type, f"{side}_hand", f"{side}_hand.urdf",
    )


def describe_effective_tool(td, config_key, enabled_key):
    if not td.get(enabled_key, False):
        return "已禁用"
    hand_type = cfg.get(config_key)
    tool_name = resolve_tool_name(td, hand_type)
    if tool_name:
        params = td[tool_name]
        side = "left" if config_key == "left_hand_type" else "right"
        source = f"URDF: {resolve_tool_urdf(hand_type, side)}" if is_urdf_tool(params) else "质量/质心点载荷"
        name = f"Default（{hand_type} 未匹配）" if tool_name == "Default" else tool_name
        return f"{name} · {source}"
    if not hand_type or hand_type == "null":
        return "未配置手型"
    return f"无有效配置（{hand_type} 未匹配且无 Default）"


def print_header():
    print("\n========= 末端工具配置 =========")
    print("")
    td = get_td(cfg)
    print(f"  robot_version          : {cfg['version']}")
    print(f"  tool_primary_enabled   : {'true' if td.get('tool_primary_enabled', False) else 'false'}")
    print(f"  tool_secondary_enabled : {'true' if td.get('tool_secondary_enabled', False) else 'false'}")
    print("")
    print("  当前生效工具:")
    print(f"    primary（左手）  : {describe_effective_tool(td, 'left_hand_type', 'tool_primary_enabled')}")
    print(f"    secondary（右手）: {describe_effective_tool(td, 'right_hand_type', 'tool_secondary_enabled')}")
    print("")
    tools = [(k, v) for k, v in td.items() if k not in ("tool_primary_enabled", "tool_secondary_enabled")]
    if tools:
        print("  所有末端工具:")
        for name, params in tools:
            if is_urdf_tool(params):
                print(f"    {name}: URDF 载荷（按版本自动解析）")
                print(f"      左手: {resolve_tool_urdf(name, 'left')}")
                print(f"      右手: {resolve_tool_urdf(name, 'right')}")
            else:
                mass = params.get("mass_kg", "?")
                com = params.get("com_m", "?")
                print(f"    {name}: 质量/质心点载荷, mass_kg={mass}, com_m={com}")
    else:
        print("  (无末端工具)")
    print("")


def edit_enable(which):
    td = get_td(cfg)
    key = f"tool_{which}_enabled"
    cur = td.get(key, False)
    print(f"\n当前: {key} = {'true' if cur else 'false'}")
    raw = input("新值 (true/false): ").strip().lower()
    if raw in ("true", "1", "yes", "on"):
        td[key] = True
    elif raw in ("false", "0", "no", "off"):
        td[key] = False
    else:
        print("无效输入，取消操作")
        return
    save_config(cfg)
    print(f"[OK] {key} = {'true' if td[key] else 'false'}")
    print_restart_hint()


def edit_tool_params(td, name, params, allow_delete=True):
    while True:
        clear_screen()
        print(f"\n========= 编辑末端工具: {name} =========")
        urdf_tool = is_urdf_tool(params)
        if urdf_tool:
            print(f"  URDF 路径按机器人版本 {cfg['version']} 自动解析")
            print(f"    左手: {resolve_tool_urdf(name, 'left')}")
            print(f"    右手: {resolve_tool_urdf(name, 'right')}")
        else:
            print(f"  1: mass_kg = {params.get('mass_kg', '?')}")
            print(f"  2: com_m   = {params.get('com_m', '?')}")
        if allow_delete:
            print("  3: 删除此工具")
        print(f"  4: 切换 use_urdf（当前={'true' if urdf_tool else 'false'}）")
        print(f"  b: 返回上级")
        print("  q: 退出程序")
        print("===========================")

        cmd = input("请输入选项: ").strip()

        if cmd.lower() == "b":
            break
        elif cmd.lower() == "q":
            print("退出")
            sys.exit(0)
        elif cmd == "4":
            params["use_urdf"] = not urdf_tool
            save_config(cfg)
            print(f"[OK] use_urdf = {'true' if params['use_urdf'] else 'false'}")
            print_restart_hint()
        elif cmd == "1" and not urdf_tool:
            v = input(f"新 mass_kg (当前={params.get('mass_kg')}): ").strip()
            try:
                params["mass_kg"] = float(v)
                save_config(cfg)
                print(f"[OK] mass_kg = {params['mass_kg']}")
                print_restart_hint()
            except ValueError:
                print("无效数字")
        elif cmd == "2" and not urdf_tool:
            print(f"当前 com_m = {params.get('com_m')}")
            raw = input("新 com_m (x y z): ").strip()
            try:
                parts = [float(x) for x in raw.split()]
                if len(parts) != 3:
                    print("需要 3 个值")
                    continue
                params["com_m"] = parts
                save_config(cfg)
                print(f"[OK] com_m = {parts}")
                print_restart_hint()
            except ValueError:
                print("无效数字")
        elif cmd == "3" and allow_delete:
            confirm = input(f"确认删除 '{name}'? (y/N): ").strip().lower()
            if confirm == "y":
                del td[name]
                save_config(cfg)
                print(f"[OK] 已删除 '{name}'")
                print_restart_hint()
                break
        else:
            print("无效输入")


def edit_current_tool():
    td = get_td(cfg)
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
    hand_type = cfg.get(config_key)
    tool_name = resolve_tool_name(td, hand_type)
    if tool_name == "Default":
        print(f"[INFO] {config_key}={hand_type or '未配置'} 未匹配，{arm} 当前使用 Default")
    elif not tool_name:
        print(f"{config_key}={hand_type or '未配置'} 未匹配，且没有 Default 配置")
        return

    params = td.get(tool_name)
    if not isinstance(params, dict):
        print(f"工具配置 '{tool_name}' 无效")
        return
    edit_tool_params(td, tool_name, params, allow_delete=False)


def edit_tool():
    td = get_td(cfg)
    tools = [(k, v) for k, v in td.items() if k not in ("tool_primary_enabled", "tool_secondary_enabled")]

    if not tools:
        print("\n无已配置的末端工具，请先添加。")
        return

    print("\n选择要编辑的末端工具:")
    for i, (name, params) in enumerate(tools):
        print(f"  {i+1}: {name}")
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

    name, params = tools[idx]
    edit_tool_params(td, name, params)


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
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    global cfg
    cfg = load_config()

    if args.list:
        print_header()
        return

    run_interactive()


if __name__ == "__main__":
    main()
