#!/usr/bin/python3
import math
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
        td["enabled"] = True
    elif raw in ("false", "0", "no", "off"):
        td["enabled"] = False
    else:
        print("无效输入，取消操作")
        return
    save_config(cfg)
    print(f"[OK] tool_dynamics_{which}.enabled = {td['enabled']}")
    print_restart_hint()


def edit_tool_params(td, name, params, arm, allow_delete=True):
    while True:
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
                params["mass_kg"] = mass
                save_config(cfg)
                print(f"[OK] mass_kg = {params['mass_kg']}")
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
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

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
    main()
