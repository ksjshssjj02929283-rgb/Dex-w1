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


def get_td(cfg):
    if "tool_dynamics" not in cfg:
        cfg["tool_dynamics"] = {
            "tool_primary_enabled": True,
            "tool_secondary_enabled": True,
        }
    return cfg["tool_dynamics"]


def print_header():
    print("\n========= 末端工具配置 =========")
    print("")
    td = get_td(cfg)
    print(f"  tool_primary_enabled   : {'true' if td.get('tool_primary_enabled', False) else 'false'}")
    print(f"  tool_secondary_enabled : {'true' if td.get('tool_secondary_enabled', False) else 'false'}")
    print("")
    tools = [(k, v) for k, v in td.items() if k not in ("tool_primary_enabled", "tool_secondary_enabled")]
    if tools:
        print("  当前末端工具:")
        for name, params in tools:
            mass = params.get("mass_kg", "?")
            com = params.get("com_m", "?")
            print(f"    {name}: mass_kg={mass}, com_m={com}")
    else:
        print("  (无当前末端工具)")
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

    while True:
        clear_screen()
        print(f"\n========= 编辑末端工具: {name} =========")
        print(f"  1: mass_kg = {params.get('mass_kg', '?')}")
        print(f"  2: com_m   = {params.get('com_m', '?')}")
        print(f"  3: 删除此工具")
        print(f"  b: 返回上级")
        print("===========================")

        cmd = input("请输入选项: ").strip()

        if cmd.lower() == "b":
            break
        elif cmd == "1":
            v = input(f"新 mass_kg (当前={params.get('mass_kg')}): ").strip()
            try:
                params["mass_kg"] = float(v)
                save_config(cfg)
                print(f"[OK] mass_kg = {params['mass_kg']}")
            except ValueError:
                print("无效数字")
        elif cmd == "2":
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
            except ValueError:
                print("无效数字")
        elif cmd == "3":
            confirm = input(f"确认删除 '{name}'? (y/N): ").strip().lower()
            if confirm == "y":
                del td[name]
                save_config(cfg)
                print(f"[OK] 已删除 '{name}'")
                break
        else:
            print("无效输入")


def add_tool():
    td = get_td(cfg)
    print("\n========= 添加新末端工具 =========")
    name = input("末端工具名称: ").strip()
    if not name:
        print("取消操作")
        return

    try:
        mass = float(input("mass_kg: ").strip())
    except ValueError:
        print("无效数字")
        return

    raw = input("com_m (x y z): ").strip()
    try:
        parts = [float(x) for x in raw.split()]
        if len(parts) != 3:
            print("需要 3 个值")
            return
    except ValueError:
        print("无效数字")
        return

    td[name] = {"mass_kg": mass, "com_m": parts}
    save_config(cfg)
    print(f"[OK] 已添加 '{name}': mass_kg={mass}, com_m={parts}")


def run_interactive():
    while True:
        print_header()
        print("1: 启用/禁用 primary 工具补偿")
        print("2: 启用/禁用 secondary 工具补偿")
        print("3: 编辑已配置的末端工具")
        print("4: 添加新末端工具")
        print("q: 退出")
        print("===========================")

        cmd = input("请输入选项: ").strip()
        clear_screen()

        if cmd == "1":
            edit_enable("primary")
        elif cmd == "2":
            edit_enable("secondary")
        elif cmd == "3":
            edit_tool()
        elif cmd == "4":
            add_tool()
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