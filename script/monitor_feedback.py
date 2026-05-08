import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import curses
import time
import threading

class RobotServerMonitor(Node):
    def __init__(self):
        super().__init__("robot_server_monitor")
        self.subscription = self.create_subscription(
            String, "/feedback/robot_server_state", self.listener_callback, 10
        )
        self.latest_data = None
        self.last_update_time = 0
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        try:
            with self.lock:
                self.latest_data = json.loads(msg.data)
                self.last_update_time = time.time()
        except:
            pass

    def get_latest_data(self):
        with self.lock:
            return self.latest_data, self.last_update_time

def addstr_safe(stdscr, y, x, text, attr=0):
    try:
        max_y, max_x = stdscr.getmaxyx()
        if x < max_x and y < max_y:
            stdscr.addstr(y, x, str(text)[:max_x - x], attr)
    except:
        pass

def center_text(val, width):
    s = str(val)
    return s[:width].center(width) if len(s) <= width else s[:width]

def monitor(stdscr, node):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)
    
    IN_TABLE = {
        "joint_name", "joint_position", "joint_position_cmd", "joint_velocity", "joint_velocity_cmd", 
        "joint_torque", "joint_torque_cmd", "torque_limit_negative", "torque_limit_positive", 
        "motor_status", "motor_temperature", "motor_error_code", "joint_stiffness", "joint_damping", 
        "enable_butterworth_filter", "butterworth_filter_frequency", "enable_input_shaper", "input_shaper_frequency",
        "cart_stiffness", "cart_damping_ratio", "force_sensor",
        "chassis_status_word", "chassis_error_code_left", "chassis_error_code_right",
        "chassis_left_velocity", "chassis_right_velocity", "chassis_left_current", "chassis_right_current",
        "chassis_driver_temperature", "chassis_left_temperature", "chassis_right_temperature", "enable_chassis_shaper"
    }
    
    count = 0
    while True:
        data, _ = node.get_latest_data()
        JOINT_NAMES = data.get("joint_name", []) if data else []
        if count % 10 == 0:
            stdscr.clear()
            row = 0
            if data:
                ts = data.get("timestamp", 0)
                addstr_safe(stdscr, row, 0, f"=== Robot Monitor (TS: {ts}) (q: quit) ===", curses.A_BOLD)
                row += 1
                row_fields = []
                array_fields = []
                for k, v in data.items():
                    if k not in IN_TABLE and k != "timestamp":
                        if isinstance(v, list): array_fields.append((k, v))
                        else: row_fields.append(f"{k}: {v}")
                for i in range(0, len(row_fields), 3):
                    # 特殊处理：将 torso_pose_X_Z_RY 和 joint_streaming_bounds 放在同一行
                    line_fields = row_fields[i:i+3]
                    if i == 0:
                        # 查找这两个字段并尝试配对
                        tp = next((f for f in row_fields if "torso_pose_X_Z_RY" in f), None)
                        jsb = next((f for f in row_fields if "joint_streaming_bounds" in f), None)
                        if tp and jsb:
                            others = [f for f in row_fields if f != tp and f != jsb]
                            # 重新构建显示序列
                            display_fields = [f"{tp} | {jsb}"]
                            for j in range(0, len(others), 2):
                                display_fields.append(" | ".join(others[j:j+2]))
                            for line in display_fields:
                                addstr_safe(stdscr, row, 0, line); row += 1
                            break
                    addstr_safe(stdscr, row, 0, " | ".join(line_fields)); row += 1
                for k, v in array_fields:
                    addstr_safe(stdscr, row, 0, f"{k}: {v}"); row += 1
                row += 1
                pos = data.get("joint_position", [])
                if pos:
                    cols1 = [("ID", 3), ("Name", 10), ("PosFb", 8), ("PosCmd", 8), ("VelFb", 8), ("VelCmd", 8), ("TorqFb", 8), ("TorqCmd", 8)]
                    addstr_safe(stdscr, row, 0, "".join(n.center(w) for n, w in cols1), curses.A_UNDERLINE); row += 1
                    cp=data.get("joint_position_cmd",[]); fv=data.get("joint_velocity",[]); cv=data.get("joint_velocity_cmd",[]); ft=data.get("joint_torque",[]); ct=data.get("joint_torque_cmd",[])
                    for i in range(len(pos)):
                        v1 = [str(i), JOINT_NAMES[i] if i<len(JOINT_NAMES) else f"J{i}", f"{pos[i]:.3f}", f"{cp[i] if i<len(cp) else 0:.3f}", f"{fv[i] if i<len(fv) else 0:.3f}", f"{cv[i] if i<len(cv) else 0:.3f}", f"{ft[i] if i<len(ft) else 0:.1f}", f"{ct[i] if i<len(ct) else 0:.1f}"]
                        addstr_safe(stdscr, row, 0, "".join(center_text(v, w) for v, w in zip(v1, [c[1] for c in cols1]))); row += 1
                    row += 1
                    cols2 = [("ID", 3), ("Name", 10), ("Eth", 6), ("Err", 4), ("Temp", 6), ("TLim-", 6), ("TLim+", 6), ("JtKp", 7), ("JtKd", 7), ("Filter", 9), ("ZVD", 9)]
                    addstr_safe(stdscr, row, 0, "".join(n.center(w) for n, w in cols2), curses.A_UNDERLINE); row += 1
                    ms=data.get("motor_status",[]); me=data.get("motor_error_code",[]); mt=data.get("motor_temperature",[]); ln=data.get("torque_limit_negative",[]); lp=data.get("torque_limit_positive",[]); st=data.get("joint_stiffness",[]); da=data.get("joint_damping",[]); be=data.get("enable_butterworth_filter",[]); bh=data.get("butterworth_filter_frequency",[]); se=data.get("enable_input_shaper",[]); sh=data.get("input_shaper_frequency",[])
                    for i in range(len(pos)):
                        f_s = "Y" if (i<len(be) and be[i]) else "N"
                        z_s = "Y" if (i<len(se) and se[i]) else "N"
                        f_t = f"{f_s}|{bh[i] if i<len(bh) else 0:3.0f}"
                        z_d = f"{z_s}|{sh[i] if i<len(sh) else 0:3.0f}"
                        v2 = [str(i), JOINT_NAMES[i] if i<len(JOINT_NAMES) else f"J{i}", str(ms[i] if i<len(ms) else "N"), str(me[i] if i<len(me) else 0), f"{mt[i] if i<len(mt) else 0:.0f}", f"{ln[i] if i<len(ln) else 0:.1f}", f"{lp[i] if i<len(lp) else 0:.1f}", f"{st[i] if i<len(st) else 0:.1f}", f"{da[i] if i<len(da) else 0:.1f}", f_t, z_d]
                        addstr_safe(stdscr, row, 0, "".join(center_text(v, w) for v, w in zip(v2, [c[1] for c in cols2]))); row += 1
                    row += 1
                    ck=data.get("cart_stiffness",[]); cd=data.get("cart_damping_ratio",[]); fs=data.get("force_sensor",[])
                    if any([ck, cd, fs]):
                        cols3 = [("CartImpedance", 18), ("X/Fx", 10), ("Y/Fy", 10), ("Z/Fz", 10), ("RX/Mx", 10), ("RY/My", 10), ("RZ/Mz", 10)]
                        addstr_safe(stdscr, row, 0, "".join(n.center(w) for n, w in cols3), curses.A_UNDERLINE); row += 1
                        # 检查数据长度，如果只有6个元素则为单臂
                        sides = []
                        if len(ck) == 6 or len(cd) == 6 or len(fs) == 6:
                            sides = [("Arm", 0)]
                        else:
                            sides = [("Left", 0), ("Right", 6)]
                            
                        for side, off in sides:
                            for label, arr in [("Stiffness", ck), ("DampRatio", cd), ("ForceSensor", fs)]:
                                if not arr: continue
                                v3 = [f"{side} {label}"]
                                for j in range(6):
                                    idx = off + j; v3.append(f"{arr[idx]:.1f}" if idx < len(arr) else "0.0")
                                addstr_safe(stdscr, row, 0, "".join(center_text(v, w) for v, w in zip(v3, [c[1] for c in cols3]))); row += 1
                    row += 1
                    cs = data.get("chassis_status_word", "N/A"); el = data.get("chassis_error_code_left", 0); er = data.get("chassis_error_code_right", 0)
                    vl = data.get("chassis_left_velocity", 0); vr = data.get("chassis_right_velocity", 0); cl = data.get("chassis_left_current", 0); cr = data.get("chassis_right_current", 0)
                    td = data.get("chassis_driver_temperature", 0); tl = data.get("chassis_left_temperature", 0); tr = data.get("chassis_right_temperature", 0); sh = data.get("enable_chassis_shaper", 0)
                    cols4 = [("Chassis", 12), ("Status/Err", 12), ("Vel(RPM)", 12), ("Cur(A)", 12), ("Temp(C)", 12), ("ZVD", 6)]
                    addstr_safe(stdscr, row, 0, "".join(n.center(w) for n, w in cols4), curses.A_UNDERLINE); row += 1
                    addstr_safe(stdscr, row, 0, "".join(center_text(v, w) for v, w in zip(["Status", str(cs), "-", "-", f"{td:.1f}", "Y" if sh else "N"], [c[1] for c in cols4]))); row += 1
                    addstr_safe(stdscr, row, 0, "".join(center_text(v, w) for v, w in zip(["LeftWheel", str(el), f"{vl:.1f}", f"{cl:.2f}", f"{tl:.1f}", "-"], [c[1] for c in cols4]))); row += 1
                    addstr_safe(stdscr, row, 0, "".join(center_text(v, w) for v, w in zip(["RightWheel", str(er), f"{vr:.1f}", f"{cr:.2f}", f"{tr:.1f}", "-"], [c[1] for c in cols4]))); row += 1
            else:
                addstr_safe(stdscr, row, 0, "Waiting for data...")
            stdscr.refresh()
        count += 1
        if stdscr.getch() == ord("q"): break

def main():
    rclpy.init(); node = RobotServerMonitor()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try: curses.wrapper(monitor, node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == "__main__": main()
