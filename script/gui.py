import os
import signal
import subprocess
import math
import tkinter as tk
from PIL import Image, ImageTk

class LauncherApp:
    def __init__(self, root):
        self.root = root
        root.title("Robot Launcher")
        root.attributes('-fullscreen', True)

        # 脚本配置
        self.scripts = {
            "bull":   {"name": "Bull",   "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/bulls/",   "image": "img/bull.jpg"},
            "gun":    {"name": "Gun",    "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/guns/",    "image": "img/gun.jpg"},
            "heart":  {"name": "Heart",  "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/hearts/",  "image": "img/heart.jpg"},
            "like":   {"name": "Like",   "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/likes/",   "image": "img/like.jpg"},
            "point":  {"name": "Point",  "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/points/",  "image": "img/point.jpg"},
            "salute": {"name": "Salute", "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/salute/", "image": "img/salute.jpg"},
            "six":    {"name": "Six",    "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/sixs/",   "image": "img/six.jpg"},
            "all":    {"name": "All",    "cmd": "python shuffle_replay_in_folder.py records/WAIC_DEMOS/", "image": "img/robot.jpg"},
        }

        self.processes = {}       # key -> Popen 对象
        self.photo_refs = {}      # 防止 PhotoImage 被 GC
        self.status_circles = {}  # key -> Canvas

        # 网格行列
        self.GRID_COLS = 4
        self.GRID_ROWS = math.ceil(len(self.scripts) / self.GRID_COLS)
        for c in range(self.GRID_COLS):
            root.grid_columnconfigure(c, weight=1)
        for r in range(self.GRID_ROWS):
            root.grid_rowconfigure(r, weight=1)

        # 构建按钮面板
        for idx, key in enumerate(self.scripts):
            row, col = divmod(idx, self.GRID_COLS)
            self._build_frame(key, row, col)

        # 定时刷新指示灯
        self._check_processes()

        # ESC 退出
        root.bind("<Escape>", lambda e: self._quit_all())

    def _start_process(self, cmd):
        """启动独立进程组，便于一并终止。"""
        if os.name == 'nt':
            return subprocess.Popen(
                cmd, shell=True,
                creationflags=subprocess.CREATE_NEW_PROCESS_GROUP
            )
        else:
            return subprocess.Popen(
                cmd, shell=True,
                preexec_fn=os.setsid
            )

    def _kill_proc(self, proc):
        """向进程组发送 SIGTERM/CTRL_BREAK_EVENT 并 wait()。"""
        if os.name == 'nt':
            proc.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait()

    def _build_frame(self, key, row, col):
        cfg = self.scripts[key]
        frame = tk.Frame(self.root, padx=8, pady=8)
        frame.grid(row=row, column=col, sticky="nsew")

        # frame 内部分区：图片/指示灯/Stop/Label
        frame.grid_rowconfigure(0, weight=3)
        frame.grid_rowconfigure(1, weight=0)
        frame.grid_rowconfigure(2, weight=0)
        frame.grid_rowconfigure(3, weight=1)
        frame.grid_columnconfigure(0, weight=1)

        # 载入并缩放图像
        img = Image.open(cfg['image'])
        maxw = self.root.winfo_screenwidth() // self.GRID_COLS
        maxh = self.root.winfo_screenheight() // self.GRID_ROWS
        img.thumbnail((maxw, maxh))
        tk_img = ImageTk.PhotoImage(img)
        self.photo_refs[key] = tk_img

        # 点击图标切换 启动/停止，同时确保先停掉其他脚本
        btn = tk.Button(
            frame, image=tk_img, bd=0,
            command=lambda k=key: self.toggle_script(k)
        )
        btn.grid(row=0, column=0, sticky="nsew")

        # 状态指示灯
        canvas = tk.Canvas(frame, width=20, height=20, highlightthickness=0)
        canvas.grid(row=1, column=0, pady=4)
        self.status_circles[key] = canvas
        self._update_status(key)

        # Stop 按钮（同 toggle_script 停止当前）
        stop_btn = tk.Button(
            frame, text="Stop",
            command=lambda k=key: self.stop_script(k)
        )
        stop_btn.grid(row=2, column=0, pady=4)

        # 脚本名称
        lbl = tk.Label(frame, text=cfg['name'], font=("Arial", 14))
        lbl.grid(row=3, column=0, pady=4)

    def _update_status(self, key):
        c = self.status_circles[key]
        c.delete("all")
        running = key in self.processes and self.processes[key].poll() is None
        color = "green" if running else "gray"
        c.create_oval(2, 2, 18, 18, fill=color)

    def _check_processes(self):
        for key in self.scripts:
            self._update_status(key)
        self.root.after(500, self._check_processes)

    def toggle_script(self, key):
        proc = self.processes.get(key)
        if proc and proc.poll() is None:
            # 当前脚本在跑，先停止它
            self._kill_proc(proc)
            del self.processes[key]
        else:
            # 启动前，停止所有其他正在跑的脚本
            for k, p in list(self.processes.items()):
                if p.poll() is None:
                    self._kill_proc(p)
                    del self.processes[k]
            # 启动选中脚本
            newp = self._start_process(self.scripts[key]['cmd'])
            self.processes[key] = newp
        self._update_status(key)

    def stop_script(self, key):
        proc = self.processes.get(key)
        if proc and proc.poll() is None:
            self._kill_proc(proc)
            del self.processes[key]
        self._update_status(key)

    def _quit_all(self):
        # 退出前，停止所有子进程
        for p in list(self.processes.values()):
            if p.poll() is None:
                self._kill_proc(p)
        self.root.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    app = LauncherApp(root)
    root.mainloop()