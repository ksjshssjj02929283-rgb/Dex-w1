import os
import signal
import subprocess
import math
import tkinter as tk
from PIL import Image, ImageTk

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = os.path.join(SCRIPT_DIR, "records", "WAIC_DEMOS", "img")

class LauncherApp:
    def __init__(self, root):
        self.root = root
        root.title("Robot Launcher")
        sw = root.winfo_screenwidth()
        sh = root.winfo_screenheight()
        root.geometry(f"{sw}x{sh}+0+0")
        root.resizable(True, True)
        root.attributes('-topmost', True)
        root.after(100, lambda: root.attributes('-topmost', False))

        self.scripts = {
            "bull":   {"name": "Bull",   "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/bulls/",   "image": os.path.join(IMG_DIR, "bull.jpg")},
            "gun":    {"name": "Gun",    "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/guns/",    "image": os.path.join(IMG_DIR, "gun.jpg")},
            "heart":  {"name": "Heart",  "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/hearts/",  "image": os.path.join(IMG_DIR, "heart.jpg")},
            "like":   {"name": "Like",   "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/likes/",   "image": os.path.join(IMG_DIR, "like.jpg")},
            "point":  {"name": "Point",  "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/points/",  "image": os.path.join(IMG_DIR, "point.jpg")},
            "salute": {"name": "Salute", "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/salute/", "image": os.path.join(IMG_DIR, "salute.jpg")},
            "six":    {"name": "Six",    "cmd": "python shuffle_replay_in_folder.py records/ACTIONS/sixs/",   "image": os.path.join(IMG_DIR, "six.jpg")},
            "all":    {"name": "All",    "cmd": "python shuffle_replay_in_folder.py records/WAIC_DEMOS/", "image": os.path.join(IMG_DIR, "robot.jpg")},
        }

        self.processes = {}
        self.photo_refs = {}
        self.status_circles = {}
        self.buttons = {}
        self.frames = {}
        self.raw_images = {}

        self.GRID_COLS = 4
        self.GRID_ROWS = math.ceil(len(self.scripts) / self.GRID_COLS)
        for c in range(self.GRID_COLS):
            root.grid_columnconfigure(c, weight=1)
        for r in range(self.GRID_ROWS):
            root.grid_rowconfigure(r, weight=1)

        for idx, key in enumerate(self.scripts):
            row, col = divmod(idx, self.GRID_COLS)
            self._build_frame(key, row, col)

        self._check_processes()
        root.bind("<Escape>", lambda e: self._quit_all())
        self._resize_after_id = None
        root.bind("<Configure>", self._on_resize)

    def _start_process(self, cmd):
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
        if os.name == 'nt':
            proc.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait()

    def _build_frame(self, key, row, col):
        cfg = self.scripts[key]
        frame = tk.Frame(self.root, padx=8, pady=8)
        frame.grid(row=row, column=col, sticky="nsew")
        self.frames[key] = frame

        frame.grid_rowconfigure(0, weight=3)
        frame.grid_rowconfigure(1, weight=0)
        frame.grid_rowconfigure(2, weight=0)
        frame.grid_rowconfigure(3, weight=1)
        frame.grid_columnconfigure(0, weight=1)

        self.raw_images[key] = Image.open(cfg['image'])

        btn = tk.Button(
            frame, bd=0,
            command=lambda k=key: self.toggle_script(k)
        )
        btn.grid(row=0, column=0, sticky="nsew")
        self.buttons[key] = btn

        canvas = tk.Canvas(frame, width=20, height=20, highlightthickness=0)
        canvas.grid(row=1, column=0, pady=4)
        self.status_circles[key] = canvas
        self._update_status(key)

        stop_btn = tk.Button(
            frame, text="Stop",
            command=lambda k=key: self.stop_script(k)
        )
        stop_btn.grid(row=2, column=0, pady=4)

        lbl = tk.Label(frame, text=cfg['name'], font=("Arial", 14))
        lbl.grid(row=3, column=0, pady=4)

    def _on_resize(self, event):
        if event.widget != self.root:
            return
        if self._resize_after_id is not None:
            self.root.after_cancel(self._resize_after_id)
        self._resize_after_id = self.root.after(100, self._do_resize)

    def _do_resize(self):
        self._resize_after_id = None
        w = self.root.winfo_width()
        h = self.root.winfo_height()
        cell_w = max(w // self.GRID_COLS - 16, 40)
        cell_h = max(h // self.GRID_ROWS - 80, 40)

        for key in self.scripts:
            img = self.raw_images[key].copy()
            img.thumbnail((cell_w, cell_h))
            tk_img = ImageTk.PhotoImage(img)
            self.photo_refs[key] = tk_img
            self.buttons[key].configure(image=tk_img)

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
            self._kill_proc(proc)
            del self.processes[key]
        else:
            for k, p in list(self.processes.items()):
                if p.poll() is None:
                    self._kill_proc(p)
                    del self.processes[k]
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
        for p in list(self.processes.values()):
            if p.poll() is None:
                self._kill_proc(p)
        self.root.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    app = LauncherApp(root)
    root.mainloop()
