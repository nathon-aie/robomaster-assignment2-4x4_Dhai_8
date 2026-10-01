"""Navigation window for planning, playback, and live saved-map missions."""

import argparse
import copy
import hashlib
import json
import math
import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from PIL import Image, ImageTk

from .settings import get as setting, project_path
from .navigation_simulation import load_saved_plan, simulation_frames
from .target_navigation import (COLORS, SHAPES, execute, load_explore_map,
                                   make_plan, sign_key)


COLOR_HEX = {"Red": "#df424b", "Green": "#25a66a",
             "Blue": "#397bd8", "Yellow": "#dca924"}
BG = "#111113"
PANEL = "#19191c"
MAP_BG = "#0a0a0c"
BORDER = "#2b2b30"
TEXT = "#e8e8eb"
MUTED = "#89898f"
GOLD = "#d4a034"
SHAPE_NAMES = {"Circle": "กลม", "Square": "จัตุรัส",
               "Vertical_Rect": "ผืนผ้าแนวตั้ง", "Horizontal_Rect": "ผืนผ้าแนวนอน"}
COLOR_NAMES = {"Red": "แดง", "Green": "เขียว", "Blue": "น้ำเงิน", "Yellow": "เหลือง"}


class TargetNavigationGUI:
    def __init__(self, map_path=None, plan_path=None):
        self.root = tk.Tk()
        self.root.title("RoboMaster EP — Navigate to Signs")
        self.root.geometry("1500x980")
        self.root.minsize(1080, 780)
        self.root.configure(bg=BG)
        self.messages = queue.Queue()
        self.camera_lock = threading.Lock()
        self.latest_camera_frame = None
        self.camera_photo = None
        self.map_path = None
        self.data = None
        self.plan = None
        self.worker = None
        self.system = None
        self.cancel = None
        self.progress = 0
        self.live_pose = None
        self.origin = None
        self.fired = set()
        self.results = {}
        self.simulated = set()
        self.simulation_running = False
        self.simulation_after = None
        self.simulation_frames = None
        self.simulation_gimbal = None
        self.signs = []
        self.selected_signs = set()
        self.sign_points = []
        self.wall_points = []
        self.status = tk.StringVar(value="เลือก explored_map.json หรือ *_map.json")
        self.position = tk.StringVar(value="ยังไม่มีข้อมูลตำแหน่งสด")
        self.selection_summary = tk.StringVar(value="เลือกแมพเพื่อดูป้ายที่พบ")
        self.camera_status = tk.StringVar(value="รอภาพจากกล้อง · เริ่มเดินและยิงเพื่อดูภาพสด")
        self.ready = tk.BooleanVar(value=False)
        self.fire_type = tk.StringVar(value="water_fire")
        self.add_mode = tk.BooleanVar(value=False)
        self.new_color = tk.StringVar(value=COLOR_NAMES["Red"])
        self.new_shape = tk.StringVar(value=SHAPE_NAMES["Circle"])
        self.classes = {(color, shape): tk.BooleanVar(value=True)
                        for color in COLORS for shape in SHAPES}
        self._configure_style()
        self._build()
        self.root.after(60, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        if plan_path:
            self._load_plan(plan_path)
        elif map_path:
            self._load(map_path)

    def _configure_style(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("App.TFrame", background=BG)
        style.configure("Card.TFrame", background=PANEL)
        style.configure("Header.TFrame", background=BG)
        style.configure("Header.TLabel", background=BG, foreground=TEXT,
                        font=("Segoe UI", 17, "bold"))
        style.configure("HeaderSmall.TLabel", background=BG, foreground=MUTED,
                        font=("Segoe UI", 9))
        style.configure("Title.TLabel", background=PANEL, foreground=TEXT,
                        font=("Segoe UI", 10, "bold"))
        style.configure("Hint.TLabel", background=PANEL, foreground=MUTED,
                        font=("Segoe UI", 9))
        style.configure("Body.TLabel", background=BG, foreground=MUTED,
                        font=("Segoe UI", 9))
        style.configure("Card.TCheckbutton", background=PANEL, foreground=TEXT,
                        font=("Segoe UI", 9), padding=2)
        style.map("Card.TCheckbutton", background=[("active", PANEL)])
        style.configure("Footer.TCheckbutton", background=BG, foreground=MUTED,
                        font=("Segoe UI", 9), padding=2)
        style.map("Footer.TCheckbutton", background=[("active", BG)])
        style.configure("Card.TRadiobutton", background=PANEL, foreground=TEXT,
                        font=("Segoe UI", 9))
        style.map("Card.TRadiobutton", background=[("selected", "#483719"),
                                                   ("active", "#30291d")],
                  foreground=[("selected", "#f1c25e"), ("disabled", MUTED)])
        style.configure("Dark.TCombobox", fieldbackground="#252529", background="#252529",
                        foreground=TEXT, arrowcolor=GOLD, bordercolor=BORDER)
        style.map("Dark.TCombobox", fieldbackground=[("readonly", "#252529")],
                  foreground=[("readonly", TEXT)])
        style.configure("Action.TButton", font=("Segoe UI", 9, "bold"), padding=(11, 7),
                        background=GOLD, foreground="#15120c", borderwidth=1)
        style.map("Action.TButton", background=[("active", "#e8b64e"),
                                                 ("disabled", "#29251e")],
                  foreground=[("disabled", "#77716a")])
        style.configure("Soft.TButton", font=("Segoe UI", 9), padding=(11, 7),
                        background=PANEL, foreground=TEXT, borderwidth=1)
        style.map("Soft.TButton", background=[("active", "#29292d"),
                                               ("disabled", PANEL)],
                  foreground=[("disabled", "#5d5d62")])
        style.configure("Stop.TButton", font=("Segoe UI", 9), padding=(11, 7),
                        background=PANEL, foreground="#e17972", borderwidth=1)
        style.map("Stop.TButton", background=[("active", "#332321"),
                                               ("disabled", PANEL)],
                  foreground=[("disabled", "#5d5d62")])

    def _build(self):
        outer = ttk.Frame(self.root, padding=(16, 10), style="App.TFrame")
        outer.pack(fill="both", expand=True)
        header = ttk.Frame(outer, style="Header.TFrame")
        header.pack(fill="x")
        heading = ttk.Frame(header, style="Header.TFrame")
        heading.pack(side="left")
        ttk.Label(heading, text="RoboMaster EP", style="Header.TLabel").pack(anchor="w")
        ttk.Label(heading, text="นำทางไปยังป้ายจากแผนที่ Explore",
                  style="HeaderSmall.TLabel").pack(anchor="w")
        bar = ttk.Frame(header, style="Header.TFrame")
        bar.pack(side="right")
        self.choose_button = ttk.Button(bar, text="เลือกแผนที่ Explore", command=self._choose,
                                        style="Soft.TButton")
        self.choose_button.pack(side="left", padx=(0, 6))
        self.plan_button = ttk.Button(bar, text="วางเส้นทาง", command=self._plan,
                                      style="Soft.TButton")
        self.plan_button.pack(side="left", padx=(0, 6))
        self.open_plan_button = ttk.Button(bar, text="เปิดแผน", command=self._choose_plan,
                                           style="Soft.TButton")
        self.open_plan_button.pack(side="left", padx=(0, 6))
        self.simulate_button = ttk.Button(bar, text="จำลอง", command=self._simulate,
                                          style="Soft.TButton")
        self.simulate_button.pack(side="left", padx=(0, 6))
        self.start_button = ttk.Button(bar, text="เริ่มเดินและยิง", command=self._start,
                                       style="Action.TButton")
        self.start_button.pack(side="left", padx=(0, 6))
        self.stop_button = ttk.Button(bar, text="หยุด", command=self._stop,
                                      style="Stop.TButton")
        self.stop_button.pack(side="left")

        body = ttk.Frame(outer, style="App.TFrame")
        body.pack(fill="both", expand=True, pady=(10, 0))
        map_column = tk.Frame(body, bg=BG)
        map_column.pack(side="left", fill="both", expand=True, padx=(0, 14))
        map_card = tk.Frame(map_column, bg=MAP_BG, highlightthickness=1,
                            highlightbackground=BORDER)
        map_card.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(map_card, bg=MAP_BG, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=2, pady=2)
        self.canvas.bind("<Configure>", lambda _: self._draw())
        self.canvas.bind("<Button-1>", self._map_click)
        self.canvas.bind("<Button-3>", self._map_right_click)
        self.canvas.bind("<Motion>", self._map_hover)

        panel = ttk.Frame(body, width=385, style="App.TFrame")
        panel.pack(side="right", fill="y")
        panel.pack_propagate(False)
        fire_card = tk.Frame(panel, bg=PANEL, highlightthickness=1,
                             highlightbackground=BORDER, padx=10, pady=8)
        fire_card.pack(fill="x", pady=(0, 10))
        tk.Label(fire_card, text="โหมดการยิง", bg=PANEL, fg=TEXT,
                 font=("Segoe UI", 10, "bold")).pack(anchor="w")
        fire_row = tk.Frame(fire_card, bg=PANEL)
        fire_row.pack(anchor="w", pady=(5, 0))
        self.fire_buttons = []
        for label, value in (("กระสุนน้ำ", "water_fire"), ("อินฟราเรด", "infared_fire")):
            button = ttk.Radiobutton(fire_row, text=label, value=value,
                                     variable=self.fire_type, style="Card.TRadiobutton")
            button.pack(side="left", padx=(0, 18))
            self.fire_buttons.append(button)

        class_card = tk.Frame(panel, bg=PANEL, highlightthickness=1,
                              highlightbackground=BORDER, padx=10, pady=8)
        class_card.pack(fill="x", pady=(0, 10))
        tk.Label(class_card, text="เลือกป้ายที่จะยิง · สี × รูปทรง", bg=PANEL,
                 fg=TEXT, font=("Segoe UI", 10, "bold")).pack(anchor="w")
        tk.Label(class_card, text="ช่องสี×รูปทรง: เลือกทั้งชนิด · ป้ายบนแมพ: เลือกทีละป้าย",
                 bg=PANEL, fg=MUTED, font=("Segoe UI", 8)).pack(anchor="w", pady=(2, 0))
        self.class_grid = tk.Canvas(class_card, height=120, bg=PANEL,
                                    highlightthickness=0, cursor="hand2")
        self.class_grid.pack(fill="x", pady=(5, 0))
        self.class_grid.bind("<Configure>", lambda _: self._draw_class_grid())
        self.class_grid.bind("<Button-1>", self._class_grid_click)
        add_row = tk.Frame(class_card, bg=PANEL)
        add_row.pack(fill="x", pady=(6, 0))
        self.add_check = ttk.Checkbutton(add_row, text="เพิ่มป้ายบนแมพ", variable=self.add_mode,
                                         command=self._add_mode_changed, style="Card.TCheckbutton")
        self.add_check.pack(side="left")
        self.shape_picker = ttk.Combobox(add_row, textvariable=self.new_shape,
                                         values=list(SHAPE_NAMES.values()), state="readonly",
                                         width=15, style="Dark.TCombobox")
        self.shape_picker.pack(side="right", padx=(5, 0))
        self.color_picker = ttk.Combobox(add_row, textvariable=self.new_color,
                                         values=list(COLOR_NAMES.values()), state="readonly",
                                         width=9, style="Dark.TCombobox")
        self.color_picker.pack(side="right")
        tk.Label(class_card, text="คลิกขอบกำแพงเพื่อวาง · คลิกขวาที่ป้ายเพิ่มเองเพื่อลบ",
                 bg=PANEL, fg=MUTED, font=("Segoe UI", 8)).pack(anchor="w", pady=(3, 0))
        class_footer = tk.Frame(class_card, bg=PANEL)
        class_footer.pack(fill="x", pady=(4, 0))
        tk.Label(class_footer, textvariable=self.selection_summary, bg=PANEL,
                 fg=MUTED, font=("Segoe UI", 8)).pack(side="left")
        self.all_button = ttk.Button(class_footer, text="เลือกทั้งหมด",
                                     command=lambda: self._select_all(True), style="Soft.TButton")
        self.all_button.pack(side="right")
        self.none_button = ttk.Button(class_footer, text="ล้างที่เลือก",
                                      command=lambda: self._select_all(False), style="Soft.TButton")
        self.none_button.pack(side="right", padx=(0, 5))

        camera_card = tk.Frame(panel, bg=PANEL, highlightthickness=1,
                               highlightbackground=BORDER, padx=10, pady=8)
        camera_card.pack(fill="both", expand=True)
        camera_heading = tk.Frame(camera_card, bg=PANEL)
        camera_heading.pack(fill="x", pady=(0, 8))
        tk.Label(camera_heading, text="ภาพจากกล้อง RoboMaster EP", bg=PANEL, fg=TEXT,
                 font=("Segoe UI", 10, "bold")).pack(side="left")
        tk.Label(camera_heading, text="LIVE", bg="#382b17", fg=GOLD,
                 font=("Segoe UI", 8, "bold"), padx=7, pady=2).pack(side="right")
        self.camera_view = tk.Label(camera_card, text="รอภาพจากกล้อง", bg=MAP_BG,
                                    fg=MUTED, font=("Segoe UI", 10),
                                    borderwidth=0, anchor="center")
        self.camera_view.pack(fill="both", expand=True)
        tk.Label(camera_card, textvariable=self.camera_status, bg=PANEL, fg=MUTED,
                 font=("Segoe UI", 8), anchor="w").pack(fill="x", pady=(7, 0))

        details = tk.Frame(map_column, bg=BG, height=145)
        details.pack(side="bottom", fill="x", pady=(10, 0))
        details.pack_propagate(False)
        sign_card = tk.Frame(details, bg=PANEL, highlightthickness=1,
                             highlightbackground=BORDER, padx=10, pady=8)
        sign_card.pack(side="left", fill="both", expand=True, padx=(0, 10))
        tk.Label(sign_card, text="ป้ายในแผนที่ / ผลการยิง", bg=PANEL, fg=TEXT,
                 font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(0, 5))
        self.sign_list = tk.Listbox(sign_card, font=("Segoe UI", 9),
                                    bg=PANEL, fg=TEXT, relief="flat",
                                    selectbackground="#493615", selectforeground=TEXT,
                                    highlightthickness=0, borderwidth=0)
        self.sign_list.pack(fill="both", expand=True)
        self.sign_list.bind("<Double-Button-1>", self._list_double_click)

        route_card = tk.Frame(details, bg=PANEL, width=385, highlightthickness=1,
                              highlightbackground=BORDER, padx=10, pady=8)
        route_card.pack(side="right", fill="y")
        route_card.pack_propagate(False)
        tk.Label(route_card, text="เส้นทางที่วางไว้", bg=PANEL, fg=TEXT,
                 font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(0, 5))
        self.route_list = tk.Listbox(route_card, font=("Consolas", 9), bg=PANEL,
                                     fg=MUTED, relief="flat", selectbackground="#493615",
                                     highlightthickness=0, borderwidth=0)
        self.route_list.pack(fill="both", expand=True)
        footer = ttk.Frame(outer, style="App.TFrame")
        footer.pack(fill="x", pady=(9, 0))
        ttk.Checkbutton(footer,
                        text="วางหุ่นที่กลางช่องเริ่มต้น และหันหน้าเหมือนตอนเริ่ม Explore แล้ว",
                        variable=self.ready, command=self._controls,
                        style="Footer.TCheckbutton").pack(anchor="w")
        ttk.Label(footer, textvariable=self.status, style="Body.TLabel",
                  wraplength=1260).pack(anchor="w", pady=(2, 0))
        ttk.Label(footer, textvariable=self.position, style="Body.TLabel").pack(anchor="w")
        ttk.Label(footer, text="เทา: trajectory รอบแรก    ฟ้า: เส้นทางที่วางไว้    "
                  "เขียว: เดินแล้ว    ส้ม: ตำแหน่งหุ่น    ดำ: กำแพง",
                  style="Body.TLabel").pack(anchor="w", pady=(2, 0))
        self.class_buttons = []
        self._controls()
        self._draw_class_grid()

    def _selected(self):
        return {(sign["color"], sign["shape"]) for sign in self.signs
                if sign_key(sign) in self.selected_signs}

    def _draw_class_grid(self):
        canvas = self.class_grid
        canvas.delete("all")
        width = max(canvas.winfo_width(), 330)
        left, top, row_height = 38, 18, 24
        cell_width = (width - left - 3) / 4
        labels = ("กลม", "จัตุรัส", "ตั้ง", "นอน")
        for column, label in enumerate(labels):
            x = left + (column + .5) * cell_width
            canvas.create_text(x, 8, text=label, fill=MUTED,
                               font=("Segoe UI", 8))
        for row, color in enumerate(COLORS):
            cy = top + row * row_height + 11
            color_hex = COLOR_HEX[color]
            matching = [sign for sign in self.signs if sign["color"] == color]
            selected_count = sum(sign_key(sign) in self.selected_signs for sign in matching)
            canvas.create_oval(9, cy - 6, 21, cy + 6, outline="",
                               fill=color_hex)
            if matching and selected_count == len(matching):
                canvas.create_oval(12, cy - 3, 18, cy + 3, fill=PANEL, outline="")
            elif selected_count:
                canvas.create_oval(12, cy - 3, 18, cy + 3, fill=GOLD, outline="")
            for column, shape in enumerate(SHAPES):
                class_signs = [sign for sign in matching if sign["shape"] == shape]
                count = sum(sign_key(sign) in self.selected_signs for sign in class_signs)
                selected = bool(class_signs) and count == len(class_signs)
                partial = 0 < count < len(class_signs)
                x1 = left + column * cell_width + 2
                x2 = left + (column + 1) * cell_width - 3
                y1 = top + row * row_height
                y2 = y1 + 20
                canvas.create_rectangle(x1, y1, x2, y2, fill="#211b11" if selected else
                                        "#292318" if partial else PANEL,
                                        outline=GOLD if selected else "#8d6a2c" if partial else BORDER,
                                        width=2 if selected else 1)
                cx, iy = (x1 + x2) / 2, (y1 + y2) / 2
                fill = color_hex if selected or partial else "#55565b"
                if shape == "Circle":
                    canvas.create_oval(cx - 6, iy - 6, cx + 6, iy + 6,
                                       fill=fill, outline="")
                else:
                    half_w, half_h = {"Square": (6, 6), "Vertical_Rect": (4, 8),
                                      "Horizontal_Rect": (8, 4)}[shape]
                    canvas.create_rectangle(cx - half_w, iy - half_h, cx + half_w,
                                            iy + half_h, fill=fill, outline="")
                if partial:
                    canvas.create_text(x2 - 4, y1 + 10, text="{}/{}".format(count, len(class_signs)),
                                       anchor="e", fill="#e6bd64", font=("Segoe UI", 7, "bold"))

    def _class_grid_click(self, event):
        if self._running() or self.data is None:
            return
        width = max(self.class_grid.winfo_width(), 330)
        left, top, row_height = 38, 18, 24
        row = int((event.y - top) // row_height)
        if row not in range(4) or not top <= event.y < top + 4 * row_height:
            return
        if event.x < left:
            color = COLORS[row]
            matching = {sign_key(sign) for sign in self.signs if sign["color"] == color}
            if matching and matching <= self.selected_signs:
                self.selected_signs.difference_update(matching)
            else:
                self.selected_signs.update(matching)
            self._selection_changed()
            return
        cell_width = (width - left - 3) / 4
        column = int((event.x - left) // cell_width)
        if column in range(4):
            self._toggle_class((COLORS[row], SHAPES[column]))

    def _sync_class_checks(self):
        for color in COLORS:
            for shape in SHAPES:
                matching = {sign_key(sign) for sign in self.signs
                            if (sign["color"], sign["shape"]) == (color, shape)}
                self.classes[color, shape].set(bool(matching) and matching <= self.selected_signs)
        self.selection_summary.set("เลือก {}/{} ป้ายในแมพ".format(
            len(self.selected_signs), len(self.signs)))
        self._draw_class_grid()

    def _toggle_class(self, key):
        if self._running():
            return
        matching = {sign_key(sign) for sign in self.signs
                    if (sign["color"], sign["shape"]) == key}
        if matching and matching <= self.selected_signs:
            self.selected_signs.difference_update(matching)
        else:
            self.selected_signs.update(matching)
        self._selection_changed()

    def _select_all(self, value):
        if self._running() or self.data is None:
            return
        self.selected_signs = ({sign_key(sign) for sign in self.signs} if value else set())
        self._selection_changed()

    def _toggle_sign(self, index):
        if self._running() or index < 0 or index >= len(self.signs):
            return
        key = sign_key(self.signs[index])
        if key in self.selected_signs:
            self.selected_signs.remove(key)
        else:
            self.selected_signs.add(key)
        self._selection_changed()

    def _selection_changed(self):
        self._sync_class_checks()
        self._refresh_sign_list()
        self._invalidate()
        self.status.set("เลือก {}/{} ป้ายแล้ว · กดวางเส้นทางเพื่ออัปเดตแผน".format(
            len(self.selected_signs), len(self.signs)))

    def _refresh_sign_list(self):
        self.sign_list.delete(0, "end")
        for index, sign in enumerate(self.signs):
            key = sign_key(sign)
            selected = key in self.selected_signs
            result = self.results.get(key)
            if result == "simulated":
                marker = "◇"
            elif result == "fired":
                marker = "✓"
            elif result == "not_fired":
                marker = "!"
            else:
                marker = "●" if selected else "○"
            label = "{} {} {} · ({},{}) {}{}".format(
                marker, COLOR_NAMES[sign["color"]], SHAPE_NAMES[sign["shape"]],
                *sign["cell"], "NESW"[key[1]],
                " [เพิ่มเอง]" if sign.get("source") == "manual" else "")
            self.sign_list.insert("end", label)
            self.sign_list.itemconfig(index, fg=(GOLD if result == "simulated" else
                                                 "#13865d" if result == "fired" else
                                                 COLOR_HEX[sign["color"]] if selected else MUTED))

    def _list_double_click(self, event):
        if self.sign_list.size():
            self._toggle_sign(self.sign_list.nearest(event.y))

    def _hit_sign(self, x, y):
        candidates = [(math.hypot(x - sx, y - sy), index)
                      for index, sx, sy, radius in self.sign_points
                      if math.hypot(x - sx, y - sy) <= radius]
        return min(candidates)[1] if candidates else None

    def _hit_wall(self, x, y):
        candidates = []
        for cell, direction, x1, y1, x2, y2, cx, cy, radius in self.wall_points:
            if x1 == x2:
                distance = abs(x - x1) if min(y1, y2) <= y <= max(y1, y2) else float("inf")
            else:
                distance = abs(y - y1) if min(x1, x2) <= x <= max(x1, x2) else float("inf")
            if distance <= radius:
                candidates.append((distance, math.hypot(x - cx, y - cy),
                                   cell, direction, x1, y1, x2, y2))
        if not candidates:
            return None
        _, _, cell, direction, x1, y1, x2, y2 = min(candidates)
        return cell, direction, (x1, y1, x2, y2)

    def _add_mode_changed(self):
        self.canvas.delete("wall_hover")
        self.status.set("คลิกขอบกำแพงของ Grid ที่สำรวจแล้วเพื่อวางป้าย" if self.add_mode.get()
                        else "คลิกป้ายบนแมพเพื่อเลือกหรือยกเลิกทีละป้าย")

    def _save_manual_signs(self, signs):
        folder = Path(project_path("paths.telemetry")) / "manual_maps"
        if self.map_path.parent.resolve() == folder.resolve() and self.map_path.name.endswith("_manual_map.json"):
            destination = self.map_path
        else:
            digest = hashlib.sha1(str(self.map_path.resolve()).encode("utf-8")).hexdigest()[:8]
            destination = folder / "{}_{}_manual_map.json".format(self.map_path.stem, digest)
        staged = destination.with_name(destination.name + ".tmp")
        try:
            folder.mkdir(parents=True, exist_ok=True)
            updated = copy.deepcopy(self.data)
            updated["signs"] = signs
            staged.write_text(json.dumps(updated, ensure_ascii=False, indent=2) + "\n",
                              encoding="utf-8")
            data, _, loaded_signs = load_explore_map(staged)
            staged.replace(destination)
        except Exception as exc:
            staged.unlink(missing_ok=True)
            messagebox.showerror("บันทึกป้ายไม่ได้", str(exc), parent=self.root)
            return False
        self.map_path = destination
        self.data = data
        self.signs = loaded_signs
        return True

    def _add_sign(self, cell, direction):
        color = next(key for key, label in COLOR_NAMES.items() if label == self.new_color.get())
        shape = next(key for key, label in SHAPE_NAMES.items() if label == self.new_shape.get())
        sign = {"cell": list(cell), "direction": direction, "color": color,
                "shape": shape, "source": "manual"}
        key = sign_key(sign)
        if any(sign_key(item) == key for item in self.signs):
            self.status.set("มีป้ายสีและรูปทรงนี้บนกำแพงด้านนี้แล้ว")
            return
        if self._save_manual_signs(list(self.data.get("signs", [])) + [sign]):
            self.selected_signs.add(key)
            self._selection_changed()
            self.status.set("เพิ่มป้ายที่ Grid {} ด้าน {} · บันทึกแมพ: {}".format(
                cell, "NESW"[direction], self.map_path.name))

    def _map_click(self, event):
        if self._running():
            return
        if self.add_mode.get():
            wall = self._hit_wall(event.x, event.y)
            if wall is None:
                self.status.set("เลือกขอบกำแพงของ Grid ที่สำรวจแล้ว")
            else:
                self._add_sign(*wall[:2])
            return
        index = self._hit_sign(event.x, event.y)
        if index is not None:
            self._toggle_sign(index)

    def _map_right_click(self, event):
        if self._running():
            return
        index = self._hit_sign(event.x, event.y)
        if index is None or self.signs[index].get("source") != "manual":
            return
        key = sign_key(self.signs[index])
        signs = [item for item in self.data["signs"] if sign_key(item) != key]
        if self._save_manual_signs(signs):
            self.selected_signs.discard(key)
            self._selection_changed()
            self.status.set("ลบป้ายที่เพิ่มเองแล้ว · บันทึกแมพ: {}".format(self.map_path.name))

    def _map_hover(self, event):
        self.canvas.delete("wall_hover")
        if self.add_mode.get() and not self._running():
            wall = self._hit_wall(event.x, event.y)
            if wall is not None:
                self.canvas.create_line(*wall[2], fill=GOLD, width=5, tags="wall_hover")
            self.canvas.configure(cursor="crosshair" if wall is not None else "")
            return
        index = self._hit_sign(event.x, event.y)
        self.canvas.configure(cursor="hand2" if index is not None and not self._running() else "")

    def _running(self):
        return self.simulation_running or (self.worker is not None and self.worker.is_alive())

    def _controls(self):
        running = self._running()
        self.choose_button.configure(state="disabled" if running else "normal")
        self.open_plan_button.configure(state="disabled" if running else "normal")
        self.plan_button.configure(state="normal" if self.data and not running else "disabled")
        self.simulate_button.configure(state="normal" if self.plan and not running else "disabled")
        self.start_button.configure(state="normal" if self.plan and self.ready.get() and not running else "disabled")
        self.stop_button.configure(state="normal" if running else "disabled")
        for button in self.class_buttons + [self.all_button, self.none_button, self.add_check]:
            button.configure(state="disabled" if running or self.data is None else "normal")
        for picker in (self.color_picker, self.shape_picker):
            picker.configure(state="disabled" if running or self.data is None else "readonly")
        if running or self.data is None:
            self.add_mode.set(False)
        for button in self.fire_buttons:
            button.configure(state="disabled" if running else "normal")
        self.sign_list.configure(state="disabled" if running else "normal")

    def _invalidate(self):
        self.plan = None
        self.ready.set(False)
        self.progress = 0
        self.simulated.clear()
        self.simulation_gimbal = None
        self.route_list.delete(0, "end")
        self._controls()
        self._draw()

    def _choose(self):
        root = Path(project_path("paths.telemetry"))
        path = filedialog.askopenfilename(parent=self.root, title="เลือกแผนที่ Explore",
                                          initialdir=root, filetypes=[("JSON", "*.json")])
        if path:
            self._load(path)

    def _choose_plan(self):
        path = filedialog.askopenfilename(
            parent=self.root, title="เลือก navigation_plan.json",
            initialdir=Path(project_path("paths.telemetry")),
            filetypes=[("Navigation plan", "navigation_plan.json"), ("JSON", "*.json")])
        if path:
            self._load_plan(path)

    def _load_plan(self, path):
        try:
            try:
                plan, source = load_saved_plan(path)
            except FileNotFoundError:
                replacement = filedialog.askopenfilename(
                    parent=self.root, title="เลือกแมพ Explore ที่ใช้สร้างแผน",
                    initialdir=Path(project_path("paths.telemetry")),
                    filetypes=[("Explore map", "*.json")])
                if not replacement:
                    return
                plan, source = load_saved_plan(path, source_map=replacement)
        except Exception as exc:
            messagebox.showerror("เปิดแผนไม่ได้", str(exc), parent=self.root)
            return
        if not self._load(source):
            return
        self.plan = plan
        self.selected_signs = {sign_key(sign) for sign in plan["targets"]}
        self._sync_class_checks()
        self._refresh_sign_list()
        self._show_plan()
        self.status.set("เปิดแผน {} · {} ก้าว / {} ป้าย · กดจำลองเพื่อดูการเดิน".format(
            Path(path).name, plan["step_count"], len(plan["targets"])))

    def _load(self, path):
        try:
            data, _, signs = load_explore_map(path)
        except Exception as exc:
            messagebox.showerror("เปิดแผนที่ไม่ได้", str(exc), parent=self.root)
            return False
        self.map_path = Path(path)
        self.data = data
        self.signs = signs
        self.selected_signs = {sign_key(sign) for sign in signs}
        self._invalidate()
        self.live_pose = None
        self.origin = None
        self.fired.clear()
        self.simulated.clear()
        self.simulation_gimbal = None
        self.results.clear()
        self._sync_class_checks()
        self._refresh_sign_list()
        self.status.set("แมพ: {} | สำรวจ {} ช่อง | พบ {} ป้าย{}".format(
            self.map_path.name, len(data["visited"]), len(signs),
            " | แผนที่บางส่วน: วางหุ่นที่ Grid {} ก่อนเริ่ม".format(tuple(data["start_cell"]))
            if data["status"] != "completed" else ""))
        self._draw()
        return True

    def _plan(self):
        try:
            self.plan = make_plan(self.map_path, self._selected(), self.selected_signs)
        except Exception as exc:
            messagebox.showerror("วางเส้นทางไม่ได้", str(exc), parent=self.root)
            return
        self._show_plan()
        self.status.set("{} ป้าย / {} ช่องเป้าหมาย | เส้นทาง {} ก้าว{}".format(
            len(self.plan["targets"]), len(self.plan["target_cells"]), self.plan["step_count"],
            " | วางหุ่นที่ Grid {} ก่อนเริ่ม".format(tuple(self.plan["start_cell"]))
            if self.data["status"] != "completed" else ""))

    def _show_plan(self):
        self.progress = 0
        self.live_pose = None
        self.fired.clear()
        self.simulated.clear()
        self.simulation_gimbal = None
        self.results.clear()
        self._refresh_sign_list()
        self.route_list.delete(0, "end")
        for index, (a, b) in enumerate(zip(self.plan["route"], self.plan["route"][1:]), 1):
            self.route_list.insert("end", "{:02d}  {} → {}".format(index, tuple(a), tuple(b)))
        self._controls()
        self._draw()

    def _start(self):
        if not self.plan or not self.ready.get() or self._running():
            return
        self._clear_camera("กำลังเชื่อมต่อกล้อง...")
        self.cancel = threading.Event()
        self.system = None
        self.origin = None
        self.live_pose = None
        self.progress = 0
        self.fired.clear()
        self.simulated.clear()
        self.simulation_gimbal = None
        self.results.clear()
        self._refresh_sign_list()
        self.selected_fire_type = self.fire_type.get()
        self.status.set("กำลังเชื่อมต่อ RoboMaster EP...")
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()
        self._controls()

    def _simulate(self):
        if not self.plan or self._running():
            return
        self._clear_camera("โหมดจำลอง · ไม่มีภาพจากกล้อง")
        self.ready.set(False)
        self.simulation_frames = iter(simulation_frames(
            self.plan, scan_pitch=setting("fire.scan_pitch_deg")))
        self.simulation_running = True
        self.progress = 0
        self.live_pose = None
        self.fired.clear()
        self.simulated.clear()
        self.simulation_gimbal = None
        self.results.clear()
        self._refresh_sign_list()
        self.position.set("จำลอง: ยังไม่เริ่มเดิน · ไม่มีการเชื่อมต่อหุ่น")
        self.status.set("[จำลอง] กำลังเล่นเส้นทาง · ไม่มีการยิงจริง")
        self._controls()
        self._simulation_tick()

    def _simulation_tick(self):
        if not self.simulation_running:
            return
        try:
            delay, kind, *payload = next(self.simulation_frames)
        except StopIteration:
            self.simulation_running = False
            self.simulation_after = None
            self._controls()
            return
        if kind == "position":
            row, column, yaw = payload
            self.live_pose = (row, column, yaw)
            self.position.set("จำลอง: Grid ({:.2f}, {:.2f}) · yaw {:.0f}°".format(
                row, column, yaw))
        elif kind == "moving":
            source, target, step = payload
            self.status.set("[จำลอง] เดิน {}/{}: {} → {}".format(
                step, self.plan["step_count"], tuple(source), tuple(target)))
        elif kind == "arrive":
            cell, step = payload
            self.progress = step
            if step <= self.route_list.size():
                self.route_list.see(step - 1)
            self.status.set("[จำลอง] ถึง Grid {} · {}/{} ก้าว".format(
                tuple(cell), step, self.plan["step_count"]))
        elif kind == "face":
            cell, direction = payload
            self.live_pose = (cell[0], cell[1], direction * 90)
            self.position.set("จำลอง: Grid {} · Chassis หัน {}".format(
                tuple(cell), "NESW"[direction]))
            self.status.set("[จำลอง] Chassis หันเข้ากำแพงด้าน {} ที่ Grid {}".format(
                "NESW"[direction], tuple(cell)))
        elif kind == "scan":
            cell, direction, targets = payload
            self.simulation_gimbal = {"cell": tuple(cell), "direction": direction,
                                      "yaw": 0.0, "pitch": 0.0}
            self.status.set("[จำลอง] Gimbal สแกนด้าน {} ที่ Grid {} · {} ป้ายในแผน".format(
                "NESW"[direction], tuple(cell), len(targets)))
        elif kind in ("gimbal", "scan_hold"):
            cell, direction, yaw, pitch = payload
            self.simulation_gimbal = {"cell": tuple(cell), "direction": direction,
                                      "yaw": yaw, "pitch": pitch}
            self.status.set("[จำลอง] Gimbal ด้าน {} · yaw {:+.0f}° / pitch {:+.0f}°".format(
                "NESW"[direction], yaw, pitch))
        elif kind == "gimbal_end":
            self.simulation_gimbal = None
        elif kind == "targets":
            targets = payload[0]
            for target in targets:
                key = sign_key(target)
                self.simulated.add(key)
                self.results[key] = "simulated"
            self._refresh_sign_list()
            self.status.set("[จำลอง] ผ่านป้าย {}/{} จุด · ไม่มีการยิงจริง".format(
                len(self.simulated), len(self.plan["targets"])))
        elif kind == "complete":
            self.simulation_running = False
            self.simulation_after = None
            self.simulation_gimbal = None
            self.status.set("[จำลอง] จบเส้นทาง {} ก้าว / {} ป้าย · ไม่มีการเดินหรือยิงจริง".format(
                self.plan["step_count"], len(self.simulated)))
            self._controls()
            self._draw()
            return
        self._draw()
        self.simulation_after = self.root.after(delay, self._simulation_tick)

    def _stop_simulation(self):
        if self.simulation_after is not None:
            self.root.after_cancel(self.simulation_after)
        self.simulation_after = None
        self.simulation_frames = None
        self.simulation_running = False
        self.simulation_gimbal = None
        self.status.set("[จำลอง] หยุดแล้วที่ก้าว {}/{} · ไม่มีการยิงจริง".format(
            self.progress, self.plan["step_count"]))
        self._controls()
        self._draw()

    def _run(self):
        def sensor(snapshot):
            self.messages.put(("sensor", snapshot.pos_x, snapshot.pos_y,
                               snapshot.yaw, snapshot.frame_index))

        def ready(system):
            self.system = system
            if self.cancel.is_set():
                system.thread_2_controller.stop_running()

        def frame_ready(frame):
            # The SDK camera runs on another thread. Keep only the newest frame.
            with self.camera_lock:
                self.latest_camera_frame = frame

        try:
            result = execute(self.plan, on_sensor=sensor, on_system_ready=ready,
                             on_progress=lambda *args: self.messages.put(("progress", *args)),
                             on_target=lambda item: self.messages.put(("target", item)),
                             on_phase=lambda phase: self.messages.put(("phase", phase)),
                             on_frame=frame_ready, cancel=self.cancel,
                             fire_type=self.selected_fire_type)
            self.messages.put(("done", result))
        except Exception as exc:
            self.messages.put(("error", str(exc)))

    def _stop(self):
        if self.simulation_running:
            self._stop_simulation()
            return
        if self.cancel is None or self.cancel.is_set():
            return
        self.cancel.set()
        self.status.set("กำลังหยุดหุ่น...")
        if self.system is not None and self.system.thread_2_controller is not None:
            threading.Thread(target=self.system.thread_2_controller.stop_running,
                             daemon=True).start()

    def _poll(self):
        redraw = False
        try:
            while True:
                item = self.messages.get_nowait()
                if item[0] == "sensor" and self.plan:
                    _, x, y, yaw, frame = item
                    if self.origin is None:
                        self.origin = (x, y)
                    size = self.plan["cell_size_m"]
                    start = self.plan["start_cell"]
                    self.live_pose = (start[0] + (x - self.origin[0]) / size,
                                      start[1] + (y - self.origin[1]) / size, yaw)
                    self.position.set("Odometry: x={:.2f} m, y={:.2f} m, yaw={:.1f}° | frame {}".format(
                        x, y, yaw, frame))
                    redraw = True
                elif item[0] == "progress":
                    _, step, current, target = item
                    self.progress = step
                    self.status.set("เดิน {}/{}: {} → {}".format(
                        step, self.plan["step_count"], tuple(current),
                        tuple(target) if target is not None else "ถึงแล้ว"))
                    if step and step <= self.route_list.size():
                        self.route_list.see(step - 1)
                    redraw = True
                elif item[0] == "phase":
                    self.status.set(item[1])
                elif item[0] == "target":
                    record = item[1]
                    key = (tuple(record["cell"]), record["direction"],
                           record["color"], record["shape"])
                    self.results[key] = record["status"]
                    if record["status"] == "fired":
                        self.fired.add(key)
                    self._refresh_sign_list()
                    self.status.set("{} {} @ {}: {}".format(record["color"], record["shape"],
                                                            tuple(record["cell"]), record["status"]))
                    redraw = True
                elif item[0] == "done":
                    result = item[1]
                    self.worker = None
                    self.camera_status.set("การเชื่อมต่อกล้องสิ้นสุดแล้ว")
                    self.progress = result["completed_steps"]
                    if result.get("error"):
                        self.status.set("ผล: {} | {}".format(result["status"], result["error"]))
                        if result["status"] == "failed":
                            messagebox.showerror("Navigation ไม่สำเร็จ", result["error"],
                                                 parent=self.root)
                    else:
                        self.status.set("ผล: {} | ยิง {}/{} ป้าย | {}".format(
                            result["status"], sum(v["status"] == "fired" for v in result["targets"]),
                            len(result["targets"]), result["result_file"]))
                    self._controls()
                    redraw = True
                elif item[0] == "error":
                    self.worker = None
                    self.camera_status.set("ไม่สามารถรับภาพจากกล้อง")
                    self.status.set("ผิดพลาด: " + item[1])
                    self._controls()
        except queue.Empty:
            pass
        self._show_latest_camera_frame()
        if redraw:
            self._draw()
        self.root.after(60, self._poll)

    def _clear_camera(self, status):
        with self.camera_lock:
            self.latest_camera_frame = None
        self.camera_photo = None
        self.camera_view.configure(image="", text="รอภาพจากกล้อง")
        self.camera_status.set(status)

    def _show_latest_camera_frame(self):
        with self.camera_lock:
            frame = self.latest_camera_frame
            self.latest_camera_frame = None
        if frame is None:
            return
        # RoboMaster/OpenCV supplies BGR; Tk image objects must be built on this thread.
        height, width = frame.shape[:2]
        area_width = max(self.camera_view.winfo_width() - 4, 1)
        area_height = max(self.camera_view.winfo_height() - 4, 1)
        if area_width < 20 or area_height < 20:
            return
        scale = min(area_width / width, area_height / height)
        picture = Image.fromarray(frame[:, :, ::-1]).resize(
            (max(1, int(width * scale)), max(1, int(height * scale))),
            Image.Resampling.BILINEAR)
        self.camera_photo = ImageTk.PhotoImage(picture, master=self.root)
        self.camera_view.configure(image=self.camera_photo, text="")
        if self.worker is not None:
            self.camera_status.set("ภาพสดจากกล้อง · กำลังนำทาง")

    def _draw_map_corners(self):
        canvas = self.canvas
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 40 or height < 40:
            return
        inset, length = 9, 18
        color = "#e2ad55"
        for x, dx in ((inset, 1), (width - inset, -1)):
            for y, dy in ((inset, 1), (height - inset, -1)):
                canvas.create_line(x, y + dy * length, x, y,
                                   x + dx * length, y, fill=color, width=2)

    def _draw(self):
        canvas = self.canvas
        canvas.delete("all")
        self.sign_points = []
        self.wall_points = []
        if self.data is None:
            width, height = canvas.winfo_width(), canvas.winfo_height()
            for fraction in (1 / 3, 2 / 3):
                canvas.create_line(width * fraction, 0, width * fraction, height,
                                   fill="#1b1b1f")
                canvas.create_line(0, height * fraction, width, height * fraction,
                                   fill="#1b1b1f")
            canvas.create_text(max(canvas.winfo_width(), 200) / 2,
                               max(canvas.winfo_height(), 200) / 2,
                               text="ยังไม่มีแผนที่\nเลือก explored_map.json หรือ *_map.json",
                               fill="#c3c3c6", font=("Segoe UI", 11), justify="center")
            canvas.create_text(14, max(canvas.winfo_height(), 200) - 18,
                               text="รอข้อมูลจากการสำรวจ", anchor="w",
                               fill="#6d6d73", font=("Segoe UI", 8))
            self._draw_map_corners()
            return
        rows = self.data["map_info"]["rows"]
        columns = self.data["map_info"]["columns"]
        width, height = max(canvas.winfo_width(), 200), max(canvas.winfo_height(), 200)
        size = min((width - 44) / columns, (height - 44) / rows)
        x0, y0 = (width - columns * size) / 2, (height - rows * size) / 2

        def point(cell):
            return (x0 + (cell[1] + .5) * size,
                    y0 + (rows - cell[0] - .5) * size)

        visited = {tuple(cell) for cell in self.data["visited"]}
        walls = {frozenset(tuple(cell) for cell in edge["cells"])
                 for edge in self.data["edges"] if edge["wall"]}
        for row in range(rows):
            for column in range(columns):
                x, y = x0 + column * size, y0 + (rows - row - 1) * size
                canvas.create_rectangle(x, y, x + size, y + size,
                                        fill="#14171a" if (row, column) in visited else "#101113",
                                        outline="#303137")
                if size > 33:
                    canvas.create_text(x + 4, y + 4, text="{},{}".format(row, column),
                                       anchor="nw", fill="#777d84")
                cell = (row, column)
                if cell not in visited:
                    continue
                cx, cy = point(cell)
                sides = ((1, 0, 0, 0), (0, 1, 1, 1),
                         (-1, 0, 0, 1), (0, -1, 1, 0))
                for direction, (dr, dc, vertical, far) in enumerate(sides):
                    other = (row + dr, column + dc)
                    boundary = not (0 <= other[0] < rows and 0 <= other[1] < columns)
                    if not boundary and frozenset((cell, other)) not in walls:
                        continue
                    if vertical:
                        side_x = x + size if far else x
                        segment = (side_x, y, side_x, y + size)
                    else:
                        side_y = y + size if far else y
                        segment = (x, side_y, x + size, side_y)
                    self.wall_points.append((cell, direction, *segment, cx, cy,
                                             max(12, min(20, size * .18))))
        trajectory = [point(item["cell"]) for item in self.data.get("trajectory", [])]
        for a, b in zip(trajectory, trajectory[1:]):
            canvas.create_line(*a, *b, fill="#74787f", width=2, dash=(5, 4))
        if self.plan:
            route = [point(cell) for cell in self.plan["route"]]
            for i, (a, b) in enumerate(zip(route, route[1:])):
                canvas.create_line(*a, *b, fill="#27aa78" if i < self.progress else "#3185cf",
                                   width=4, arrow="last", arrowshape=(8, 10, 4))
        if self.simulation_gimbal:
            gimbal = self.simulation_gimbal
            gx, gy = point(gimbal["cell"])
            chassis_yaw = self.live_pose[2] if self.live_pose else gimbal["direction"] * 90
            beam_length = size * .49
            for offset in (-18, 0, 18):
                guide = math.radians(chassis_yaw + offset)
                canvas.create_line(gx, gy, gx + beam_length * math.sin(guide),
                                   gy - beam_length * math.cos(guide),
                                   fill="#64512c", width=1, dash=(3, 4),
                                   tags="gimbal_guide")
            angle = math.radians(chassis_yaw + gimbal["yaw"])
            canvas.create_line(gx, gy, gx + beam_length * math.sin(angle),
                               gy - beam_length * math.cos(angle),
                               fill=GOLD, width=4, arrow="last", arrowshape=(8, 10, 4),
                               tags="gimbal_beam")
        for edge in self.data["edges"]:
            if not edge["wall"]:
                continue
            a, b = edge["cells"]
            if a[0] != b[0]:
                row, column = max(a[0], b[0]), a[1]
                x, y = x0 + column * size, y0 + (rows - row) * size
                canvas.create_line(x, y, x + size, y, fill="#a8adb3", width=4)
            else:
                row, column = a[0], max(a[1], b[1])
                x, y = x0 + column * size, y0 + (rows - row - 1) * size
                canvas.create_line(x, y, x, y + size, fill="#a8adb3", width=4)
        canvas.create_rectangle(x0, y0, x0 + columns * size, y0 + rows * size,
                                outline="#a8adb3", width=3)
        groups = {}
        for index, sign in enumerate(self.signs):
            key = sign_key(sign)
            groups.setdefault((key[0], key[1]), []).append(index)
        for index, sign in enumerate(self.signs):
            key = sign_key(sign)
            direction = key[1]
            x, y = point(sign["cell"])
            offsets = ((0, -size * .34), (size * .34, 0),
                       (0, size * .34), (-size * .34, 0))
            dx, dy = offsets[direction]
            siblings = groups[key[:2]]
            shift = (siblings.index(index) - (len(siblings) - 1) / 2) * min(19, size * .22)
            if direction in (0, 2):
                dx += shift
            else:
                dy += shift
            x, y = x + dx, y + dy
            selected = key in self.selected_signs
            color = COLOR_HEX[sign["color"]]
            outline = ("#ffffff" if key in self.fired else GOLD if key in self.simulated
                       else color if selected else "#777a80")
            canvas.create_oval(x - 13, y - 13, x + 13, y + 13,
                               fill=PANEL, outline=outline, width=2,
                               dash=() if selected or key in self.fired else (3, 2))
            fill = color if selected else "#36383c"
            shape = sign["shape"]
            if shape == "Circle":
                canvas.create_oval(x - 7, y - 7, x + 7, y + 7,
                                   fill=fill, outline=outline)
            else:
                half_w, half_h = {"Square": (7, 7), "Vertical_Rect": (5, 9),
                                  "Horizontal_Rect": (9, 5)}[shape]
                canvas.create_rectangle(x - half_w, y - half_h, x + half_w, y + half_h,
                                        fill=fill, outline=outline)
            self.sign_points.append((index, x, y, max(16, size * .15)))
        sx, sy = point(self.data["start_cell"])
        canvas.create_rectangle(sx - 7, sy - 7, sx + 7, sy + 7, fill="#198e65", outline="white")
        if self.live_pose:
            x, y = point(self.live_pose)
            canvas.create_oval(x - 10, y - 10, x + 10, y + 10,
                               fill="#ef8b24", outline="#824410", width=2)
            yaw = math.radians(self.live_pose[2])
            canvas.create_line(x, y, x + 16 * math.sin(yaw), y - 16 * math.cos(yaw),
                               fill="#713509", width=3, arrow="last")
            if self.simulation_gimbal:
                turret_yaw = math.radians(self.live_pose[2] + self.simulation_gimbal["yaw"])
                tx, ty = x + 9 * math.sin(turret_yaw), y - 9 * math.cos(turret_yaw)
                canvas.create_oval(tx - 4, ty - 4, tx + 4, ty + 4,
                                   fill=GOLD, outline="#3b2b10", tags="gimbal_head")
        elif self.plan:
            x, y = point(self.plan["route"][self.progress])
            canvas.create_oval(x - 8, y - 8, x + 8, y + 8, fill="#ef8b24", outline="")
        if self.simulation_running or self.simulated:
            canvas.create_text(15, 15, text="SIMULATION · ไม่มีการยิงจริง",
                               anchor="nw", fill=GOLD, font=("Segoe UI", 9, "bold"))
        if self.simulation_gimbal:
            canvas.create_text(15, 34,
                               text="GIMBAL yaw {:+.0f}°  pitch {:+.0f}° · Chassis yaw {:.0f}°".format(
                                   self.simulation_gimbal["yaw"],
                                   self.simulation_gimbal["pitch"], self.live_pose[2]),
                               anchor="nw", fill="#e7c373", font=("Segoe UI", 9, "bold"))
        self._draw_map_corners()

    def _close(self):
        if self._running():
            self._stop()
            self.root.after(200, self._close)
        else:
            self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", help="Explore map JSON (complete or partial)")
    parser.add_argument("--plan", help="Saved navigation_plan.json to simulate")
    args = parser.parse_args()
    TargetNavigationGUI(args.map, args.plan).run()


if __name__ == "__main__":
    main()
