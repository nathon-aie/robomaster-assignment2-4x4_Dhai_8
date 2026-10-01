"""Interactive operation selection and live GUI for RoboMaster EP."""
import json
import queue
import signal
import sys
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, simpledialog, ttk
from PIL import Image, ImageTk

from .settings import get as setting, map_geometry, project_path


TARGET_COLOR_OPTIONS = (('แดง', 'Red'), ('เหลือง', 'Yellow'),
                        ('น้ำเงิน', 'Blue'), ('เขียว', 'Green'))
TARGET_SHAPE_OPTIONS = (('วงกลม', 'Circle'), ('สี่เหลี่ยมจตุรัส', 'Square'),
                        ('สี่เหลี่ยมผืนผ้าแนวตั้ง', 'Vertical_Rect'),
                        ('สี่เหลี่ยมผืนผ้าแนวนอน', 'Horizontal_Rect'),
                        ('รูปทรงทั้งหมดในตัวเลือกนี้', 'All'))
PREVIEW_INTERVAL_SEC = 1 / 30
PREVIEW_MAX_WIDTH = 1280


class RunControl:
    def __init__(self):
        self.cancel = threading.Event()
        self.system = None
        self.stopped_controller = None

    def set_system(self, system):
        self.system = system
        if self.cancel.is_set():
            self._stop_controller()

    def stop(self):
        self.cancel.set()
        self._stop_controller()

    def _stop_controller(self):
        controller = getattr(self.system, 'thread_2_controller', None)
        if controller is not None and controller is not self.stopped_controller:
            self.stopped_controller = controller
            threading.Thread(target=controller.stop_running, daemon=True).start()


class _GuiWriter:
    def __init__(self, messages, log_path):
        self.messages = messages
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = log_path.open('a', encoding='utf-8', buffering=1)
        self.lock = threading.Lock()

    def write(self, value):
        if value:
            self.messages.put(('log', value))
            with self.lock:
                self.stream.write(value)

    def flush(self):
        with self.lock:
            self.stream.flush()

    def close(self):
        with self.lock:
            self.stream.close()


class OperationGUI:
    def __init__(self, runner):
        self.root = tk.Tk()
        self.root.title('RoboMaster EP — ระบบควบคุมและสำรวจอัตโนมัติ')
        self.root.geometry('1220x780')
        self.root.attributes('-fullscreen', True)
        self.runner = runner
        self.messages = queue.Queue()
        self.control = None
        self.worker = None
        self.closing = False
        self.interrupt_requested = threading.Event()
        self.map_path = None
        self.map_mtime = None
        self.map_data = None
        self.proceed_to_explore = False
        self.selected_conn_type = 'ap'
        self.selected_fire_type = 'water_fire'
        self.selected_target_color = 'Red'
        self.selected_target_shape = 'All'
        self.fire_preview_active = False
        self.fire_preview_rgb = None
        self.fire_preview_lock = threading.Lock()
        self.preview_size = (720, 400)
        self.next_fire_preview_at = 0.0
        self.old_stdout, self.old_stderr = sys.stdout, sys.stderr
        self._build()
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self.root.bind('<Escape>', lambda _: self.root.attributes('-fullscreen', False))
        self.root.bind('<F11>', self._toggle_fullscreen)
        self.root.after(100, self._poll)
        self.root.after(33, self._render_fire_preview)
        self.root.after(400, self._refresh_map)
        self._draw_map()

    def _build(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use('clam')
        except Exception:
            pass
        
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill='both', expand=True)

        body = ttk.Frame(outer)
        body.pack(fill='both', expand=True)

        left = ttk.Frame(body, width=320)
        left.pack(side='left', fill='y', padx=(0, 14))

        right = ttk.Frame(body)
        right.pack(side='left', fill='both', expand=True)

        # ---------------- Section 1: Main Mission ----------------
        mission_box = ttk.LabelFrame(left, text=' ภารกิจหลัก (Main Mission) ', padding=8)
        mission_box.pack(fill='x', pady=(0, 10))

        ttk.Label(mission_box, text='โหมดการเชื่อมต่อหุ่นยนต์:').pack(anchor='w', pady=(0, 2))
        self.conn_mode = ttk.Combobox(mission_box, state='readonly',
                                      values=('AP (Wi-Fi หุ่นยนต์)', 'STA (เราเตอร์/เน็ตบ้าน)'))
        self.conn_mode.current(0 if setting('robot.conn_type') == 'ap' else 1)
        self.conn_mode.pack(fill='x', pady=(0, 8))

        ttk.Label(mission_box, text='ชนิดการยิงเป้าหมาย:').pack(anchor='w', pady=(0, 2))
        self.fire_mode = ttk.Combobox(
            mission_box, state='readonly', values=('water_fire', 'infared_fire')
        )
        self.fire_mode.current(0)
        self.fire_mode.pack(fill='x', pady=(0, 8))

        target_row = ttk.Frame(mission_box)
        target_row.pack(fill='x', pady=(0, 8))
        color_field = ttk.Frame(target_row)
        color_field.pack(side='left', fill='x', expand=True, padx=(0, 4))
        shape_field = ttk.Frame(target_row)
        shape_field.pack(side='left', fill='x', expand=True)
        ttk.Label(color_field, text='สีเป้าที่จะยิง:').pack(anchor='w', pady=(0, 2))
        self.target_color = ttk.Combobox(
            color_field, state='readonly',
            values=[label for label, _ in TARGET_COLOR_OPTIONS], width=12,
        )
        self.target_color.current(0)
        self.target_color.pack(fill='x')

        ttk.Label(shape_field, text='รูปร่างเป้าที่จะยิง:').pack(anchor='w', pady=(0, 2))
        self.target_shape = ttk.Combobox(
            shape_field, state='readonly',
            values=[label for label, _ in TARGET_SHAPE_OPTIONS], width=23,
        )
        self.target_shape.current(4)
        self.target_shape.pack(fill='x')

        self.start_explore_btn = ttk.Button(
            mission_box,
            text='เริ่มสำรวจ + เล็งยิงเป้าที่เลือก',
            command=lambda: self._select('explore-detect')
        )
        self.start_explore_btn.pack(fill='x', ipady=4, pady=(0, 4))

        self.direct_explore_btn = ttk.Button(
            mission_box,
            text='ปิด GUI แล้วเริ่มสำรวจและยิง',
            command=self._proceed_headless,
        )
        self.direct_explore_btn.pack(fill='x', pady=(0, 2))

        # ---------------- Section 2: Preparation & Testing ----------------
        tools_box = ttk.LabelFrame(left, text=' 🛠️ เตรียมความพร้อม / ทดสอบ ', padding=8)
        tools_box.pack(fill='x', pady=(0, 8))

        self.task_buttons = [self.start_explore_btn, self.direct_explore_btn]
        for label, task in [
            ('ตรวจจับเป้าหมายจากกล้องสด (ทดสอบกล้อง)', 'detect-camera'),
            ('ตรวจจับเป้าหมายจากเว็บแคม', 'detect-webcam'),
            ('ทดสอบเล็งและยิง (หุ่นไม่เดิน)', 'fire-test'),
            ('สำรวจ SLAM (เฉพาะเดิน ไม่ใช้กล้อง)', 'explore'),
            ('ทดสอบเดินหน้า 1 ช่อง', 'step-test'),
            ('ทดสอบเลี้ยว', 'turn-test'),
            ('ดูเซนเซอร์สด', 'monitor'),
            ('ทดสอบ Gimbal', 'gimbal-test'),
            ('Calibration เซนเซอร์', 'calibrate'),
            ('วิเคราะห์ Log', 'analysis'),
        ]:
            button = ttk.Button(tools_box, text=label, command=lambda t=task: self._select(t))
            button.pack(fill='x', pady=2)
            self.task_buttons.append(button)

        ttk.Label(tools_box, text='ทิศทางทดสอบเลี้ยว:').pack(anchor='w', pady=(6, 2))
        self.turn = ttk.Combobox(tools_box, state='readonly', values=('ขวา 90°', 'ซ้าย 90°', 'กลับหลัง 180°'))
        self.turn.current(0)
        self.turn.pack(fill='x')

        ttk.Label(tools_box, text='Calibration:').pack(anchor='w', pady=(6, 2))
        self.calibration = ttk.Combobox(tools_box, state='readonly',
            values=('Sharp ซ้าย', 'Sharp ขวา', 'ToF', 'คำนวณสมการ'))
        self.calibration.current(0)
        self.calibration.pack(fill='x')

        # ---------------- Section 3: Status & Control ----------------
        ctrl_box = ttk.LabelFrame(left, text=' สถานะและควบคุม ', padding=8)
        ctrl_box.pack(fill='x', pady=(4, 0))

        self.status = tk.StringVar(value='พร้อมทำงาน')
        ttk.Label(ctrl_box, textvariable=self.status, font=('', 10, 'italic'), wraplength=280).pack(anchor='w', pady=(0, 6))

        self.stop_button = ttk.Button(ctrl_box, text='🛑 หยุดงาน (Stop)', command=self._stop, state='disabled')
        self.stop_button.pack(fill='x', ipady=3)
        ttk.Label(ctrl_box, text='Esc: ออกจากเต็มจอ   F11: สลับเต็มจอ').pack(anchor='w', pady=(5, 0))

        # ---------------- Right Side: Live SLAM Map and Camera ----------------
        map_frame = ttk.Frame(right)
        map_frame.pack(side='left', fill='both', expand=True)
        self.map_status = tk.StringVar(value='ยังไม่มีแผนที่')
        ttk.Label(map_frame, textvariable=self.map_status, font=('', 11, 'bold')).pack(anchor='w', pady=(0, 5))

        self.canvas = tk.Canvas(map_frame, background='white', highlightthickness=1,
                                highlightbackground='#aaaaaa')
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Configure>', lambda _: self._draw_map())

        ttk.Label(map_frame, text='🟩 จุดเริ่ม   🟦 สำรวจแล้ว   🟧 หุ่นยนต์   ⬛ กำแพง   🔴/🔵 เป้าหมายที่พบ').pack(anchor='w', pady=4)

        self.preview_column = ttk.Frame(right, width=600)
        self.preview_column.pack_propagate(False)
        self.camera_frame = ttk.LabelFrame(self.preview_column, text=' กล้อง ', padding=4)
        self.camera_frame.pack(fill='both', expand=True, pady=(0, 4))
        self.camera_frame.bind('<Configure>', self._update_preview_size)
        self.fire_preview_label = ttk.Label(self.camera_frame, text='กำลังรอภาพจากกล้อง...', anchor='center')
        self.fire_preview_label.pack(fill='both', expand=True)
        self.mask_frame = ttk.LabelFrame(self.preview_column, text=' Mask ', padding=4)
        self.mask_frame.pack(fill='both', expand=True, pady=(4, 0))
        self.mask_preview_label = ttk.Label(self.mask_frame, text='กำลังรอภาพ mask...', anchor='center')
        self.mask_preview_label.pack(fill='both', expand=True)

        # ---------------- Bottom: Log View ----------------
        log_frame = ttk.LabelFrame(outer, text=' บันทึกการทำงาน (Log Output) ', padding=6)
        log_frame.pack(fill='x', pady=(8, 0))

        self.log = scrolledtext.ScrolledText(log_frame, height=7, state='disabled', wrap='word', background='#fbfbfb')
        self.log.pack(fill='x')

    def _proceed_headless(self):
        self.selected_conn_type = 'ap' if self.conn_mode.current() == 0 else 'sta'
        self.selected_fire_type = self.fire_mode.get()
        self.selected_target_color = TARGET_COLOR_OPTIONS[self.target_color.current()][1]
        self.selected_target_shape = TARGET_SHAPE_OPTIONS[self.target_shape.current()][1]
        self.proceed_to_explore = True
        self._close()

    def _select(self, task):
        params = {}
        if task == 'explore-detect':
            params['conn_type'] = 'ap' if self.conn_mode.current() == 0 else 'sta'
            params['fire_type'] = self.fire_mode.get()
            params['target_color'] = TARGET_COLOR_OPTIONS[self.target_color.current()][1]
            params['target_shape'] = TARGET_SHAPE_OPTIONS[self.target_shape.current()][1]
        elif task == 'fire-test':
            params['conn_type'] = 'ap' if self.conn_mode.current() == 0 else 'sta'
            params['fire_type'] = self.fire_mode.get()
            params['target_color'] = TARGET_COLOR_OPTIONS[self.target_color.current()][1]
            params['target_shape'] = TARGET_SHAPE_OPTIONS[self.target_shape.current()][1]
        elif task == 'detect-camera':
            params['mode'] = 'robot-ap' if self.conn_mode.current() == 0 else 'robot-sta'
        elif task == 'detect-webcam':
            params['mode'] = 'webcam'
        elif task == 'turn-test':
            params['direction'] = ('right', 'left', 'around')[self.turn.current()]
        elif task == 'motion':
            commands = getattr(self, 'commands', None)
            cmd_text = commands.get().strip() if commands else ''
            if not cmd_text:
                messagebox.showinfo('คำสั่งเดิน', 'กรอกคำสั่งเดินก่อนเริ่มงาน', parent=self.root)
                return
            params['commands'] = cmd_text
        elif task == 'calibrate':
            params['action'] = ('sharp_left', 'sharp_right', 'tof', 'fit')[self.calibration.current()]
        elif task == 'analysis':
            path = filedialog.askopenfilename(parent=self.root, title='เลือกไฟล์ Log JSON',
                initialdir=project_path('paths.telemetry'), filetypes=[('JSON', '*.json')])
            if not path:
                return
            params['file'] = path

        self.control = RunControl()
        if task in ('fire-test', 'explore-detect', 'detect-camera', 'detect-webcam'):
            self._open_fire_preview(task)
        self.status.set('กำลังทำงาน: ' + task)
        for button in self.task_buttons:
            button.configure(state='disabled')
        self.stop_button.configure(state='normal')
        self.worker = threading.Thread(target=self._run, args=(task, params, self.control), daemon=True)
        self.worker.start()

    def _run(self, task, params, control):
        try:
            result = self.runner(task, params, self, control)
            self.messages.put(('done', task, result))
        except Exception as exc:
            self.messages.put(('log', '[GUI] {}\n'.format(exc)))
            self.messages.put(('done', task, 1))

    def _stop(self):
        if self.control is not None:
            self.status.set('กำลังหยุดงาน...')
            self.control.stop()

    def _toggle_fullscreen(self, _event=None):
        fullscreen = bool(int(self.root.attributes('-fullscreen')))
        self.root.attributes('-fullscreen', not fullscreen)

    def _update_preview_size(self, event):
        if event.width > 80 and event.height > 80:
            with self.fire_preview_lock:
                self.preview_size = (min(PREVIEW_MAX_WIDTH, event.width - 16), event.height - 28)

    def _open_fire_preview(self, task):
        self.camera_frame.configure(text=' กล้อง ({}) '.format(task))
        self.preview_column.pack(side='right', fill='both', expand=True, padx=(8, 0))
        with self.fire_preview_lock:
            self.fire_preview_active = True
            self.fire_preview_rgb = None
            self.next_fire_preview_at = 0.0

    def _close_fire_preview(self):
        with self.fire_preview_lock:
            self.fire_preview_active = False
            self.fire_preview_rgb = None
        self.fire_preview_label.configure(image='', text='กำลังรอภาพจากกล้อง...')
        self.fire_preview_label.image = None
        self.mask_preview_label.configure(image='', text='กำลังรอภาพ mask...')
        self.mask_preview_label.image = None
        self.preview_column.pack_forget()

    def show_fire_frame(self, frame):
        """Keep only the latest camera frame for the Tk preview."""
        now = time.monotonic()
        with self.fire_preview_lock:
            if now < self.next_fire_preview_at or not self.fire_preview_active:
                return
            self.next_fire_preview_at = now + PREVIEW_INTERVAL_SEC
            target_width, target_height = self.preview_size
        import cv2
        height, width = frame.shape[:2]
        panel_width = (width - 3) // 2
        if panel_width <= 0:
            return
        panels = (frame[:, :panel_width], frame[:, -panel_width:])
        rgb_panels = []
        for panel in panels:
            scale = min(target_width / panel_width, target_height / height)
            target_size = (max(1, round(panel_width * scale)), max(1, round(height * scale)))
            if target_size != (panel_width, height):
                interpolation = cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
                panel = cv2.resize(panel, target_size, interpolation=interpolation)
            rgb_panels.append(cv2.cvtColor(panel, cv2.COLOR_BGR2RGB))
        with self.fire_preview_lock:
            if self.fire_preview_active:
                self.fire_preview_rgb = tuple(rgb_panels)

    def _render_fire_preview(self):
        with self.fire_preview_lock:
            panels = self.fire_preview_rgb if self.fire_preview_active else None
            self.fire_preview_rgb = None
        if panels is not None:
            for panel, label in zip(panels, (self.fire_preview_label, self.mask_preview_label)):
                photo = ImageTk.PhotoImage(Image.fromarray(panel), master=self.root)
                label.configure(image=photo, text='')
                label.image = photo
        self.root.after(33, self._render_fire_preview)

    def show_map(self, path):
        self.messages.put(('map', str(path)))

    def ask_reference(self, sensor, sample_id):
        if self.control and self.control.cancel.is_set():
            return None
        ready = threading.Event()
        answer = []
        self.messages.put(('reference', sensor, sample_id, ready, answer))
        ready.wait()
        return None if self.control and self.control.cancel.is_set() else answer[0]

    def _poll(self):
        if self.interrupt_requested.is_set():
            self.interrupt_requested.clear()
            self._close()
            if self.closing:
                return
        log_parts = []
        try:
            # Keep Tk responsive when the robot produces log messages continuously.
            for _ in range(100):
                message = self.messages.get_nowait()
                if message[0] == 'log':
                    log_parts.append(message[1])
                elif message[0] == 'map':
                    self.map_path = Path(message[1])
                    self.map_mtime = None
                    self.map_data = None
                    self._draw_map()
                elif message[0] == 'reference':
                    _, sensor, sample_id, ready, answer = message
                    answer.append(simpledialog.askfloat('Calibration',
                        '{} ตัวอย่าง {}: ระยะอ้างอิง (mm)'.format(sensor, sample_id),
                        minvalue=0.001, parent=self.root))
                    ready.set()
                elif message[0] == 'done':
                    _, task, result = message
                    if task in ('fire-test', 'explore-detect', 'detect-camera', 'detect-webcam'):
                        self._close_fire_preview()
                    self.status.set('{}: {}'.format(task, 'เสร็จสมบูรณ์' if result == 0 else 'หยุด/ไม่สำเร็จ'))
                    self.stop_button.configure(state='disabled')
                    for button in self.task_buttons:
                        button.configure(state='normal')
                    self.control = None
        except queue.Empty:
            pass
        if log_parts:
            self.log.configure(state='normal')
            self.log.insert('end', ''.join(log_parts))
            if int(self.log.index('end-1c').split('.')[0]) > 3000:
                self.log.delete('1.0', '1001.0')
            self.log.see('end')
            self.log.configure(state='disabled')
        self.root.after(100, self._poll)

    def _refresh_map(self):
        if self.map_path and self.map_path.exists():
            try:
                mtime = self.map_path.stat().st_mtime_ns
                if mtime != self.map_mtime:
                    self.map_data = json.loads(self.map_path.read_text(encoding='utf-8'))
                    self.map_mtime = mtime
                    self._draw_map()
            except (OSError, ValueError):
                pass
        self.root.after(400, self._refresh_map)

    def _draw_map(self):
        canvas = self.canvas
        canvas.delete('all')
        data = self.map_data
        rows, columns, start = map_geometry()
        if data:
            rows, columns = data['map_info']['rows'], data['map_info']['columns']
            start = data.get('start_cell', start)
        width, height = max(canvas.winfo_width(), 100), max(canvas.winfo_height(), 100)
        size = min((width - 50) / columns, (height - 50) / rows)
        x0 = (width - columns * size) / 2
        y0 = (height - rows * size) / 2
        def center(cell):
            return x0 + (cell[1] + 0.5) * size, y0 + (rows - cell[0] - 0.5) * size

        visited = set(map(tuple, data.get('visited', []))) if data else set()
        for row in range(rows):
            for column in range(columns):
                x = x0 + column * size
                y = y0 + (rows - row - 1) * size
                canvas.create_rectangle(x, y, x + size, y + size,
                    fill='#dff2fa' if (row, column) in visited else '#f8f9fa',
                    outline='#c5cbd0')
                if size >= 35:
                    canvas.create_text(x + 5, y + 5, text='{},{}'.format(row, column),
                        anchor='nw', fill='#606870', font=('', 9))

        # Draw walls
        if data:
            for edge in data.get('edges', []):
                if not edge['wall']:
                    continue
                a, b = edge['cells']
                if a[0] != b[0]:
                    row = max(a[0], b[0])
                    column = a[1]
                    x1, y1 = x0 + column * size, y0 + (rows - row) * size
                    x2, y2 = x1 + size, y1
                else:
                    row = a[0]
                    column = max(a[1], b[1])
                    x1, y1 = x0 + column * size, y0 + (rows - row - 1) * size
                    x2, y2 = x1, y1 + size
                canvas.create_line(x1, y1, x2, y2, fill='#172b3a', width=4)

        # Draw start position
        sx, sy = center(start)
        canvas.create_rectangle(sx - 7, sy - 7, sx + 7, sy + 7, fill='#2d9965', outline='')

        # Draw detected signs
        if data and 'signs' in data:
            offsets = {0: (0, -size * 0.35), 1: (size * 0.35, 0), 2: (0, size * 0.35), 3: (-size * 0.35, 0)}
            for s in data['signs']:
                s_cell = s.get('cell')
                s_col = s.get('color', 'black').lower()
                s_shape = s.get('shape', 'sign')
                s_dir = s.get('direction', 0)
                if s_cell and len(s_cell) == 2:
                    if isinstance(s_dir, str):
                        if s_dir not in ('N', 'E', 'S', 'W'):
                            continue
                        s_dir = 'NESW'.index(s_dir)
                    elif not isinstance(s_dir, int):
                        continue
                    cx, cy = center(s_cell)
                    dx, dy = offsets.get(s_dir % 4, (0, 0))
                    dot_color = '#e02424' if 'red' in s_col else ('#1c64f2' if 'blue' in s_col else '#333333')
                    canvas.create_oval(cx + dx - 6, cy + dy - 6, cx + dx + 6, cy + dy + 6,
                                       fill=dot_color, outline='white', width=1)
                    canvas.create_text(cx + dx, cy + dy, text=s_shape[:1].upper(), fill='white', font=('', 7, 'bold'))

        # Draw robot
        if data:
            row, column = data['cell']
            heading = data.get('heading', 0)
            if 0 <= row < rows and 0 <= column < columns:
                x, y = center((row, column))
                canvas.create_oval(x - 11, y - 11, x + 11, y + 11,
                                   fill='#ef8a24', outline='#8d4b00', width=2)
                h_offsets = {0: (0, -12), 1: (12, 0), 2: (0, 12), 3: (-12, 0)}
                hx, hy = h_offsets.get(heading % 4, (0, -12))
                canvas.create_line(x, y, x + hx, y + hy, fill='white', width=3)

            sign_count = len(data.get('signs', []))
            self.map_status.set('สถานะ: {} | สำรวจ {}/{} ช่อง | เป้าหมาย: {} ป้าย | หุ่นอยู่ ({},{})'.format(
                data['status'], len(visited), rows * columns, sign_count, row, column))

    def _close(self):
        if not self.closing:
            self.closing = True
            if self.worker and self.worker.is_alive():
                self._stop()
        if self.worker and self.worker.is_alive():
            self.root.after(200, self._close)
            return
        self.root.destroy()

    def run(self):
        log_path = Path(project_path('paths.telemetry')) / ('gui_session_{}.log'.format(
            time.strftime('%Y%m%d_%H%M%S')))
        writer = _GuiWriter(self.messages, log_path)
        previous_handlers = {}
        def request_stop(_signum, _frame):
            self.interrupt_requested.set()
        try:
            stop_signals = [signal.SIGINT, signal.SIGTERM]
            if hasattr(signal, 'SIGHUP'):
                stop_signals.append(signal.SIGHUP)
            for signum in stop_signals:
                previous_handlers[signum] = signal.getsignal(signum)
                signal.signal(signum, request_stop)
            sys.stdout = writer
            sys.stderr = writer
            self.root.mainloop()
        finally:
            sys.stdout, sys.stderr = self.old_stdout, self.old_stderr
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
            writer.close()
