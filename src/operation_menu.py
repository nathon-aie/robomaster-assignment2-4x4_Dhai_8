"""Interactive operation selection and live GUI for RoboMaster EP."""
import base64
import json
import queue
import sys
import threading
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, simpledialog, ttk

from .settings import get as setting, map_geometry, project_path


class RunControl:
    def __init__(self):
        self.cancel = threading.Event()
        self.system = None

    def set_system(self, system):
        self.system = system
        if self.cancel.is_set():
            self.stop()

    def stop(self):
        self.cancel.set()
        controller = getattr(self.system, 'thread_2_controller', None)
        if controller is not None:
            threading.Thread(target=controller.stop_running, daemon=True).start()


class _GuiWriter:
    def __init__(self, messages):
        self.messages = messages

    def write(self, value):
        if value:
            self.messages.put(('log', value))

    def flush(self):
        pass


class OperationGUI:
    def __init__(self, runner):
        self.root = tk.Tk()
        self.root.title('RoboMaster EP — ระบบควบคุมและสำรวจอัตโนมัติ')
        self.root.geometry('1100x740')
        self.runner = runner
        self.messages = queue.Queue()
        self.control = None
        self.worker = None
        self.map_path = None
        self.map_mtime = None
        self.map_data = None
        self.proceed_to_explore = False
        self.selected_conn_type = 'ap'
        self.selected_fire_type = 'water_fire'
        self.fire_preview_window = None
        self.fire_preview_label = None
        self.fire_preview_frame = None
        self.fire_preview_lock = threading.Lock()
        self.old_stdout, self.old_stderr = sys.stdout, sys.stderr
        self._build()
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self.root.after(100, self._poll)
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
        mission_box = ttk.LabelFrame(left, text=' 🚀 ภารกิจหลัก (Main Mission) ', padding=8)
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

        self.start_explore_btn = ttk.Button(
            mission_box,
            text='🚀 เริ่มสำรวจ + เล็งยิงทุกเป้าหมาย',
            command=lambda: self._select('explore-detect')
        )
        self.start_explore_btn.pack(fill='x', ipady=4, pady=(0, 4))

        self.direct_explore_btn = ttk.Button(
            mission_box,
            text='🚪 ปิด GUI แล้วเริ่มสำรวจและยิง',
            command=self._proceed_headless
        )
        self.direct_explore_btn.pack(fill='x', pady=(0, 2))

        # ---------------- Section 2: Preparation & Testing ----------------
        tools_box = ttk.LabelFrame(left, text=' 🛠️ เตรียมความพร้อม / ทดสอบ ', padding=8)
        tools_box.pack(fill='x', pady=(0, 8))

        self.task_buttons = [self.start_explore_btn]
        for label, task in [
            ('🎯 ตรวจจับเป้าหมายจากกล้องสด (ทดสอบกล้อง)', 'detect-camera'),
            ('💧 ทดสอบเล็งและยิง (หุ่นไม่เดิน)', 'fire-test'),
            ('🗺️ สำรวจ SLAM (เฉพาะเดิน ไม่ใช้กล้อง)', 'explore'),
            ('👣 ทดสอบเดินหน้า 1 ช่อง', 'step-test'),
            ('🔄 ทดสอบเลี้ยว', 'turn-test'),
            ('📡 ดูเซนเซอร์สด', 'monitor'),
            ('🕹️ ทดสอบ Gimbal', 'gimbal-test'),
            ('Calibration เซนเซอร์', 'calibrate'),
            ('📊 วิเคราะห์ Log', 'analysis'),
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

        # ---------------- Right Side: Live SLAM Map ----------------
        self.map_status = tk.StringVar(value='ยังไม่มีแผนที่')
        ttk.Label(right, textvariable=self.map_status, font=('', 11, 'bold')).pack(anchor='w', pady=(0, 5))

        self.canvas = tk.Canvas(right, background='white', highlightthickness=1,
                                highlightbackground='#aaaaaa')
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Configure>', lambda _: self._draw_map())

        ttk.Label(right, text='🟩 จุดเริ่ม   🟦 สำรวจแล้ว   🟧 หุ่นยนต์   ⬛ กำแพง   🔴/🔵 เป้าหมายที่พบ').pack(anchor='w', pady=4)

        # ---------------- Bottom: Log View ----------------
        log_frame = ttk.LabelFrame(outer, text=' บันทึกการทำงาน (Log Output) ', padding=6)
        log_frame.pack(fill='x', pady=(8, 0))

        self.log = scrolledtext.ScrolledText(log_frame, height=7, state='disabled', wrap='word', background='#fbfbfb')
        self.log.pack(fill='x')

    def _proceed_headless(self):
        """Close GUI and tell main() to run exploration on the main thread."""
        self.selected_conn_type = 'ap' if self.conn_mode.current() == 0 else 'sta'
        self.selected_fire_type = self.fire_mode.get()
        self.proceed_to_explore = True
        self._close()

    def _select(self, task):
        params = {}
        if task == 'explore-detect':
            params['conn_type'] = 'ap' if self.conn_mode.current() == 0 else 'sta'
            params['fire_type'] = self.fire_mode.get()
        elif task == 'fire-test':
            params['conn_type'] = 'ap' if self.conn_mode.current() == 0 else 'sta'
            params['fire_type'] = self.fire_mode.get()
        elif task == 'detect-camera':
            params['mode'] = 'robot-ap' if self.conn_mode.current() == 0 else 'robot-sta'
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
        if task in ('fire-test', 'explore-detect', 'detect-camera'):
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

    def _open_fire_preview(self, task):
        window = tk.Toplevel(self.root)
        window.title('RoboMaster — ภาพกล้อง ({})'.format(task))
        window.protocol('WM_DELETE_WINDOW', self._stop_fire_preview)
        self.fire_preview_window = window
        self.fire_preview_label = ttk.Label(window, text='กำลังรอภาพจากกล้อง...')
        self.fire_preview_label.pack(padx=8, pady=8)

    def _stop_fire_preview(self):
        self._stop()
        self._close_fire_preview()

    def _close_fire_preview(self):
        if self.fire_preview_window is not None:
            self.fire_preview_window.destroy()
            self.fire_preview_window = None
            self.fire_preview_label = None
        with self.fire_preview_lock:
            self.fire_preview_frame = None

    def show_fire_frame(self, frame):
        """Accept a camera frame from the robot worker; Tk renders it on its own thread."""
        with self.fire_preview_lock:
            self.fire_preview_frame = frame

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
        try:
            while True:
                message = self.messages.get_nowait()
                if message[0] == 'log':
                    self.log.configure(state='normal')
                    self.log.insert('end', message[1])
                    self.log.see('end')
                    self.log.configure(state='disabled')
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
                    if task in ('fire-test', 'explore-detect', 'detect-camera'):
                        self._close_fire_preview()
                    self.status.set('{}: {}'.format(task, 'เสร็จสมบูรณ์' if result == 0 else 'หยุด/ไม่สำเร็จ'))
                    self.stop_button.configure(state='disabled')
                    for button in self.task_buttons:
                        button.configure(state='normal')
                    self.control = None
        except queue.Empty:
            pass
        if self.fire_preview_label is not None:
            with self.fire_preview_lock:
                frame = self.fire_preview_frame
                self.fire_preview_frame = None
            if frame is not None:
                import cv2
                height, width = frame.shape[:2]
                if width > 1050:
                    frame = cv2.resize(frame, (1050, round(height * 1050 / width)))
                encoded, png = cv2.imencode('.png', frame)
                if encoded:
                    photo = tk.PhotoImage(
                        data=base64.b64encode(png.tobytes()).decode('ascii'), format='png')
                    self.fire_preview_label.configure(image=photo, text='')
                    self.fire_preview_label.image = photo
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
        if self.worker and self.worker.is_alive():
            self._stop()
            self.root.after(200, self._close)
            return
        sys.stdout, sys.stderr = self.old_stdout, self.old_stderr
        self.root.destroy()

    def run(self):
        sys.stdout = _GuiWriter(self.messages)
        sys.stderr = _GuiWriter(self.messages)
        self.root.mainloop()


# Terminal / CLI interactive functions:
def choose(title, options):
    print('\n' + title)
    for number, (label, _) in enumerate(options, 1):
        print('  {}. {}'.format(number, label))
    print('  0. ออก / ยกเลิก')
    while True:
        value = input('เลือกหมายเลข: ').strip()
        if value == '0':
            return None
        if value.isdigit() and 1 <= int(value) <= len(options):
            return options[int(value) - 1][1]
        print('กรุณาเลือกหมายเลข 0–{}'.format(len(options)))


def calibration_action():
    return choose('เลือก Calibration', [
        ('Sharp ซ้าย', 'sharp_left'),
        ('Sharp ขวา', 'sharp_right'),
        ('ToF', 'tof'),
        ('คำนวณสมการ', 'fit'),
    ])


def select_operation():
    """Return (task, function parameters), or None on cancellation."""
    try:
        task = choose('RoboMaster EP — เลือกงาน (ค่าพื้นฐานอ่านจาก config/settings.yaml)', [
            ('สำรวจและสร้างแผนที่ SLAM + BFS', 'explore'),
            ('สำรวจ SLAM + เล็งยิงทุกเป้าหมาย', 'explore-detect'),
            ('ทดสอบตรวจจับเป้าหมายจากกล้อง (Auto Capture 3 รูป)', 'detect-camera'),
            ('ทดสอบเล็งและยิงเป้า (หุ่นไม่เดิน)', 'fire-test'),
            ('ทดสอบเดินหน้า 1 ช่อง', 'step-test'),
            ('ทดสอบเลี้ยว', 'turn-test'),
            ('ดูเซนเซอร์สด', 'monitor'),
            ('ทดสอบชุดคำสั่งเดิน', 'motion'),
            ('Calibration เซนเซอร์', 'calibrate'),
            ('วิเคราะห์ Log', 'analysis'),
            ('ทดสอบ Gimbal', 'gimbal-test'),
        ])
        if task is None:
            return None
        if task in ('explore-detect', 'fire-test'):
            mode = choose('เลือกชนิดการยิงเป้าหมาย', [
                ('ยิงกระสุนน้ำ', 'water_fire'),
                ('ยิงอินฟราเรด', 'infared_fire'),
            ])
            return (task, {'fire_type': mode}) if mode is not None else None
        if task == 'detect-camera':
            camera = choose('เลือกกล้องสำหรับตรวจจับ', [
                ('กล้องหุ่นยนต์ RoboMaster (AP mode)', 'robot-ap'),
                ('กล้องหุ่นยนต์ RoboMaster (STA mode)', 'robot-sta'),
                ('เว็บแคมของคอมพิวเตอร์ (PC Webcam)', 'webcam'),
            ])
            return (task, {'mode': camera}) if camera is not None else None
        if task == 'turn-test':
            direction = choose('เลือกการเลี้ยว', [
                ('ขวา 90 องศา', 'right'),
                ('ซ้าย 90 องศา', 'left'),
                ('กลับหลัง 180 องศา', 'around'),
            ])
            return (task, {'direction': direction}) if direction is not None else None
        if task == 'motion':
            commands = input('คำสั่งเคลื่อนที่ เช่น fwd 1, right, fwd 1 (เว้นว่างเพื่อยกเลิก): ').strip()
            return (task, {'commands': commands}) if commands else None
        if task == 'calibrate':
            action = calibration_action()
            return (task, {'action': action}) if action is not None else None
        if task == 'analysis':
            base = Path(project_path('paths.telemetry'))
            runs = [p for p in base.glob('run*') if p.is_dir() and any(p.glob('*.json'))]
            runs += list(base.glob('*.json'))
            runs.sort(key=lambda path: path.stat().st_mtime, reverse=True)
            if not runs:
                print('ยังไม่มี Log สำหรับวิเคราะห์ใน {}'.format(base))
                return None
            selected = choose('เลือก Log ที่ต้องการวิเคราะห์', [(p.name, str(p)) for p in runs])
            return (task, {'file': selected}) if selected else None
        return task, {}
    except (EOFError, KeyboardInterrupt):
        print('\nออกจากเมนู')
        return None
