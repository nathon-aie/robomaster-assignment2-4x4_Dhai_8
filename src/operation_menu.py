"""Clickable task menu and live grid map for the RoboMaster operator."""
import json
import queue
import sys
import threading
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, simpledialog, ttk

from .settings import map_geometry, project_path


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
        self.root.title('RoboMaster EP')
        self.root.geometry('1050x700')
        self.runner = runner
        self.messages = queue.Queue()
        self.control = None
        self.worker = None
        self.map_path = None
        self.map_mtime = None
        self.map_data = None
        self.old_stdout, self.old_stderr = sys.stdout, sys.stderr
        self._build()
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self.root.after(100, self._poll)
        self.root.after(400, self._refresh_map)
        self._draw_map()

    def _build(self):
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill='both', expand=True)
        body = ttk.Frame(outer)
        body.pack(fill='both', expand=True)
        left = ttk.Frame(body)
        left.pack(side='left', fill='y', padx=(0, 12))
        right = ttk.Frame(body)
        right.pack(side='left', fill='both', expand=True)
        ttk.Label(left, text='เลือกงาน', font=('', 14, 'bold')).pack(anchor='w', pady=(0, 8))
        self.task_buttons = []
        for label, task in [
            ('สำรวจและสร้างแผนที่', 'explore'),
            ('ทดสอบเดินหน้า 1 ช่อง', 'step-test'),
            ('ทดสอบเลี้ยว', 'turn-test'),
            ('ดูเซนเซอร์สด', 'monitor'),
            ('ทดสอบชุดคำสั่งเดิน', 'motion'),
            ('Calibration เซนเซอร์', 'calibrate'),
            ('วิเคราะห์ Log', 'analysis'),
            ('ทดสอบ Gimbal', 'gimbal-test'),
        ]:
            button = ttk.Button(left, text=label, command=lambda t=task: self._select(t))
            button.pack(fill='x', pady=3)
            self.task_buttons.append(button)
        ttk.Label(left, text='ทิศทางทดสอบเลี้ยว').pack(anchor='w', pady=(12, 2))
        self.turn = ttk.Combobox(left, state='readonly', values=('ขวา 90°', 'ซ้าย 90°', 'กลับหลัง 180°'))
        self.turn.current(0)
        self.turn.pack(fill='x')
        ttk.Label(left, text='คำสั่งเดิน เช่น fwd 1, right').pack(anchor='w', pady=(12, 2))
        self.commands = ttk.Entry(left)
        self.commands.pack(fill='x')
        ttk.Label(left, text='Calibration').pack(anchor='w', pady=(12, 2))
        self.calibration = ttk.Combobox(left, state='readonly', values=(
            'Sharp ซ้าย', 'Sharp ขวา', 'ToF', 'คำนวณสมการ'))
        self.calibration.current(0)
        self.calibration.pack(fill='x')
        self.stop_button = ttk.Button(left, text='หยุดงาน', command=self._stop, state='disabled')
        self.stop_button.pack(fill='x', pady=(20, 4))
        ttk.Label(right, text='แผนที่สำรวจขณะทำงาน', font=('', 14, 'bold')).pack(anchor='w')
        self.status = tk.StringVar(value='พร้อมใช้งาน')
        ttk.Label(right, textvariable=self.status).pack(anchor='w', pady=(2, 2))
        self.map_status = tk.StringVar(value='ยังไม่มีแผนที่')
        ttk.Label(right, textvariable=self.map_status).pack(anchor='w', pady=(0, 5))
        self.canvas = tk.Canvas(right, background='white', highlightthickness=1,
                                highlightbackground='#aaaaaa')
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Configure>', lambda _: self._draw_map())
        ttk.Label(right, text='ฟ้า: สำรวจแล้ว   ส้ม: หุ่น   เขียว: จุดเริ่ม   เส้นดำ: กำแพง').pack(anchor='w', pady=4)
        log_frame = ttk.Frame(outer)
        log_frame.pack(fill='x', pady=(8, 0))
        ttk.Label(log_frame, text='บันทึกการทำงาน').pack(anchor='w')
        self.log = scrolledtext.ScrolledText(log_frame, height=8, state='disabled', wrap='word')
        self.log.pack(fill='x')

    def _select(self, task):
        params = {}
        if task == 'turn-test':
            params['direction'] = ('right', 'left', 'around')[self.turn.current()]
        elif task == 'motion':
            commands = self.commands.get().strip()
            if not commands:
                messagebox.showinfo('คำสั่งเดิน', 'กรอกคำสั่งเดินก่อนเริ่มงาน', parent=self.root)
                return
            params['commands'] = commands
        elif task == 'calibrate':
            params['action'] = ('sharp_left', 'sharp_right', 'tof', 'fit')[self.calibration.current()]
        elif task == 'analysis':
            path = filedialog.askopenfilename(parent=self.root, title='เลือกไฟล์ Log JSON',
                initialdir=project_path('paths.telemetry'), filetypes=[('JSON', '*.json')])
            if not path:
                return
            params['file'] = path
        self.control = RunControl()
        self.status.set('กำลังทำงาน: ' + task)
        for button in self.task_buttons:
            button.configure(state='disabled')
        self.stop_button.configure(state='normal')
        self.worker = threading.Thread(target=self._run, args=(task, params, self.control))
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
                    self.status.set('{}: {}'.format(task, 'เสร็จแล้ว' if result == 0 else 'ไม่สำเร็จ/ถูกหยุด'))
                    self.stop_button.configure(state='disabled')
                    for button in self.task_buttons:
                        button.configure(state='normal')
                    self.control = None
        except queue.Empty:
            pass
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
                pass  # A new snapshot will be read on the next tick.
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
                    fill='#dff2fa' if (row, column) in visited else '#f3f3f3',
                    outline='#c5cbd0')
                if size >= 35:
                    canvas.create_text(x + 5, y + 5, text='{},{}'.format(row, column),
                        anchor='nw', fill='#606870', font=('', 9))
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
        sx, sy = center(start)
        canvas.create_rectangle(sx - 7, sy - 7, sx + 7, sy + 7, fill='#2d9965', outline='')
        if data:
            row, column = data['cell']
            if 0 <= row < rows and 0 <= column < columns:
                x, y = center((row, column))
                canvas.create_oval(x - 11, y - 11, x + 11, y + 11,
                                   fill='#ef8a24', outline='#8d4b00', width=2)
            self.map_status.set('{} | สำรวจ {}/{} ช่อง | หุ่นอยู่ ({},{})'.format(
                data['status'], len(visited), rows * columns, row, column))

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
