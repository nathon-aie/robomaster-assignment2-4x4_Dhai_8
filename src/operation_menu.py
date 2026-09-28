"""Interactive operation selection; configuration stays in settings.yaml."""
from pathlib import Path
from .settings import project_path


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
    return choose('Calibration', [
        ('เก็บตัวอย่าง Sharp ซ้ายจากหุ่นจริง', 'sharp_left'),
        ('เก็บตัวอย่าง Sharp ขวาจากหุ่นจริง', 'sharp_right'),
        ('เก็บตัวอย่าง ToF จากหุ่นจริง', 'tof'),
        ('คำนวณสมการจากข้อมูลที่เก็บไว้', 'fit'),
    ])


def select_operation():
    """Return (task, function parameters), or None on cancellation."""
    try:
        task = choose('RoboMaster EP — เลือกงาน (ค่าพื้นฐานอ่านจาก config/settings.yaml)', [
            ('สำรวจและสร้างแผนที่ SLAM + DFS', 'explore'),
            ('ทดสอบเดินหน้า 1 ช่อง', 'step-test'),
            ('ทดสอบเลี้ยว', 'turn-test'),
            ('ดูข้อมูลเซนเซอร์สด', 'monitor'),
            ('ทดสอบชุดคำสั่งเคลื่อนที่', 'motion'),
            ('Calibration เซนเซอร์', 'calibrate'),
            ('วิเคราะห์ผลการสำรวจ / Log', 'analysis'),
            ('ทดสอบเฉพาะ Gimbal (หุ่นจริง ไม่เดิน)', 'gimbal-test'),
        ])
        if task is None:
            return None
        if task == 'gimbal-test':
            return task, {}
        if task in ('explore', 'step-test', 'turn-test', 'monitor', 'motion'):
            if task == 'motion':
                commands = input('คำสั่งเคลื่อนที่ เช่น fwd 1, right, fwd 1 (เว้นว่างเพื่อยกเลิก): ').strip()
                if not commands:
                    return None
                return task, {'commands': commands}
            parameters = {}
            if task == 'turn-test':
                direction = choose('เลือกการเลี้ยว', [
                    ('ขวา 90 องศา', 'right'), ('ซ้าย 90 องศา', 'left'), ('กลับหลัง 180 องศา', 'around')])
                if direction is None:
                    return None
                parameters['direction'] = direction
            return task, parameters
        if task == 'calibrate':
            action = calibration_action()
            if action is None:
                return None
            return task, {'action': action}
        base = Path(project_path('paths.telemetry'))
        runs = [p for p in base.glob('run*') if p.is_dir() and any(p.glob('*.json'))]
        runs += list(base.glob('*.json'))
        runs.sort(key=lambda path: path.stat().st_mtime, reverse=True)
        if not runs:
            print('ยังไม่มี Log สำหรับวิเคราะห์ใน {}'.format(base))
            return None
        selected = choose('เลือก Log ที่ต้องการวิเคราะห์', [(p.name, str(p)) for p in runs])
        return (task, {'file': selected}) if selected else None
    except (EOFError, KeyboardInterrupt):
        print('\nออกจากเมนู')
        return None
