"""Post-run map, trajectory and action reports."""
import csv
import json
import math
from html import escape
from pathlib import Path
from .grid_slam import NAMES


def action_steps(data):
    events = data['events']
    marked = any(e['type'] == 'motion_start' for e in events)
    boundaries, last_scan = [(0, None)], -1
    for i, e in enumerate(events):
        if e['type'] == 'scan':
            last_scan = i
        if e['type'] == ('motion_start' if marked else 'move'):
            boundaries.append((i if marked else last_scan + 1, e))
    steps, heading = [], 0
    for n, (begin, motion) in enumerate(boundaries):
        end = boundaries[n + 1][0] if n + 1 < len(boundaries) else len(events)
        actions = []
        delta = 0
        if motion:
            delta = (motion['direction'] - heading) % 4
            if delta and not any(e['type'] == 'chassis_turn' for e in events[begin:end]):
                actions.append('TURN (planned/inferred) ' + {1: 'right 90 deg', 2: 'around 180 deg', 3: 'left 90 deg'}[delta])
            heading = motion['direction']
        for e in events[begin:end]:
            kind = e['type']
            if kind == 'hardware_action' and e['name'] != 'Gimbal move action':
                actions.append('{} [{}]'.format(e['name'], 'OK' if e['succeeded'] else 'FAILED'))
            elif kind == 'gimbal' and e.get('phase') == 'move':
                actions.append('GIMBAL move {:+g} deg -> {:+g} deg'.format(e['delta_yaw'], e['target_yaw']))
            elif kind == 'tof_sample':
                actions.append('ToF {:+g} deg: {:.0f} mm'.format(e['gimbal_yaw'], e['range_m'] * 1000))
            elif kind == 'move':
                actions.append('ARRIVED {} -> {}{}'.format(tuple(e['from']), tuple(e['to']), ' BACKTRACK' if e['backtrack'] else ''))
            elif kind == 'chassis_turn':
                degrees = e['degrees']
                actions.append('TURN {} {} deg'.format('right' if degrees < 0 else 'around' if abs(degrees) == 180 else 'left', abs(degrees)))
            elif kind == 'walk_start':
                actions.append('WALK {} | nominal {:.2f} m | Sharp centering + ToF guard'.format(NAMES[e['direction']], e['cell_size_m']))
            elif kind == 'walk_end':
                actions.append('WALK stop: {} | actual {:.2f} m | {}'.format(e['reason'], e['distance_m'], 'OK' if e['completed'] else 'FAILED'))
            elif kind == 'chassis_wheel_stop':
                actions.append('STOP wheels [ACK]')
            elif kind == 'scan':
                actions.append('MAP cell {}'.format(tuple(e['cell'])))
                heading = round(e['pose'][2] / 90) % 4
            elif kind == 'scan_skipped':
                actions.append('MAP reuse cell {} (already scanned)'.format(tuple(e['cell'])))
            elif kind == 'finish':
                actions.append('END: {}{}'.format(e['status'], ' | ' + e['error'] if e.get('error') else ''))
            elif kind in ('range_mismatch', 'wall_mismatch', 'odometry_mismatch', 'scan_position_drift', 'scan_pose_wait', 'scan_pose_recovered'):
                actions.append('NOTE: ' + kind)
        steps.append({'completed': n == 0 or any(e['type'] == 'move' for e in events[begin:end]), 'step': n, 'from': motion['from'] if motion else data.get('start_cell', [0, 0]),
                      'to': motion['to'] if motion else data.get('start_cell', [0, 0]), 'actions': actions,
                      'turn': delta, 'backtrack': bool(motion and motion.get('backtrack')),
                      'direction': motion['direction'] if motion else heading,
                      'distance_m': next((sum(v * d for v, d in zip(e['odometry_delta_m'],
                          [(1, 0), (0, 1), (-1, 0), (0, -1)][e['direction']]))
                          for e in events[begin:end] if e['type'] == 'move'), None),
                      'warnings': sum(e['type'] in ('range_mismatch', 'wall_mismatch', 'odometry_mismatch',
                          'scan_position_drift') for e in events[begin:end])})
    return steps


def save_actions_html(data, output):
    """Standalone, searchable step list; no network or external assets required."""
    steps = action_steps(data)
    cards = []
    for step in steps:
        items = ''.join('<li>{}</li>'.format(escape(action)) for action in step['actions'])
        if step['step'] == 0:
            title = 'เริ่มต้นที่ช่อง {}'.format(tuple(step['to']))
            operation = 'ตั้ง Gimbal กลาง → สแกนรอบตัว → บันทึกแผนที่'
            badge = 'จุดเริ่มต้น'
        else:
            title = '{} → {}'.format(tuple(step['from']), tuple(step['to']))
            turn = {0: 'เดินตรง', 1: 'เลี้ยวขวา 90° แล้วเดิน',
                    2: 'กลับหลัง 180° แล้วเดิน', 3: 'เลี้ยวซ้าย 90° แล้วเดิน'}[step['turn']]
            operation = turn + ' 1 ช่อง'
            if step['distance_m'] is not None:
                operation += ' · เดินจริง {:.1f} ซม.'.format(step['distance_m'] * 100)
            operation += ' → หยุด → สแกนรอบตัว' if step['completed'] else ' · ยังไม่ยืนยันถึงช่อง'
            badge = 'ย้อนกลับ' if step['backtrack'] else 'สำรวจทางใหม่'
        status = 'ถึงช่องแล้ว' if step['completed'] and step['step'] else 'เริ่มสำรวจ' if step['step'] == 0 else 'ไม่สำเร็จ'
        notes = '<span class="warning">มีข้อสังเกต {} รายการ</span>'.format(step['warnings']) if step['warnings'] else ''
        cards.append('<article class="step"><div class="heading"><span class="number">{}</span>'
                     '<div><h2>{}</h2><span class="badge">{}</span> <span class="status">{}</span></div></div>'
                     '<p class="operation">{}</p>{}<details><summary>ดูรายละเอียด Gimbal / ToF และ log</summary>'
                     '<ol>{}</ol></details></article>'.format(step['step'], escape(title), badge, status,
                                                              escape(operation), notes, items))
    header = '{} · สำรวจ {} ช่อง · เดินสำเร็จ {} ครั้ง'.format(data['status'], len(data['visited']),
        sum(e['type'] == 'move' for e in data['events']))
    page = """<!doctype html><html lang="th"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>รายการ Step และ Action</title><style>
body{margin:0;background:#f4f6f8;color:#203040;font:16px/1.65 system-ui,sans-serif}
main{max-width:1000px;margin:32px auto;padding:0 24px}h1{margin-bottom:0;font-size:28px}
.meta{color:#526373}nav{position:sticky;top:0;background:#f4f6f8;padding:12px 0;display:flex;gap:8px;flex-wrap:wrap}
input,button{font:inherit;border:1px solid #bbc9d2;border-radius:8px;padding:8px 12px;background:white}
input{flex:1;min-width:180px}button{cursor:pointer}
.step{background:white;border:1px solid #dbe3e8;border-radius:12px;margin:16px 0;padding:18px 22px}
.heading{display:flex;align-items:center;gap:16px}.number{background:#124c72;color:white;border-radius:10px;padding:8px 12px;font-size:24px;min-width:32px;text-align:center}
h2{font-size:23px;margin:0 0 5px}.badge{background:#e4f1f8;border-radius:6px;padding:3px 8px;font-size:14px}
.status{color:#527363;font-size:14px}.operation{font-size:18px;margin:16px 0 8px}.warning{color:#996000;font-size:14px}
details{border-top:1px solid #e7edf1;margin-top:12px}article[hidden]{display:none}
summary{padding:14px 18px;font-weight:650;color:#124c72;cursor:pointer}ol{padding:0 32px 16px 52px}
li{padding:6px;border-top:1px solid #eef1f4;overflow-wrap:anywhere}a{color:#126fa4}
@media print{nav{display:none}details{break-inside:avoid}body{background:white}}
</style><main><h1>รายการ Step / Action</h1><p class="meta">HEADER</p>
<p>เลขบนการ์ดตรงกับเลข Step บน map · แต่ละการ์ดสรุปการเดินหนึ่งครั้ง
รายละเอียดเซนเซอร์พับไว้ด้านล่าง เพื่อให้อ่านเส้นทางได้เร็ว
พิกัด (row, column) คือ (แถว, คอลัมน์) · row เพิ่มขึ้นด้านบน และ column เพิ่มไปทางขวา · <a href="map.png">เปิดแผนที่</a></p>
<p class="meta">TURN (planned/inferred) คือคำสั่งที่วางแผนหรืออนุมานจาก log เก่า
ARRIVED คือยืนยันถึงช่องแล้ว; FAILED คือคำสั่งไม่สำเร็จ</p>
<nav><input id="search" type="search" placeholder="ค้นหา step, พิกัด หรือ action" aria-label="ค้นหา">
<button onclick="toggleAll(true)">เปิดรายละเอียดทั้งหมด</button><button onclick="toggleAll(false)">พับรายละเอียดทั้งหมด</button></nav>
CARDS</main><script>
const steps=Array.from(document.querySelectorAll('.step'));
function toggleAll(open){steps.forEach(s=>{if(!s.hidden)s.querySelector('details').open=open})}
document.getElementById('search').addEventListener('input',e=>{let q=e.target.value.trim().toLowerCase();steps.forEach(s=>{s.hidden=!s.textContent.toLowerCase().includes(q);if(q&&!s.hidden)s.querySelector('details').open=true})});

</script></html>"""
    Path(output).write_text(page.replace('HEADER', escape(header)).replace('CARDS', ''.join(cards)), encoding='utf-8')
    return Path(output)


def save_events_csv(events, output):
    """Export each SLAM event with its full payload for spreadsheet inspection."""
    path = Path(output)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['event_index', 'timestamp', 'type', 'details_json'])
        for index, event in enumerate(events):
            writer.writerow([index, event['timestamp'], event['type'],
                             json.dumps(event, ensure_ascii=False)])
    return path


def save_report(map_file):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    path = Path(map_file)
    data = json.loads(path.read_text(encoding='utf-8'))
    telemetry_file = path.with_name((path.stem[:-4] if path.stem.endswith('_map') else path.stem) + '.json')
    duration_label = 'Elapsed: unavailable'
    if telemetry_file.is_file():
        duration = json.loads(telemetry_file.read_text(encoding='utf-8')).get('duration_sec')
        if isinstance(duration, (int, float)) and math.isfinite(duration) and duration >= 0:
            minutes, seconds = divmod(round(duration), 60)
            duration_label = 'Elapsed: {:.2f} min ({} min {:02d} s)'.format(duration / 60, minutes, seconds)
    size = data['cell_size_m']
    steps = action_steps(data)
    figure, axis = plt.subplots(figsize=(9, 9))
    for row, column in data['visited']:
        axis.add_patch(plt.Rectangle(((column - 0.5) * size, (row - 0.5) * size), size, size,
                                     facecolor='#e4f1f8', edgecolor='#b6c5d0', linewidth=0.5))
    for edge in data['edges']:
        if not edge['wall']:
            continue
        (row1, column1), (row2, column2) = edge['cells']
        row, column = (row1 + row2) * size / 2, (column1 + column2) * size / 2
        if row1 != row2:
            axis.plot([column - size / 2, column + size / 2], [row, row], color='#263747', linewidth=3)
        else:
            axis.plot([column, column], [row - size / 2, row + size / 2], color='#263747', linewidth=3)
    trajectory = data['trajectory']
    if trajectory:
        axis.plot([p['pose'][1] for p in trajectory], [p['pose'][0] for p in trajectory],
                  'o-', color='#1689c1', markersize=3, linewidth=1, label='Estimated trajectory')
    start = data.get('start_pose', [0, 0, 0])
    axis.scatter([start[1]], [start[0]], marker='s', color='#26a269', s=90,
                 label='Start (row, column) {}'.format(tuple(data.get('start_cell', [0, 0]))), zorder=5)
    axis.scatter([data['pose'][1]], [data['pose'][0]], marker='x', color='#e33b35', s=90,
                 label='Last estimated pose', zorder=6)
    if 'map_info' in data:
        axis.set_xlim(-size / 2, (data['map_info']['columns'] - 0.5) * size)
        axis.set_ylim(-size / 2, (data['map_info']['rows'] - 0.5) * size)
        # Plot grid lines at cell boundaries, matching wall coordinates.
        axis.set_xticks([(i - 0.5) * size for i in range(data['map_info']['columns'] + 1)])
        axis.set_yticks([(i - 0.5) * size for i in range(data['map_info']['rows'] + 1)])
        # Major ticks keep the boundary grid; minor ticks label cell indices.
        axis.set_xticklabels([])
        axis.set_yticklabels([])
        axis.set_xticks([column * size for column in range(data['map_info']['columns'])], minor=True)
        axis.set_yticks([row * size for row in range(data['map_info']['rows'])], minor=True)
        axis.set_xticklabels([str(column) for column in range(data['map_info']['columns'])], minor=True)
        axis.set_yticklabels([str(row) for row in range(data['map_info']['rows'])], minor=True)
        axis.tick_params(which='minor', length=0)
        axis.set_axisbelow(True)
    # Cell coordinates are indices (row, column), separate from metre axes.
    if 'map_info' in data:
        cells = ((row, column) for row in range(data['map_info']['rows'])
                 for column in range(data['map_info']['columns']))
    else:
        cells = (tuple(cell) for cell in data['visited'])
    for row, column in cells:
        axis.text((column - 0.40) * size, (row - 0.40) * size, '({},{})'.format(row, column),
                  ha='left', va='bottom', fontsize=10, color='#4b5563', zorder=9,
                  bbox=dict(boxstyle='round,pad=0.15', facecolor='white',
                            edgecolor='none', alpha=0.85))
    labels = {}
    for step in steps:
        if not step['completed']:
            continue
        labels.setdefault(tuple(step['to']), []).append(str(step['step']))
    for (row, column), numbers in labels.items():
        # One clear badge per step, stacked within its cell. Offset from the
        # estimated route and start/end markers rather than printing over them.
        spacing = min(0.19, 0.6 / max(1, len(numbers))) * size
        for index, number in enumerate(numbers):
            vertical = ((len(numbers) - 1) / 2 - index) * spacing
            axis.text((column + 0.18) * size, row * size + vertical, number,
                      ha='center', va='center', fontsize=13, color='#124c72', weight='bold',
                      zorder=10, bbox=dict(boxstyle='round,pad=0.22', facecolor='white',
                                          edgecolor='#87b4d0', linewidth=1))
    axis.set_aspect('equal')
    axis.set_xlabel('Column - increases right')
    axis.set_ylabel('Row - increases upward')
    axis.set_title('Explored map | {} | {} cells\n{}\nCell coordinates = (row, column) | Numbered badges = movement steps (top to bottom)'.format(
        data['status'], len(data['visited']), duration_label))
    axis.legend(loc='upper center', bbox_to_anchor=(0.5, -0.13), ncol=3, fontsize=9)
    axis.grid(which='major', color='#b6c5d0', linewidth=0.7, alpha=0.65)

    figure.tight_layout()
    actions = save_actions_html(data, path.parent / 'actions.html')
    save_events_csv(data['events'], path.parent / 'events.csv')
    plot = path.parent / 'map.png'
    figure.savefig(str(plot), dpi=160)
    plt.close(figure)
    return plot, actions
