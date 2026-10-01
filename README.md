# RoboMaster EP — Explore และ Navigation

โปรเจกต์ควบคุมหุ่น RoboMaster EP เพื่อ **สำรวจสนาม สร้างแผนที่ และเดินไปยิงป้ายที่เลือก** ผ่าน GUI

- **Explore:** สำรวจช่องที่ยังไม่รู้จัก สร้างแผนที่ และตรวจป้ายด้วยกล้อง
- **Navigation:** ใช้แผนที่จาก Explore วางเส้นทางไปยังป้ายที่เลือก แล้วตรวจด้วยกล้องก่อนยิง

## 1. ติดตั้งและเปิดโปรแกรม

ใช้ **Python 3.8**, Tkinter และเครื่องที่มีหน้าจอกราฟิก รันคำสั่งจาก root ของโปรเจกต์:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python main.py
```

หากมี `.venv` อยู่แล้ว ใช้เพียง `source .venv/bin/activate` แล้วรัน `python main.py`

งานที่ใช้กล้องต้องมีตัวถอดรหัสวิดีโอ `libmedia_codec` ของ SDK หรือมี `libh264decoder` และ `opus_decoder` สำหรับ adapter ใน `src/libmedia_codec.py`

## 2. ใช้งาน GUI หลัก

เชื่อมต่อ Wi-Fi ของหุ่น แล้วเลือกงานจากหน้าต่าง `main.py`:

| ต้องการทำอะไร | ปุ่มที่ใช้ |
| --- | --- |
| เช็กว่ากล้องพร้อมใช้งาน | **ตรวจกล้องหุ่นยนต์ (รับภาพ 5 เฟรม)** |
| สำรวจ สร้างแผนที่ และยิงเป้า | เลือกชนิดการยิง สีและรูปร่าง แล้วกด **เริ่มสำรวจ + เล็งยิงเป้าที่เลือก** |
| สำรวจโดยไม่ใช้กล้อง | **สำรวจ SLAM (เฉพาะเดิน ไม่ใช้กล้อง)** |
| ดูภาพพร้อมผลตรวจจับ | **ตรวจจับเป้าหมายจากกล้องสด** หรือ **ตรวจจับเป้าหมายจากเว็บแคม** |
| ทดสอบเล็งและยิงอยู่กับที่ | **ทดสอบเล็งและยิง (หุ่นไม่เดิน)** |
| ตรวจการเดินและเซนเซอร์ | **ทดสอบเดินหน้า 1 ช่อง**, **ทดสอบเลี้ยว**, **ดูเซนเซอร์สด**, **ทดสอบ Gimbal** |
| เก็บค่าคาลิเบรต | เลือกเซนเซอร์ แล้วกด **Calibration เซนเซอร์** |
| ดูกราฟจากรอบก่อน | **วิเคราะห์ Log** |

**Stop** ใช้หยุดงาน, **Esc** ออกจากเต็มจอ และ **F11** สลับเต็มจอ

ตัวเลือก **AP/STA ใน GUI** ใช้กับงานกล้อง สำรวจพร้อมกล้อง และทดสอบยิง ส่วนงานเดิน/เซนเซอร์/Calibration และ Navigation ใช้ `robot.conn_type` ใน `config/settings.yaml`

### ตรวจกล้องก่อนเริ่มงาน

ปุ่มตรวจกล้องจะรับภาพให้ครบ **5 เฟรมภายใน 10 วินาทีหลังเปิดสตรีม** และแสดงผลใน log โดยไม่ขยับหุ่นหรือยิง กด Stop เพื่อยกเลิกได้

ตรวจผ่าน CLI ได้เช่นกัน:

```bash
python -m src.check_robot_camera --conn-type ap
```

### เลือกเป้าที่จะยิง

เลือกสี `Red`, `Yellow`, `Blue`, `Green` และรูปร่าง `Circle`, `Square`, `Vertical_Rect`, `Horizontal_Rect` หรือ `All` ค่าเริ่มต้นคือ **สีแดง ทุกรูปร่าง** และยิงแบบ `water_fire`

ระบบแสดงป้ายอื่นในภาพด้วย แต่ยิงเฉพาะป้ายที่ตรงตัวเลือก เมื่อเล็งและยืนยันเป้าได้จะสั่งยิง **3 นัด** แล้วค้าง Gimbal ตาม `fire.burst_hold_sec` ซึ่งปัจจุบันคือ **1 วินาที**

## 3. เดินไปยังป้ายด้วย Navigation

หลัง Explore พร้อมกล้อง จะได้ไฟล์ `telemetry_logs/run_detect_*/explored_map.json`

เปิดหน้าต่าง Navigation:

```bash
python navigate_targets_gui.py
```

1. **เปิดแผนที่** จากรอบ Explore
2. **เลือกป้าย** ตามสี/รูปร่าง หรือคลิกป้ายบนแผนที่
3. กด **วางเส้นทาง** แล้วกด **จำลอง** เพื่อตรวจเส้นทางได้โดยไม่เชื่อมต่อหุ่น
4. วางหุ่นที่ **ช่องเริ่มต้นของแผนที่** และหันหน้าเหมือนตอนเริ่ม Explore แล้วติ๊กยืนยัน
5. กด **เริ่มเดินและยิง** ระบบจะเดินตามเส้นทางและตรวจป้ายด้วยกล้องก่อนยิง

**หลังเปลี่ยนป้ายที่เลือก ต้องกดวางเส้นทางใหม่** ใช้แผนที่ที่สำรวจไม่ครบได้ หากมีข้อมูลช่องและเส้นทางที่ตรวจสอบได้ ระบบจะเดินผ่านเฉพาะช่องที่สำรวจแล้วและทางที่ยืนยันว่าเปิด

เพิ่มป้ายเองได้โดยติ๊ก **เพิ่มป้ายบนแมพ** แล้วคลิกกำแพงของช่องที่สำรวจแล้ว คลิกขวาเพื่อลบป้ายที่เพิ่มเอง ระบบเก็บสำเนาใน `telemetry_logs/manual_maps/` และยังต้องเห็นป้ายจริงก่อนยิง

ระบุแผนที่หรือเปิดแผนที่บันทึกไว้จาก CLI ได้:

```bash
python navigate_targets_gui.py --map <path/to/explored_map.json>
python navigate_targets_gui.py --plan <path/to/navigation_plan.json>
```

ระหว่างเดิน หากเซนเซอร์ขาดข้อมูล ระบบจะหยุดล้อและพักรอ ก่อนตรวจตำแหน่งเพื่อเดินต่อ หากกล้องขาดเฟรมนานเกินค่าที่ตั้งไว้จะหยุดภารกิจ ป้ายที่ยิงไม่สำเร็จจะถูกบันทึก และผลภารกิจเป็น `partial`

## 4. ปรับค่าการทำงาน

แก้ `config/settings.yaml` แล้วเปิดโปรแกรมใหม่ ทั้ง Explore และ Navigation ใช้ settings และไฟล์คาลิเบรตชุดเดียวกัน

| กลุ่ม | ใช้ปรับ |
| --- | --- |
| `robot`, `map` | AP/STA, ขนาดแผนที่ และช่องเริ่มต้น |
| `navigation`, `pid` | ขนาดช่อง ความเร็ว Explore, Centering, yaw และระยะหยุด |
| `second_pass` | ความเร็วและ Centering ของ Navigation |
| `gimbal`, `fire` | การสแกน มุมเล็ง ค่าชดเชย และเวลาค้างหลังยิง |
| `sensors`, `slam`, `scan_guard` | การรับเซนเซอร์ การสร้างแผนที่ และขีดจำกัดความคลาดเคลื่อน |
| `paths`, `calibration`, `telemetry`, `system` | ตำแหน่งไฟล์ การคาลิเบรต และการบันทึกข้อมูล |

ค่าปัจจุบัน:

| ค่า | ค่าที่ใช้ |
| --- | --- |
| แผนที่ / ช่องเริ่มต้น | 6 × 6 ช่อง / `(0,1)` |
| ขนาดช่อง | 60 × 60 ซม. |
| ความเร็ว Explore / Navigation | 0.425 / 0.25 m/s |
| ความเร็วทดสอบเดินหนึ่งช่อง | 0.22 m/s |
| ความเร็วแก้ด้านข้างสูงสุด Explore / Navigation | 0.04 / 0.05 m/s |
| ระยะหยุดด้านหน้า / ระยะฉุกเฉิน | 200 / 155 มม. |

พิกัดใช้ `(row, column)` โดย `(0,0)` อยู่ซ้ายล่างของแผนที่ แถวเพิ่มขึ้นด้านบน และคอลัมน์เพิ่มไปทางขวา

### Calibration

เลือก **Sharp ซ้าย**, **Sharp ขวา** หรือ **ToF** แล้วกดปุ่ม Calibration กรอกระยะอ้างอิงเป็นมิลลิเมตรตามกล่องข้อความ ปัจจุบันเก็บ **10 ตัวอย่างต่อครั้ง** กด Cancel เพื่อจบการเก็บโดยคงตัวอย่างที่บันทึกแล้วไว้

จากนั้นเลือก **คำนวณสมการ** แล้วกดปุ่มเดิม ผลอยู่ใน `calibration_output/calibration.json` และกราฟของเซนเซอร์ ขั้นคำนวณไม่ต้องเชื่อมต่อหุ่น

## 5. ดูผลการทำงาน

ผลรอบใหม่อยู่ใน `telemetry_logs/` ส่วน `final_logs/` เก็บผลที่คัดไว้แล้ว

| ตำแหน่ง | ไฟล์สำคัญ |
| --- | --- |
| `run_detect_*/` — Explore พร้อมกล้อง | `explored_map.json`, `map.png`, `captured_signs/`, `actions.html`, `events.csv` และ telemetry |
| `runN/` — SLAM และงานทดสอบ | telemetry; งาน SLAM มี `*_map.json`, `map.png` และรายงาน action |
| `navigate_*/` — Navigation | `navigation_plan.json`, `navigation_result.json`, `target_events.json` และ telemetry |
| `fire_tests/` — ทดสอบยิงอยู่กับที่ | ผลยิงแต่ละเป้าใน JSON |
| `gui_session_*.log` | log ของ GUI ทั้ง session |

telemetry มี JSON/CSV สำหรับอ่านและวิเคราะห์ และ `telemetry.jsonl` ที่เขียนระหว่างรัน งานดูเซนเซอร์สดกับทดสอบ Gimbal เก็บเฉพาะ journal นี้ ไม่ export JSON/CSV ส่วนงานตรวจกล้อง 5 เฟรมแสดงผลใน log

Explore ที่สำเร็จจะหยุดที่ช่องสุดท้าย ไม่กลับจุดเริ่ม หากหยุดก่อนจบจะบันทึกแผนที่บางส่วน ส่วน Navigation ดูสถานะล่าสุดและผลแต่ละป้ายได้ใน `navigation_result.json`

## 6. โครงสร้างโปรเจกต์

```text
main.py                  เปิด GUI หลัก
navigate_targets_gui.py  เปิด GUI Navigation
src/                     โมดูล Python ทั้งหมด
config/                  settings.yaml
data/                    ตัวอย่างคาลิเบรต
calibration_output/      สมการและกราฟคาลิเบรต
telemetry_logs/           ผลรอบใหม่
final_logs/              ผลที่คัดเก็บไว้
```

โมดูลที่ใช้แก้ไขงานหลัก:

| งาน | โมดูลใน `src/` |
| --- | --- |
| GUI หลัก / ตรวจกล้อง | `operation_menu.py`, `camera_check.py`, `check_robot_camera.py` |
| ตรวจภาพ / สำรวจพร้อมกล้อง / ยิง | `detect_camera.py`, `slam_detect_camera.py`, `target_fire.py`, `fire_test.py` |
| Navigation | `navigation_gui.py`, `target_navigation.py`, `navigation_controller.py`, `navigation_hardware.py`, `second_pass_control.py`, `navigation_simulation.py` |
| เซนเซอร์ / เดิน / SLAM | `sensor_pipeline.py`, `sensor_filters.py`, `robot_controller.py`, `pid_controller.py`, `grid_slam.py`, `slam_hardware.py` |
| เชื่อมต่อ / คาลิเบรต / รายงาน | `robot_system.py`, `sdk_connection.py`, `libmedia_codec.py`, `calibrate.py`, `telemetry.py`, `run_log.py`, `slam_report.py`, `settings.py` |

ระบบสำรวจใช้ ToF สร้างแผนที่, Sharp ซ้าย–ขวารักษากึ่งกลางทาง และ odometry/yaw ประเมินตำแหน่ง ใช้ BFS เลือกช่องที่ยังไม่สำรวจผ่านทางเปิดที่รู้จัก เหมาะกับสนามกำแพงตามแนวช่องและสนามคงที่

ตรวจ syntax โดยไม่เชื่อมต่อหุ่นได้ด้วย `python -m compileall -q src main.py navigate_targets_gui.py` ส่วนการเดิน สแกน กันชนและยิงต้องตรวจด้วยหุ่นจริงร่วมกับ telemetry
